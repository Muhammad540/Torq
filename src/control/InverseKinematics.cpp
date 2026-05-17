#include "torq/InverseKinematics.hpp"
#include "torq/PinocchioModel.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
#include "torq/Barriers.hpp"
#include "torq/logger.hpp"
#include <OsqpEigen/Compat.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace torq {

namespace {
// Tolerance for constraint slack, if below this, return zero velocity
constexpr double kConstraintSlackTol = 1e-4;

const char* osqpFlagName(OsqpEigen::ErrorExitFlag flag) {
  switch (flag) {
    case OsqpEigen::ErrorExitFlag::NoError: return "NoError";
    case OsqpEigen::ErrorExitFlag::DataValidationError: return "DataValidationError";
    case OsqpEigen::ErrorExitFlag::SettingsValidationError: return "SettingsValidationError";
    default: return "Unknown";
  }
}

/*
  QP is stacked as G * dq <= h (limits, barriers)
  For each row i, the slack (how much room is left before violating the inequality is):
  slack_i = h_i - G_i * dq

  Positive slack: constraint satisfied with a margin 
  Zero: on the boundary
  Negative: violated

  minConstraintSlack returns the minimum slack across all constraints
*/
double minConstraintSlack(const QPProblem& qp, const Eigen::VectorXd& dq) {
  const int m = static_cast<int>(qp.G.rows());
  double slack_min = std::numeric_limits<double>::infinity();
  for (int i = 0; i < m; ++i) {
    slack_min = std::min(slack_min, qp.h(i) - qp.G.row(i).dot(dq));
  }
  return slack_min;
}

/*
  After slack check, if any commanded velocity is above the model velocity limit, clamp it to the limit.
*/
Eigen::VectorXd clampDeltaQByVelocityLimit(const Eigen::VectorXd& dq,
                                           const pinocchio::Model& model,
                                           double dt) {
  Eigen::VectorXd out = dq;
  for (int i = 0; i < model.nv; ++i) {
    const double v_lim = model.velocityLimit[i];
    if (!std::isfinite(v_lim) || v_lim >= 1e20 || v_lim <= 1e-10) continue;
    const double dq_max = dt * v_lim;
    out[i] = std::clamp(out[i], -dq_max, dq_max);
  }
  return out;
}

} // namespace

Eigen::SparseMatrix<double> InverseKinematics::toDenseSparse(const Eigen::MatrixXd& dense, bool is_upper_triangular) {
  const int rows = static_cast<int>(dense.rows());
  const int cols = static_cast<int>(dense.cols());
  Eigen::SparseMatrix<double> sparse(rows, cols);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(rows) * static_cast<size_t>(cols));
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      // OSQP requires Hessian to be upper triangular (row <= col).
      if (is_upper_triangular && i > j) continue;
      const double val = dense(i, j);
      // to maintain the sparsity pattern identical across update() calls.
      triplets.emplace_back(i, j, val == 0.0 ? 1e-20 : val);
    }
  }
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  sparse.makeCompressed();
  return sparse;
}

QPProblem InverseKinematics::buildIK(
    const Configuration& config,
    const std::vector<Task*>& tasks,
    double dt,
    double damping,
    const std::vector<Limit*>& limits,
    const std::vector<Barrier*>& barriers) {

  const int nv = config.nv();
  QPProblem qp;

  // Objective: Tikhonov damping + sum of task contributions.
  qp.H = damping * Eigen::MatrixXd::Identity(nv, nv);
  qp.c = Eigen::VectorXd::Zero(nv);

  for (const auto* task : tasks) {
    auto [H_task, c_task] = task->computeQPObjective(config);
    qp.H += H_task;
    qp.c += c_task;
  }

  // Symmetrize H 
  qp.H = 0.5 * (qp.H + qp.H.transpose());

  // Inequality constraints from limits.
  std::vector<Eigen::MatrixXd> G_blocks;
  std::vector<Eigen::VectorXd> h_blocks;
  int total_rows = 0;

  for (const auto* limit : limits) {
    auto result = limit->computeQPInequalities(config, dt);
    if (result.has_value()) {
      G_blocks.push_back(result->first);
      h_blocks.push_back(result->second);
      total_rows += static_cast<int>(result->first.rows());
    }
  }

  for (auto* barrier : barriers) {
    barrier->updateQP(config, dt);

    // Add objective contributions (Safe Displacement)
    qp.H += barrier->H();
    qp.c += barrier->c();

    // Add inequality constraints
    G_blocks.push_back(barrier->G());
    h_blocks.push_back(barrier->h());
    total_rows += barrier->dim();
  }

  if (total_rows > 0) {
    qp.G.resize(total_rows, nv);
    qp.h.resize(total_rows);
    int row = 0;
    for (size_t i = 0; i < G_blocks.size(); ++i) {
      const int block_rows = static_cast<int>(G_blocks[i].rows());
      qp.G.middleRows(row, block_rows) = G_blocks[i];
      qp.h.segment(row, block_rows)    = h_blocks[i];
      row += block_rows;
    }
    qp.has_constraints = true;
  }
  return qp;
}

bool InverseKinematics::initSolver(
    int nv, int m,
    const Eigen::SparseMatrix<double>& H_sparse,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& G_sparse,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper) {

  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setAbsoluteTolerance(1e-5);
  solver_.settings()->setRelativeTolerance(1e-5);
  solver_.settings()->setMaxIteration(200);
  solver_.settings()->setPolish(false);

  solver_.data()->setNumberOfVariables(nv);
  solver_.data()->setNumberOfConstraints(m);

  Eigen::VectorXd c_local     = c;
  Eigen::VectorXd lower_local = lower;
  Eigen::VectorXd upper_local = upper;

  if (!solver_.data()->setHessianMatrix(H_sparse))            return false;
  if (!solver_.data()->setGradient(c_local))                  return false;
  if (!solver_.data()->setLinearConstraintsMatrix(G_sparse))  return false;
  if (!solver_.data()->setLowerBound(lower_local))            return false;
  if (!solver_.data()->setUpperBound(upper_local))            return false;

  if (!solver_.initSolver()) return false;

  last_nv_     = nv;
  last_m_      = m;
  initialized_ = true;
  return true;
}

OsqpEigen::ErrorExitFlag InverseKinematics::updateAndSolve(
    const Eigen::SparseMatrix<double>& H_sparse,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& G_sparse,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper) {

  if (!solver_.updateHessianMatrix(H_sparse))
    return OsqpEigen::ErrorExitFlag::DataValidationError;
  if (!solver_.updateGradient(c))
    return OsqpEigen::ErrorExitFlag::DataValidationError;
  if (!solver_.updateLinearConstraintsMatrix(G_sparse))
    return OsqpEigen::ErrorExitFlag::DataValidationError;
  if (!solver_.updateBounds(lower, upper))
    return OsqpEigen::ErrorExitFlag::DataValidationError;

  return solver_.solveProblem();
}

Eigen::VectorXd InverseKinematics::solve(
    Configuration& config,
    const std::vector<Task*>& tasks,
    double dt,
    double damping,
    const std::vector<Limit*>& limits,
    const std::vector<Barrier*>& barriers) {

  const int nv = config.nv();
  if (nv == 0) return {};

  QPProblem qp = buildIK(config, tasks, dt, damping, limits, barriers);

  if (!qp.has_constraints) {
    Eigen::VectorXd dq = qp.H.ldlt().solve(-qp.c);
    dq = clampDeltaQByVelocityLimit(dq, config.model(), dt);
    return dq / dt;
  }

  const int m = static_cast<int>(qp.G.rows());

  Eigen::SparseMatrix<double> H_sparse = toDenseSparse(qp.H, true);
  Eigen::SparseMatrix<double> G_sparse = toDenseSparse(qp.G, false);

  Eigen::VectorXd lower = Eigen::VectorXd::Constant(m, -1e30);
  Eigen::VectorXd upper = qp.h;

  OsqpEigen::ErrorExitFlag solve_flag;
  const bool reinit = !initialized_ || nv != last_nv_ || m != last_m_;

  if (reinit) {
    solver_.clearSolver();
    if (!initSolver(nv, m, H_sparse, qp.c, G_sparse, lower, upper)) {
      log_.error() << "[IK] OSQP initSolver failed (nv=" << nv << " m=" << m << ")";
      return Eigen::VectorXd::Zero(nv);
    }
    solve_flag = solver_.solveProblem();
  } else {
    solve_flag = updateAndSolve(H_sparse, qp.c, G_sparse, lower, upper);
  }

  if (solve_flag != OsqpEigen::ErrorExitFlag::NoError) {
    log_.error() << "[IK] OSQP " << osqpFlagName(solve_flag)
                 << " (nv=" << nv << " m=" << m << " dt=" << dt
                 << "). Commanding zero velocity.";
    return Eigen::VectorXd::Zero(nv);
  }

  Eigen::VectorXd dq = solver_.getSolution();
  const double slack_min = minConstraintSlack(qp, dq);
  if (slack_min < -kConstraintSlackTol) {
    static int slack_warn_count = 0;
    if (slack_warn_count++ % 200 == 0) {
      log_.warning() << "[IK] Primal violates G*dq<=h (min slack=" << slack_min<< "); commanding zero velocity.";
    }
    return Eigen::VectorXd::Zero(nv);
  }

  dq = clampDeltaQByVelocityLimit(dq, config.model(), dt);
  return dq / dt;
}

} // namespace torq