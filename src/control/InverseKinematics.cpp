#include "torq/InverseKinematics.hpp"
#include "torq/PinocchioModel.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
#include "torq/Barriers.hpp"
#include "torq/logger.hpp"
#include <OsqpEigen/Compat.hpp>
#include <cmath>

namespace torq {

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
  solver_.settings()->setAbsoluteTolerance(1e-8);
  solver_.settings()->setRelativeTolerance(1e-8);
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
    const Eigen::VectorXd dq = qp.H.ldlt().solve(-qp.c);
    return dq / dt;
  }

  const int m = static_cast<int>(qp.G.rows());

  Eigen::SparseMatrix<double> H_sparse = toDenseSparse(qp.H, true);
  Eigen::SparseMatrix<double> G_sparse = toDenseSparse(qp.G, false);

  Eigen::VectorXd lower = Eigen::VectorXd::Constant(m, -1e30);
  Eigen::VectorXd upper = qp.h;

  OsqpEigen::ErrorExitFlag solve_flag;

  if (!initialized_ || nv != last_nv_ || m != last_m_) {
    solver_.clearSolver();
    if (!initSolver(nv, m, H_sparse, qp.c, G_sparse, lower, upper)) {
      return Eigen::VectorXd::Zero(nv);
    }
    solve_flag = solver_.solveProblem();
  } else {
    solve_flag = updateAndSolve(H_sparse, qp.c, G_sparse, lower, upper);
  }

  if (solve_flag != OsqpEigen::ErrorExitFlag::NoError) {
    log_.error() << "[IK] OSQP did not reach optimality. Commanding zero velocity for safety";
    return Eigen::VectorXd::Zero(nv);
  }

  const Eigen::VectorXd dq = solver_.getSolution();
  return dq / dt;
}

} // namespace torq