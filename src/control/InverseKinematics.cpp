#include "torq/InverseKinematics.hpp"
#include "torq/PinocchioModel.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
#include "torq/logger.hpp"

namespace torq {
  Eigen::SparseMatrix<double> InverseKinematics::toDenseSparse(const Eigen::MatrixXd& dense){
    int rows = static_cast<int>(dense.rows());
    int cols = static_cast<int>(dense.cols());
    Eigen::SparseMatrix<double> sparse(rows, cols);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(rows*cols);
    for (int i=0; i<rows; ++i){
      for (int j=0; j<cols; ++j){
        double val = dense(i, j);
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
       const std::vector<Limit*>& limits){
    int nv = config.nv();
    QPProblem qp;

    // Objective: Tikhonov damping + sum of task contributions
    qp.H = damping * Eigen::MatrixXd::Identity(nv, nv);
    qp.c = Eigen::VectorXd::Zero(nv);

    for (const auto* task: tasks){
      auto [H_task, c_task] = task->computeQPObjective(config);
      qp.H += H_task;
      qp.c += c_task;
    }

    // Inequality constraints from limits
    std::vector<Eigen::MatrixXd> G_blocks;
    std::vector<Eigen::VectorXd> h_blocks;
    int total_rows = 0;

    for (const auto* limit: limits){
      auto result = limit->computeQPInequalities(config, dt);
      if (result.has_value()){
        G_blocks.push_back(result->first);
        h_blocks.push_back(result->second);
        total_rows += static_cast<int>(result->first.rows());
      }
    }

    if (total_rows > 0){
      qp.G.resize(total_rows, nv);
      qp.h.resize(total_rows);
      int row = 0;
      for (size_t i=0; i<G_blocks.size(); ++i){
        int block_rows = static_cast<int>(G_blocks[i].rows());
        qp.G.middleRows(row, block_rows) = G_blocks[i];
        qp.h.segment(row, block_rows) = h_blocks[i];
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
					 const Eigen::VectorXd& upper){
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setAbsoluteTolerance(1e-5);
    solver_.settings()->setRelativeTolerance(1e-5);
    solver_.settings()->setMaxIteration(200);
    solver_.settings()->setPolish(false);

    solver_.data()->setNumberOfVariables(nv);
    solver_.data()->setNumberOfConstraints(m);

    if (!solver_.data()->setHessianMatrix(H_sparse)) return false;
    if (!solver_.data()->setGradient(const_cast<Eigen::VectorXd&>(c))) return false;
    if (!solver_.data()->setLinearConstraintsMatrix(G_sparse)) return false;
    if (!solver_.data()->setLowerBound(const_cast<Eigen::VectorXd&>(lower))) return false;
    if (!solver_.data()->setUpperBound(const_cast<Eigen::VectorXd&>(upper))) return false;

    if (!solver_.initSolver()) return false;

    last_nv_ = nv;
    last_m_ = m;
    initialized_ = true;
    return true;
  }

  bool InverseKinematics::updateAndSolve(
    const Eigen::SparseMatrix<double>& H_sparse,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& G_sparse,
    const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper){

    if (!solver_.updateHessianMatrix(H_sparse)) return false;
    if (!solver_.updateGradient(c)) return false;
    if (!solver_.updateLinearConstraintsMatrix(G_sparse)) return false;
    if (!solver_.updateBounds(lower, upper)) return false;

    return solver_.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
  }

  Eigen::VectorXd InverseKinematics::solve(
    Configuration& config,
    const std::vector<Task*>& tasks,
    double dt,
    double damping,
    const std::vector<Limit*>& limits){

    int nv = config.nv();
    if (nv == 0) return {};

    QPProblem qp = buildIK(config, tasks, dt, damping, limits);

    if (!qp.has_constraints) {
        Eigen::VectorXd dq = qp.H.ldlt().solve(-qp.c);
        return dq / dt;
    }

    int m = static_cast<int>(qp.G.rows());

    Eigen::SparseMatrix<double> H_sparse = toDenseSparse(qp.H);
    Eigen::SparseMatrix<double> G_sparse = toDenseSparse(qp.G);

    Eigen::VectorXd lower = Eigen::VectorXd::Constant(m, -1e30);
    Eigen::VectorXd upper = qp.h;

    bool ok = false;

    if (!initialized_ || nv != last_nv_ || m != last_m_) {
        solver_.clearSolver();
        ok = initSolver(nv, m, H_sparse, qp.c, G_sparse, lower, upper);
        if (!ok) {
            log_.error() << "[IK] OSQP init failed";
            return Eigen::VectorXd::Zero(nv);
        }
        ok = solver_.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
    } else {
        ok = updateAndSolve(H_sparse, qp.c, G_sparse, lower, upper);
    }

    if (!ok) {
        log_.warning() << "[IK] OSQP solve failed, falling back to unconstrained";
        Eigen::VectorXd dq = qp.H.ldlt().solve(-qp.c);
        return dq / dt;
    }

    Eigen::VectorXd dq = solver_.getSolution();
    return dq / dt;
  }
}
