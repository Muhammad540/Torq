#ifndef TORQ_INVERSE_KINEMATICS_HPP
#define TORQ_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "torq/logger.hpp"

namespace torq {

  class Configuration;
  class Task;
  class Limit;
  class Barrier;

  /**
   * @brief Assembled QP data ready to be passed to the solver.
   */
  struct QPProblem {
    Eigen::MatrixXd H;            ///< Symmetric Hessian \f$P \in \mathbb{R}^{n_v \times n_v}\f$.
    Eigen::VectorXd c;            ///< Linear gradient \f$c \in \mathbb{R}^{n_v}\f$.
    Eigen::MatrixXd G;            ///< Inequality matrix \f$G \in \mathbb{R}^{m \times n_v}\f$.
    Eigen::VectorXd h;            ///< Inequality upper bound \f$G \Delta q \le h\f$.
    bool has_constraints = false;
  };

  /**
   * @brief Differential IK via OSQP.
   *
   * Solves
   * \f$\min_{\Delta q}\,\tfrac12 \Delta q^\top P\,\Delta q + c^\top \Delta q\f$
   * s.t. \f$G\,\Delta q \le h\f$ and returns \f$v = \Delta q / \Delta t\f$.
   * When there are no inequality constraints, falls back to a direct LDLT
   * solve. On any non-optimal OSQP exit, returns zero velocity for safety.
   *
   * @see @ref qp_formulation
   */
  class InverseKinematics {
  public:
    InverseKinematics() = default;
    ~InverseKinematics() = default;

    Eigen::VectorXd solve(
        Configuration& config,
        const std::vector<Task*>& tasks,
        double dt,
        double damping = 1e-12,
        const std::vector<Limit*>& limits = {},
        const std::vector<Barrier*>& barriers = {});

  private:
    QPProblem buildIK(const Configuration& config,
                      const std::vector<Task*>& tasks,
                      double dt,
                      double damping = 1e-12,
                      const std::vector<Limit*>& limits = {},
                      const std::vector<Barrier*>& barriers = {});

    bool initSolver(int nv, int m,
                    const Eigen::SparseMatrix<double>& H_sparse,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double>& G_sparse,
                    const Eigen::VectorXd& lower,
                    const Eigen::VectorXd& upper);

    OsqpEigen::ErrorExitFlag updateAndSolve(const Eigen::SparseMatrix<double>& H_sparse,
                                            const Eigen::VectorXd& c,
                                            const Eigen::SparseMatrix<double>& G_sparse,
                                            const Eigen::VectorXd& lower,
                                            const Eigen::VectorXd& upper);

    static Eigen::SparseMatrix<double> toDenseSparse(const Eigen::MatrixXd& dense, bool is_upper_triangular);

    OsqpEigen::Solver solver_;
    bool initialized_ = false;
    int  last_nv_     = 0;
    int  last_m_      = 0;

    Logger log_;
  };

} // namespace torq

#endif // TORQ_INVERSE_KINEMATICS_HPP