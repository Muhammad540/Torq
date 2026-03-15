#ifndef TORQ_INVERSE_KINEMATICS_HPP
#define TORQ_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "torq/logger.hpp"

namespace torq{

  class Configuration;
  class Task;
  class Limit;
  class Barrier;

  /**
   * @brief Assembled QP data ready to be passed to the solver.
   *
   * Holds the Hessian, gradient, and (optionally) inequality constraints
   * that define the IK optimisation problem for a single tick.
   *
   * @see @ref qp_formulation for how these matrices are built.
   */
  struct QPProblem{
    Eigen::MatrixXd H; ///< Hessian matrix \f$P \in \mathbb{R}^{n_v \times n_v}\f$ (positive semi-definite).
    Eigen::VectorXd c; ///< Linear gradient \f$c \in \mathbb{R}^{n_v}\f$.
    Eigen::MatrixXd G; ///< Inequality constraint matrix \f$G \in \mathbb{R}^{m \times n_v}\f$.
    Eigen::VectorXd h; ///< Inequality upper bound \f$h \in \mathbb{R}^{m}\f$ such that \f$G\,\Delta q \le h\f$.
    bool has_constraints = false; ///< True if at least one limit contributed constraint rows.
  };
  
  /**
   * @brief QP-based differential inverse kinematics solver.
   *
   * Assembles a Quadratic Program from tasks (objective) and limits (constraints),
   * then solves via OSQP.  The solver is warm-started across ticks: the sparsity
   * pattern is set once and only numerical values are updated on subsequent calls.
   *
   * QP form:
   * \f[
   *   \min_{\Delta q}\;\tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q
   *   \quad\text{s.t.}\quad G\,\Delta q \le h
   * \f]
   *
   * Returns a velocity \f$v = \Delta q / \Delta t\f$.
   *
   * @see @ref qp_formulation for the full mathematical derivation.
   * @see Task, Limit, Configuration
   */
  class InverseKinematics{
  public:
    InverseKinematics() = default;
    ~InverseKinematics() = default;

    /**
     * @brief Build the QP from the current configuration and active tasks/limits/barriers.
     *
     * Sums task Hessians and gradients, adds Tikhonov damping, barrier
     * objective contributions, and stacks limit + barrier inequality rows.
     *
     * @param config   Current robot configuration (FK computed).
     * @param tasks    Active tasks contributing to the objective.
     * @param dt       Integration timestep [s].
     * @param damping  Tikhonov (solver) damping \f$\lambda\f$ added to the Hessian diagonal.
     * @param limits   Active limits contributing inequality constraints.
     * @param barriers Active barriers contributing both objective and inequality constraints.
     * @return The assembled QPProblem.
     */
    QPProblem buildIK(const Configuration& config,
		      const std::vector<Task*>& tasks,
		      double dt,
		      double damping = 1e-12,
		      const std::vector<Limit*>& limits = {},
		      const std::vector<Barrier*>& barriers = {});

    /**
     * @brief Build and solve the IK, returning the joint velocity.
     *
     * Calls buildIK() then solves via OSQP.  Integrates the result on the
     * configuration manifold via `pinocchio::integrate`.
     *
     * @param config   Current robot configuration (will be updated in-place on success).
     * @param tasks    Active tasks.
     * @param dt       Integration timestep [s].
     * @param damping  Tikhonov damping \f$\lambda\f$.
     * @param limits   Active limits.
     * @param barriers Active barriers.
     * @return Joint velocity \f$v \in \mathbb{R}^{n_v}\f$, or zero on failure.
     */
    Eigen::VectorXd solve(
			  Configuration& config,
			  const std::vector<Task*>& tasks,
			  double dt,
			  double damping = 1e-12,
			  const std::vector<Limit*>& limits = {},
			  const std::vector<Barrier*>& barriers = {});
    
    
  private:
    bool initSolver(int nv, int m,
                    const Eigen::SparseMatrix<double>& H_sparse,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double>& G_sparse,
                    const Eigen::VectorXd& lower,
                    const Eigen::VectorXd& upper);

    bool updateAndSolve(const Eigen::SparseMatrix<double>& H_sparse,
                        const Eigen::VectorXd& c,
                        const Eigen::SparseMatrix<double>& G_sparse,
                        const Eigen::VectorXd& lower,
                        const Eigen::VectorXd& upper);

    static Eigen::SparseMatrix<double> toDenseSparse(const Eigen::MatrixXd& dense);

    OsqpEigen::Solver solver_;
    bool initialized_ = false;
    int last_nv_ = 0;
    int last_m_  = 0;
    
    mutable Logger log_;
  };

} // namespace torq

#endif // TORQ_INVERSE_KINEMATICS_HPP
