#ifndef OPENMANIP_INVERSE_KINEMATICS_HPP
#define OPENMANIP_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <string>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "openmanip/logger.hpp"

namespace openmanip{

  class Configuration;
  class Task;
  class Limit;

  struct QPProblem{
    Eigen::MatrixXd H; // hessian matrix of shape (nv x nv)   
    Eigen::VectorXd c; // linear objective (nv)
    Eigen::MatrixXd G; // inequality constraint matrix (m x nv)
    Eigen::VectorXd h; // inequality constraint upper bound (m)          
    bool has_constraints = false;
  };
  
  /*
    QP based diff inverse kinematics solver
    
    Assemble a QP from tasks (objective) and limits (constraints)
    then solves via QSQP.
    
    QP form:
        min  0.5 * dq^T H dq + c^T dq
        s.t. G dq <= h

    Returns velocity v = dq / dt.
  */
  class InverseKinematics{
  public:
    InverseKinematics() = default;
    ~InverseKinematics() = default;

    /*
     Build the IK quadratic program from tasks and limits. 
     */
    QPProblem buildIK(const Configuration& config,
		      const std::vector<Task*>& tasks,
		      double dt,
		      double damping = 1e-12,
		      const std::vector<Limit*>& limits = {});

    /*
     Solve the IK and return the velocity in the tangent space. Returns zero velocity in failure.
     */
    Eigen::VectorXd solve(
			  Configuration& config,
			  const std::vector<Task*>& tasks,
			  double dt,
			  double damping = 1e-12,
			  const std::vector<Limit*>& limits = {});
    
    
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

} // namespace openmanip

#endif // OPENMANIP_INVERSE_KINEMATICS_HPP
