#include "openmanip/InverseKinematics.hpp"
#include "openmanip/logger.hpp"

namespace openmanip {
  InverseKinematics::InverseKinematics(KinematicsEngine* kinematics) : kinematics_(kinematics) {}

  InverseKinematics::~InverseKinematics() {}

  Eigen::VectorXd InverseKinematics::solve(const Eigen::VectorXd& q_curr, const Eigen::Matrix4d& target_pose, const std::string& frame_name){
    if (!kinematics_) {
      logger.error() << "[InverseKinematics] Kinematics engine not setup";
      return Eigen::VectorXd::Zero(q_curr.size());
    }
    Eigen::VectorXd err = kinematics_->computeTwistError(frame_name, target_pose);

    Eigen::MatrixXd J = kinematics_->getFrameJacobian(frame_name);

    // Damped Least Square: ref https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html 
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += config_.damping;
    Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

    // [SAFETY ] Clamping Velocities
    float max_vel = (float)dq.lpNorm<Eigen::Infinity>();
    if (max_vel > config_.velocity_limit){
      dq *= (config_.velocity_limit / max_vel);
    }

    return dq;
  }

}
