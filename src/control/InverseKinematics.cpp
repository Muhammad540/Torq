#include "openmanip/InverseKinematics.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "openmanip/logger.hpp"

namespace openmanip {
  InverseKinematics::InverseKinematics(KinematicsEngine* kinematics) : kinematics_(kinematics) {
    logger.info() << "[InverseKinematics] Initializing ..";
  }

  InverseKinematics::~InverseKinematics() {
    logger.info() << "[InverseKinematics] cleaned up";
  }

  Eigen::VectorXd InverseKinematics::solve(const Eigen::VectorXd& q_curr, const Eigen::Matrix4d& target_pose, const std::string& frame_name){
    if (!kinematics_) {
      logger.error() << "[InverseKinematics] Kinematics engine not setup";
      return Eigen::VectorXd::Zero(q_curr.size());
    }
    kinematics_->update(q_curr);
    Eigen::VectorXd err = kinematics_->computeTwistError(frame_name, target_pose);
    Eigen::MatrixXd J = kinematics_->getFrameJacobian(frame_name);
    Eigen::VectorXd v_des = config_.kp * err;
    // Damped Least Square: ref https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html 
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += config_.damping;
    Eigen::VectorXd qdot = J.transpose() * JJt.ldlt().solve(v_des);

    // [SAFETY ] Clamping Velocities
    const double max_vel = qdot.lpNorm<Eigen::Infinity>();
    if (max_vel > config_.velocity_limit){
      qdot *= (config_.velocity_limit / max_vel);
    }

    return qdot;
  }

}
