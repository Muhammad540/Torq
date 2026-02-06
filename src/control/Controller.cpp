#include "openmanip/Controller.hpp"
#include "Eigen/src/Core/Matrix.h"

namespace openmanip{
  Controller::Controller(KinematicsEngine* kinematics, HardwareInterface* hardware): kinematics_(kinematics), hardware_(hardware) {
    logger.info() << "[Controller] Initializing ..";
    ik_solver_ = std::make_unique<InverseKinematics>(kinematics_);
  }

  Controller::~Controller() {
    logger.info() << "[Controller] cleaned up";
  }

  void Controller::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, const std::string& frame_name){
    target_pose_ = target_pose;
    end_effector_frame_ = frame_name;
    mode_ = ControlMode::TASK_SPACE;
  }

  void Controller::setJointSpaceTarget(const Eigen::VectorXd& target_joints){
    target_joints_ = target_joints;
    mode_ = ControlMode::JOINT_SPACE;
  }

  void Controller::update(){
    if (!hardware_ || !kinematics_){
      logger.error() << "[Controller] Hardware and Kinematics Engine is not ready";
      return;
    }
    const double dt = hardware_->getTimestep();
    if (mode_ == ControlMode::TASK_SPACE){
      Eigen::VectorXd  q_curr = hardware_->getJointPositions();

      if (task_output_ == TaskSpaceOutput::Velcotiy){
        const Eigen::VectorXd qdot = ik_solver_->solve(q_curr, target_pose_, end_effector_frame_);
        hardware_->setJointVelocities(qdot);
        return;
      }

      constexpr int max_iter = 20;
      for (int i = 0; i < max_iter; ++i){
        const Eigen::VectorXd qdot = ik_solver_->solve(q_curr, target_pose_, end_effector_frame_);
        q_curr = kinematics_->integrate(q_curr, qdot, dt);
      }
      
      hardware_->setJointPositions(q_curr);
      return;
    } else if (mode_ == ControlMode::JOINT_SPACE){
      hardware_->setJointPositions(target_joints_);
      return;
    }
  }
}