#include "openmanip/Controller.hpp"

namespace openmanip{
  Controller::Controller(KinematicsEngine* kinematics, HardwareInterface* hardware): kinematics_(kinematics), hardware_(hardware) {
    ik_solver_ = std::make_unique<InverseKinematics>(kinematics_);
  }

  Controller::~Controller() {}

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
    }

      if (mode_ == ControlMode::TASK_SPACE){
        Eigen::VectorXd  q_curr = hardware_->getJointPositions();

        Eigen::VectorXd dq = ik_solver_->solve(q_curr, target_pose_, end_effector_frame_);

        double dt = hardware_->getTimestep();
        Eigen::VectorXd q_cmd = kinematics_->integrate(q_curr, dq, dt);

        hardware_->overrideJointPositions(q_cmd);
        hardware_->overrideJointVelocities(dq);
      } else if (mode_ == ControlMode::JOINT_SPACE){
        hardware_->overrideJointVelocities(target_joints_);
      }
  }
}