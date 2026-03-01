#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/PinocchioModel.hpp"
#include <Eigen/Geometry>

#include <iostream>
#include <memory>

namespace openmanip {
    RobotSystem::RobotSystem() {
        hardware_ = std::make_unique<MujocoDriver>();
        kinematics_ = std::make_unique<KinematicsEngine>();
        controller_ = std::make_unique<Controller>(kinematics_.get(), hardware_.get());
    }
    RobotSystem::~RobotSystem() {
        log_.info() << "[RobotSystem] cleaned up";
    }

    bool RobotSystem::initialize(const RobotConfig& config) {
        if (!hardware_) {
            log_.error() << "[RobotSystem] Hardware abstraction not initialized";
            return false;
        }
        if (!hardware_->connect(config.scene_path)) {
            log_.error() << "[RobotSystem] Failed to connect Hardware";
            return false;
        }
        if (!kinematics_->initialize(config.robot_model_path, config.locked_joints)) {
            log_.error() << "[RobotSystem] Failed to initialize the Kinematics Engine";
            return false;
        }

        end_effector_frame_ = config.end_effector_frame;

        auto* physics = getPhysics();
        mjModel* m = physics->getModel();
        mjData*  d = physics->getData();
        int gripper_idx = config.gripper_actuator_idx;
        if (gripper_idx < 0) gripper_idx = m->nu - 1;
        double low  = m->actuator_ctrlrange[2 * gripper_idx];
        double high = m->actuator_ctrlrange[2 * gripper_idx + 1];
        double current = d->ctrl[gripper_idx];
        controller_->setGripperConfig(gripper_idx, high, low, current);
        return true;
    }

    void RobotSystem::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name) {
      if (controller_){
	    controller_->setTaskSpaceTarget(target_pose, frame_name);
      } else {
	    log_.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::setJointSpaceTarget(const Eigen::VectorXd& target_pose){
      if (controller_){
	      controller_->setJointSpaceTarget(target_pose);
      } else {
        log_.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::update() {
       if (hardware_ && kinematics_) {
          // Stepping Simulation
          hardware_->step();
          if (controller_) {
            // updating physics 
            controller_->update();
          }
       }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return dynamic_cast<MujocoDriver*>(hardware_.get());
    }


    Eigen::Matrix4d RobotSystem::getFramePose(std::string frame_name){
      Eigen::VectorXd q_full = hardware_->getJointPositions();
      Eigen::VectorXd q = kinematics_->fullToReducedQ(q_full);
      auto config = kinematics_->makeConfiguration(q);
      return config.getTransformFrameToWorld(frame_name).toHomogeneousMatrix();
    }

    void RobotSystem::setHomePosition(){
      if (hardware_){
        home_position_ = hardware_->getCtrl();
        home_set_ = true;
        log_.info() << "[RobotSystem] Home position saved (" << home_position_.transpose() << ")";
      }
    }

    void RobotSystem::moveToHome() {
      if (!home_set_) {
        log_.warning() << "[RobotSystem] Home position not set";
        return;
      }
      if (controller_) {
	      controller_->setJointSpaceTarget(home_position_);
      }
    }

    bool RobotSystem::hasHomePosition() const {
      return home_set_;
    }

    void RobotSystem::setJogStep(double linear_step, double angular_step){
      jog_linear_step_ = linear_step;
      jog_angular_step_ = angular_step;
    }

    void RobotSystem::jogCartesian(int axis, double sign, const std::string& frame_name) {
      Eigen::Matrix4d pose = getFramePose(frame_name);

      if (axis < 3) {
          pose(axis, 3) += sign * jog_linear_step_;
      } else {
          Eigen::Vector3d rot_axis = Eigen::Vector3d::Zero();
          rot_axis(axis - 3) = 1.0;
          Eigen::AngleAxisd delta(sign * jog_angular_step_, rot_axis);
          pose.block<3,3>(0,0) = delta.toRotationMatrix() * pose.block<3,3>(0,0);
      }

      setTaskSpaceTarget(pose, frame_name);
    }

    void RobotSystem::toggleGripper(){
      if (controller_) controller_->toggleGripper();
    }

    bool RobotSystem::isGripperOpen() const {
      if (controller_) return controller_->isGripperOpen();
      log_.warning() << "[RobotSystem] controller not set up properly";
      return true;
    }

    Eigen::VectorXd RobotSystem::getJointPositions(){
      return hardware_->getJointPositions();
    }
  
}
