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
        log.info() << "[RobotSystem] cleaned up";
    }

    bool RobotSystem::initialize(const std::string& model_path_xml, std::string& model_path_urdf) {
        if (!hardware_) {
            log.error() << "[RobotSystem] Hardware abstraction not initialized";
            return false;
        }
        if (!hardware_->connect(model_path_xml)) {
            log.error() << "[RobotSystem] Failed to connect Hardware";
            return false;
        }
        if (!kinematics_->initialize(model_path_urdf)) {
            log.error() << "[RobotSystem] Failed to initialize the Kinematics Engine";
            return false;
        }
        return true;
    }

    void RobotSystem::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name) {
      if (controller_){
	    controller_->setTaskSpaceTarget(target_pose, frame_name);
      } else {
	    log.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::setJointSpaceTarget(const Eigen::VectorXd& target_pose){
      if (controller_){
	      controller_->setJointSpaceTarget(target_pose);
      } else {
        log.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::update() {
       if (hardware_ && kinematics_) {
          hardware_->step();
          if (controller_) {
            controller_->update();
          }
       }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return dynamic_cast<MujocoDriver*>(hardware_.get());
    }

      Eigen::Matrix4d RobotSystem::getFramePose(std::string frame_name){
      Eigen::VectorXd q = hardware_->getJointPositions();
      auto config = kinematics_->makeConfiguration(q);
      return config.getTransformFrameToWorld(frame_name).toHomogeneousMatrix();
    }

    void RobotSystem::setHomePosition(){
      if (hardware_){
        home_position_ = hardware_->getJointPositions();
        home_set_ = true;
        log.info() << "[RobotSystem] Home position saved (" << home_position_.transpose() << ")";
      }
    }

    void RobotSystem::moveToHome() {
      if (!home_set_) {
        log.warning() << "[RobotSystem] Home position not set";
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

    void RobotSystem::setGripperActuator(int actuator_idx) {
      gripper_actuator_idx_ = actuator_idx;
    }

    void RobotSystem::toggleGripper(){
      auto* physics = getPhysics();
      if (!physics) return;

      mjModel* m = physics->getModel();
      mjData* d = physics->getData();
      if (!m || !d || m->nu == 0) return;

      if (gripper_actuator_idx_ < 0) {
        gripper_actuator_idx_ = m->nu - 1;
      }

      gripper_open_ = !gripper_open_;

      double low  = m->actuator_ctrlrange[2 * gripper_actuator_idx_];
      double high = m->actuator_ctrlrange[2 * gripper_actuator_idx_ + 1];
      d->ctrl[gripper_actuator_idx_] = gripper_open_ ? high : low;
    }

    Eigen::VectorXd RobotSystem::getJointPositions(){
      return hardware_->getJointPositions();
    }
  
}
