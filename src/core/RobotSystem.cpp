#include "openmanip/RobotSystem.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/PinocchioModel.hpp"

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
	        // step physics 
	        hardware_->step();

	        // get sensor data 
	        Eigen::VectorXd jp = hardware_->getJointPositions();

	        // sync kinematics 
	        kinematics_->update(jp);   

	        // run controller
	        if (controller_){
	          controller_->update();
 	        } else {
	          log.error() << "[RobotSystem] Controller not ready";
	        }
	    } else {
	      log.error() << "[RobotSystem] Hardware or Kinematics Engine not ready";
	    }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return dynamic_cast<MujocoDriver*>(hardware_.get());
    }

    Eigen::Matrix4d RobotSystem::getFramePose(std::string frame_name){
        return kinematics_->getFramePose(frame_name);
    }
}