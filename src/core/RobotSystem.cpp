#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"
#include "openmanip/PinocchioModel.hpp"

#include <iostream>
#include <memory>

namespace openmanip {
    RobotSystem::RobotSystem() {
        hardware_ = std::make_unique<MujocoDriver>();
        kinematics_ = std::make_unique<KinematicsEngine>();
    }
    RobotSystem::~RobotSystem() {
        log.info() << "[RobotSystem] cleaned up" << std::endl;
    }

    bool RobotSystem::initialize(const std::string& model_path_xml, std::string& model_path_urdf) {
        if (!hardware_) {
            log.error() << "[RobotSystem] Hardware abstraction not initialized" << std::endl;
            return false;
        }
        if (!hardware_->connect(model_path)) {
            log.error() << "[RobotSystem] Failed to connect Hardware" << std::endl;
            return false;
        }
        if (!kinematics_->initialize(model_path_urdf)) {
            log.error() << "[RobotSystem] Failed to initialize the Kinematics Engine" << std::endl;
            return false;
        }
        return true;
    }

    void RobotSystem::update() {
        if (hardware_ && kinematics_) {
            hardware_->step();
            Eigen::VectorXd jp = hardware_->getJointPositions();
            kinematics_->update(jp);   
        }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return dynamic_cast<MujocoDriver*>(hardware_.get());
    }
}