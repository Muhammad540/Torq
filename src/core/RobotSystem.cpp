#include "openmanip/RobotSystem.hpp"
#include "openmanip/MujocoDriver.hpp"

#include <iostream>

namespace openmanip {
    RobotSystem::RobotSystem() {
        physics_ = std::make_unique<MujocoDriver>();
    }
    RobotSystem::~RobotSystem() {
        std::cout << "\033[32mRobot System cleaned up\033[0m" << std::endl;
    }

    bool RobotSystem::initialize(const std::string& model_path) {
        if (!physics_) {
            std::cerr << "RobotSystem::initialize() Error: Physics driver not initialized" << std::endl;
            return false;
        }
        if (!physics_->loadModel(model_path)) {
            std::cerr << "RobotSystem::initialize() Error: Failed to load model" << std::endl;
            return false;
        }
        return true;
    }

    void RobotSystem::update() {
        if (physics_) {
            physics_->step();
        }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return physics_.get();
    }
}