#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "MujocoDriver.hpp"
#include "HardwareInterface.hpp"
#include "logger.hpp"
#include "PinocchioModel.hpp"

#include <string>
#include <memory>
namespace openmanip {
    class HardwareInterface;
    class RobotSystem {
        public:
            RobotSystem();
            ~RobotSystem();

            bool initialize(const std::string& model_path_xml, std::string& model_path_urdf);
            // Update Sequence:
            // 1. step physics 
            // 2. get sensor data like joint pose 
            // 3. sync kinematics
            void update();
            
            // NOTE: for visualizer use only
            MujocoDriver* getPhysics();
        private:
            Logger log;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
    };
}

#endif // ROBOT_SYSTEM_HPP