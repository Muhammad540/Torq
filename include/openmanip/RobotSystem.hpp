#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "MujocoDriver.hpp"
#include "HardwareInterface.hpp"
#include "logger.hpp"
#include "PinocchioModel.hpp"
#include "Controller.hpp"

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

            // Manipulation Utilities
            void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name);
            void setJointSpaceTarget(const Eigen::VectorXd& target_pose);

            // Common Getters
            Eigen::Matrix4d getFramePose(std::string frame_name);
        private:
            Logger log;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
            std::unique_ptr<Controller> controller_;
    };
}

#endif // ROBOT_SYSTEM_HPP
