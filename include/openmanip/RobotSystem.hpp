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

            void update();
            
            // NOTE: for visualizer use only
            MujocoDriver* getPhysics();

            // Manipulation Utilities
            void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name);
            void setJointSpaceTarget(const Eigen::VectorXd& target_pose);
            void setHomePosition();
            void moveToHome();
            bool hasHomePosition() const;
            void setJogStep(double linear_step, double angular_step);
            void jogCartesian(int axis, double sign, const std::string& frame_name);
            double jogLinearStep() const { return jog_linear_step_; }
            double jogAngularStep() const { return jog_angular_step_; }
            void toggleGripper();
            bool isGripperOpen() const { return gripper_open_; }
            void setGripperActuator(int actuator_idx);
            // Common Getters
            Eigen::Matrix4d getFramePose(std::string frame_name);
            Eigen::VectorXd getJointPositions();

        private:
            Logger log;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
            std::unique_ptr<Controller> controller_;
            Eigen::VectorXd home_position_;
            bool home_set_ = false;
            double jog_linear_step_ = 0.01; // meter
            double jog_angular_step_ = 0.05; // rad
            bool gripper_open_ = true;
            int gripper_actuator_idx_ = -1;
    };
}

#endif // ROBOT_SYSTEM_HPP
