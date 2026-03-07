#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "MujocoDriver.hpp"
#include "HardwareInterface.hpp"
#include "logger.hpp"
#include "PinocchioModel.hpp"
#include "Controller.hpp"

#include <string>
#include <vector>
#include <memory>
namespace openmanip {

    struct RobotConfig {
        // MuJoCo scene (floor, lights, robot include)
        std::string scene_path;
        // robot only URDF or MJCF for pinocchio kinematics        
        std::string robot_model_path;  
        std::string end_effector_frame;
        std::vector<std::string> locked_joints;
        // -1 = last actuator
        int gripper_actuator_idx = -1; 
    };

    class HardwareInterface;
    class RobotSystem {
        public:
            RobotSystem();
            ~RobotSystem();

            bool initialize(const RobotConfig& config);

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
            bool isGripperOpen() const;
            const std::string& endEffectorFrame() const { return end_effector_frame_; }

            // IK parameter tuning
            void setFrameTaskPositionCost(double cost);
            void setFrameTaskOrientationCost(double cost);
            void setFrameTaskGain(double gain);
            void setFrameTaskLMDamping(double lm_damping);
            void setPostureTaskCost(double cost);
            void setPostureTaskGain(double gain);
            void setPostureTaskLMDamping(double lm_damping);
            void setDampingTaskCost(double cost);
            void setSolverDamping(double damping);
            void setConfigLimitGain(double gain);
            const IKConfig& ikConfig() const;
            bool ikReady() const;

            // Common Getters
            Eigen::Matrix4d getFramePose(std::string frame_name);
            Eigen::VectorXd getJointPositions();

        private:
            mutable Logger log_;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
            std::unique_ptr<Controller> controller_;
            Eigen::VectorXd home_position_;
            bool home_set_ = false;
            double jog_linear_step_ = 0.01; // meter
            double jog_angular_step_ = 0.05; // rad
            std::string end_effector_frame_;
    };
}

#endif // ROBOT_SYSTEM_HPP
