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
namespace torq {

    /**
     * @brief Configuration for initialising a RobotSystem.
     *
     * Passed to RobotSystem::initialize() to describe the robot's model files,
     * end-effector frame, and joint-locking / gripper setup.
     */
    struct RobotConfig {
        std::string scene_path;              ///< Path to the MuJoCo scene XML (floor, lights, robot include).
        std::string robot_model_path;        ///< Path to the robot-only URDF or MJCF for Pinocchio kinematics.
        std::string end_effector_frame;      ///< Name of the end-effector frame in the Pinocchio model.
        std::vector<std::string> locked_joints; ///< Joint names to freeze (removed from the reduced model).
        int gripper_actuator_idx = -1;       ///< MuJoCo actuator index for the gripper (-1 = last actuator).
    };

    class HardwareInterface;

    /**
     * @brief Top-level user-facing API that owns every subsystem.
     *
     * RobotSystem is the single entry point for all robot operations: physics
     * simulation, kinematics, IK-based control, and GUI interaction.  It owns
     * the HardwareInterface (simulation or real), KinematicsEngine, and Controller.
     *
     * Typical usage:
     * @code
     * torq::RobotConfig cfg;
     * cfg.scene_path         = "scene.xml";
     * cfg.robot_model_path   = "robot.urdf";
     * cfg.end_effector_frame = "ee_link";
     *
     * torq::RobotSystem robot;
     * robot.initialize(cfg);
     *
     * while (running) {
     *     robot.update();
     * }
     * @endcode
     *
     * @see Controller, KinematicsEngine, MujocoDriver
     */
    class RobotSystem {
        public:
            RobotSystem();
            ~RobotSystem();

            /**
             * @brief Initialise the robot from a configuration.
             *
             * Loads the MuJoCo scene, builds the Pinocchio model (with locked joints),
             * creates the Controller, and detects the gripper actuator.
             *
             * @param config  Robot configuration describing model paths and options.
             * @return True on success.
             */
            bool initialize(const RobotConfig& config);

            /**
             * @brief Execute one control + physics tick.
             *
             * Delegates to Controller::update() then steps the physics engine.
             */
            void update();
            
            /**
             * @brief Raw pointer to the MuJoCo driver (for the GUI viewport).
             * @note Intended for internal use by Gui.
             */
            MujocoDriver* getPhysics();

            /** @name Manipulation
             *  @{
             */

            /**
             * @brief Set a Cartesian target and activate task-space IK.
             * @param target_pose  Desired SE(3) as a 4×4 homogeneous matrix.
             * @param frame_name   Name of the frame to regulate.
             */
            void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name);

            /**
             * @brief Set a joint-space target (direct position command).
             * @param target_pose  Desired joint angles (reduced model nq).
             */
            void setJointSpaceTarget(const Eigen::VectorXd& target_pose);

            /** @brief Record the current joint configuration as the home position. */
            void setHomePosition();

            /** @brief Command the robot to move to the stored home position. */
            void moveToHome();

            /** @brief True if a home position has been stored. */
            bool hasHomePosition() const;

            /**
             * @brief Configure the Cartesian jog step sizes.
             * @param linear_step   Step per jog command [m].
             * @param angular_step  Step per jog command [rad].
             */
            void setJogStep(double linear_step, double angular_step);

            /**
             * @brief Jog the end-effector in Cartesian space.
             * @param axis       Axis index (0–5: ±X, ±Y, ±Z, ±Rx, ±Ry, ±Rz).
             * @param sign       Direction (+1 or −1).
             * @param frame_name Frame to jog.
             */
            void jogCartesian(int axis, double sign, const std::string& frame_name);

            /** @brief Current linear jog step [m]. */
            double jogLinearStep() const { return jog_linear_step_; }
            /** @brief Current angular jog step [rad]. */
            double jogAngularStep() const { return jog_angular_step_; }

            /** @brief Toggle gripper between open and closed. */
            void toggleGripper();
            /** @brief True if the gripper is currently open. */
            bool isGripperOpen() const;
            /** @brief Name of the end-effector frame. */
            const std::string& endEffectorFrame() const { return end_effector_frame_; }
            /** @} */

            /** @name IK parameter tuning
             *  Pass-through setters to Controller.  See @ref tuning_guide.
             *  @{
             */
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

            /** @brief Read-only access to the current IK parameters. */
            const IKConfig& ikConfig() const;
            /** @brief True if the IK subsystem has been initialised. */
            bool ikReady() const;
            /** @} */

            /** @name State queries
             *  @{
             */

            /**
             * @brief Get the SE(3) pose of a named frame.
             * @param frame_name  Frame name in the Pinocchio model.
             * @return 4×4 homogeneous transformation matrix.
             */
            Eigen::Matrix4d getFramePose(std::string frame_name);

            /**
             * @brief Get the current joint positions (reduced model).
             * @return Configuration vector \f$q \in \mathbb{R}^{n_q}\f$.
             */
            Eigen::VectorXd getJointPositions();
            /** @} */

        private:
            mutable Logger log_;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
            std::unique_ptr<Controller> controller_;
            Eigen::VectorXd home_position_;
            bool home_set_ = false;
            double jog_linear_step_ = 0.01;
            double jog_angular_step_ = 0.05;
            std::string end_effector_frame_;
    };
}

#endif // ROBOT_SYSTEM_HPP
