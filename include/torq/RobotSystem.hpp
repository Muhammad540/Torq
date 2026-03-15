#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "MujocoDriver.hpp"
#include "HardwareInterface.hpp"
#include "logger.hpp"
#include "PinocchioModel.hpp"
#include "Controller.hpp"
#include "Tasks.hpp"
#include "Limits.hpp"
#include "Barriers.hpp"

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
        /** Max Cartesian tracking error [m] for jog safety; target is clamped within this distance of actual EE. Default 0.05. */
        double max_tracking_error = 0.05;
        /** Target control loop frequency [Hz] for update(); caller should invoke update() at this rate. Typical 200–1000 for IK. Default 500. */
        double control_frequency_hz = 500.0;
    };

    class HardwareInterface;

    /**
     * @brief Top-level user-facing API and sole owner of all main components.
     *
     * **Ownership hierarchy**
     *
     * RobotSystem is the single entry point for library users. It owns:
     * - HardwareInterface (simulation or real hardware)
     * - KinematicsEngine (Pinocchio model and configurations)
     * - Controller (control modes, built-in IK tasks/limits, QP solver)
     * - User-added tasks, limits, and barriers (when added via addTask/addLimit/addBarrier)
     *
     * The Controller is an internal implementation detail: it owns the
     * InverseKinematics solver and built-in tasks/limits (FrameTask, PostureTask,
     * DampingTask, VelocityLimit, ConfigurationLimit, etc.). Users do not
     * access the Controller or IK solver directly; all interaction is through
     * RobotSystem.
     *
     * Control loop: call update() at the configured control_frequency_hz
     * (200–1000 Hz typical). The caller is responsible for rate limiting.
     *
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
     *     rate_limiter.sleep();
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
             * Runs at the rate of the caller; typically invoked at control_frequency_hz
             * (200–1000 Hz). Performs: physics step, then Controller::update() (IK solve
             * and joint command). Call at a fixed rate for deterministic behaviour.
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
             *
             * Also updates the persistent target used by jogCartesian, so programmatic
             * moves and jog stay in sync.
             *
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

            /**
             * @brief Sync the persistent task-space target to the current end-effector pose.
             *
             * Call after the robot has been moved outside task-space IK (e.g. Move to Home,
             * joint sliders, or external motion) so that the next jog does not see a huge
             * error and command a velocity spike. moveToHome() already resets target via
             * setJointSpaceTarget(); use this when the robot was moved by other means.
             *
             * @param frame_name Frame whose current pose becomes the new persistent target.
             */
            void resetTaskSpaceTargetToCurrentPose(const std::string& frame_name);

            /** @brief Current linear jog step [m]. */
            double jogLinearStep() const { return jog_linear_step_; }
            /** @brief Current angular jog step [rad]. */
            double jogAngularStep() const { return jog_angular_step_; }

            /** @brief Set max Cartesian tracking error [m] for jog safety clamp. */
            void setMaxTrackingError(double max_error);
            /** @brief Current max tracking error [m]. */
            double maxTrackingError() const { return max_tracking_error_; }

            /** @brief Set target control loop frequency [Hz] (caller should call update() at this rate). */
            void setControlFrequencyHz(double hz);
            /** @brief Current target control frequency [Hz]. */
            double controlFrequencyHz() const { return control_frequency_hz_; }
            /** @brief Control period [s] = 1 / controlFrequencyHz(), for rate limiting. */
            double controlPeriodSec() const { return 1.0 / control_frequency_hz_; }

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

            /** @name User task / limit / barrier composition
             *  RobotSystem owns all objects added here. They are included in
             *  every IK solve alongside the built-in tasks/limits. Use for
             *  humanoids, quadrupeds, or custom constraints.
             *  @{
             */

            /**
             * @brief Add a task; RobotSystem takes ownership.
             * @param task  Task to add (e.g. ComTask, RelativeFrameTask). Must be heap-allocated.
             */
            void addTask(std::unique_ptr<Task> task);

            /** @brief Remove a task by pointer. It is destroyed and no longer used in IK. */
            void removeTask(Task* task);

            /**
             * @brief Add a limit; RobotSystem takes ownership.
             * @param limit  Limit to add (e.g. FloatingBaseVelocityLimit, AccelerationLimit).
             */
            void addLimit(std::unique_ptr<Limit> limit);

            /** @brief Remove a limit by pointer. It is destroyed and no longer used in IK. */
            void removeLimit(Limit* limit);

            /**
             * @brief Add a barrier; RobotSystem takes ownership.
             * @param barrier  Barrier to add (e.g. PositionBarrier, BodySphericalBarrier).
             */
            void addBarrier(std::unique_ptr<Barrier> barrier);

            /** @brief Remove a barrier by pointer. It is destroyed and no longer used in IK. */
            void removeBarrier(Barrier* barrier);
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

            /// User-added tasks/limits/barriers (RobotSystem owns these)
            std::vector<std::unique_ptr<Task>> user_tasks_;
            std::vector<std::unique_ptr<Limit>> user_limits_;
            std::vector<std::unique_ptr<Barrier>> user_barriers_;

            Eigen::VectorXd home_position_;
            bool home_set_ = false;
            double jog_linear_step_ = 0.01;
            double jog_angular_step_ = 0.05;
            std::string end_effector_frame_;

            Eigen::Matrix4d persistent_target_pose_;
            bool target_initialized_ = false;
            std::string last_jog_frame_;
            double max_tracking_error_ = 0.05;
            double control_frequency_hz_ = 500.0;
    };
}

#endif // ROBOT_SYSTEM_HPP
