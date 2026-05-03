#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "HardwareInterface.hpp"
#include "MujocoDriver.hpp"
#include "ServoDriver.hpp"
#include "logger.hpp"
#include "PinocchioModel.hpp"
#include "Controller.hpp"
#include "Tasks.hpp"
#include "Limits.hpp"
#include "Barriers.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace torq {

    /**
     * @brief Configuration for initialising a RobotSystem.
     *
     * Passed to RobotSystem::initialize() to describe the robot's model files,
     * end-effector frame, and joint-locking / gripper setup.
     */
    struct RobotConfig {
        std::string scene_path;              ///< Path to the MuJoCo scene XML (floor, lights, robot include). Used when driver_type is "mujoco".
        std::string robot_model_path;        ///< Path to the robot-only URDF or MJCF for Pinocchio kinematics.
        std::string end_effector_frame;      ///< Name of the end-effector frame in the Pinocchio model.
        std::vector<std::string> locked_joints; ///< Joint names to freeze (removed from the reduced model).
        int gripper_actuator_idx = -1;       ///< MuJoCo actuator index for the gripper (-1 = last actuator).
        /** Max Cartesian tracking error [m] for jog safety; target is clamped within this distance of actual EE. Default 0.05. */
        double max_tracking_error = 0.05;

        /** Driver for hardware: "mujoco" (simulation) or "serial_servo" (real robot, e.g. ST3215 servo). Default "mujoco". */
        std::string driver_type = "mujoco";
        /** Connection for real robot: path to calib file (containing port, baud_rate, and calibration data). Used when driver_type is "serial_servo". */
        std::string robot_calib_file;
        /** When true (default), send position commands to real hardware. When false (passive mode), only read positions and mirror to display — e.g. move by hand and observe in GUI. Ignored when driver_type is "mujoco". */
        bool active_control = true;
        /** If set (rad/s, finite and > 0), assigns this limit to every velocity DoF of the reduced Pinocchio model after load (overrides parser defaults, e.g. unbounded MJCF). */
        std::optional<double> joint_velocity_limit_rad_s;
    };

    class HardwareInterface;

    /**
     * @brief Single entry point for the library.
     *
     * Owns the HardwareInterface, KinematicsEngine, Controller, and any
     * user-added tasks/limits/barriers. The Controller and InverseKinematics
     * are internal: all manipulation, IK tuning, and the per-tick update go
     * through RobotSystem.
     *
     * Call `update()` once per control iteration. Each call advances the
     * driver by one `getTimestep()` (MuJoCo `opt.timestep` or the servo
     * control period), independent of how often you call `update()`.
     *
     * @code
     * torq::RobotConfig cfg;
     * cfg.scene_path         = "scene.xml";
     * cfg.robot_model_path   = "robot.urdf";
     * cfg.end_effector_frame = "ee_link";
     *
     * torq::RobotSystem robot;
     * robot.initialize(cfg);
     * while (running) robot.update();
     * @endcode
     *
     * @see Controller, KinematicsEngine, HardwareInterface
     */
    class RobotSystem {
        public:
            RobotSystem();
            ~RobotSystem();

            /**
             * @brief Initialise the robot from a configuration.
             *
             * Allocates the hardware driver from config.driver_type, connects it,
             * builds the Pinocchio model (with locked joints), and detects the gripper actuator.
             * For serial_servo, optionally loads a MuJoCo display mirror when scene_path is set.
             *
             * @param config  Robot configuration describing model paths and options.
             * @return True on success.
             */
            bool initialize(const RobotConfig& config);

            /**
             * @brief Execute one control + physics tick.
             *
             * Steps the hardware then (if active control is on) runs one IK
             * solve via the Controller. The IK integration timestep is
             * `hardware_->getTimestep()`, not the wall-clock interval between
             * calls to `update()`.
             */
            void update();

            /**
             * @brief MuJoCo driver used for visualization (rendering / introspection).
             *
             * - Simulation (`driver_type == "mujoco"`): the same driver that
             *   steps physics.
             * - Real hardware with `scene_path` set: a display-only mirror
             *   driven from real joint positions (never stepped).
             * - Real hardware without `scene_path`: `nullptr`.
             */
            MujocoDriver* mujocoVisualizationDriver();

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

            /** @brief Set the home position. */
            void setHomePosition(const Eigen::VectorXd& home_position);

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
             * @brief Snap the cached task-space target to the current frame pose.
             *
             * Call after the robot has been moved outside task-space IK (e.g.
             * via joint sliders) so the next jog does not see a stale error.
             *
             * @param frame_name Frame whose current pose becomes the new target.
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

            /**
             * @brief Load collision geometry into the kinematics engine.
             *
             * Forwards to KinematicsEngine::loadCollisionModel(). Call after
             * initialize(). Accepts URDF or MJCF (detected by extension).
             *
             * @param model_path Path to the URDF or MJCF with collision geometry.
             * @param srdf_path  Optional SRDF to filter collision pairs.
             * @return True on success.
             */
            bool loadCollisionModel(const std::string& model_path,
                                    const std::string& srdf_path = "");

            /** @name User-added tasks, limits and barriers
             *  RobotSystem takes ownership and includes them in every IK
             *  solve alongside the built-in tasks/limits.
             *  @{
             */

            /** @brief Add a task; RobotSystem takes ownership. */
            void addTask(std::unique_ptr<Task> task);
            /** @brief Remove and destroy a previously added task. */
            void removeTask(Task* task);

            /** @brief Add a limit; RobotSystem takes ownership. */
            void addLimit(std::unique_ptr<Limit> limit);
            /** @brief Remove and destroy a previously added limit. */
            void removeLimit(Limit* limit);

            /** @brief Add a barrier (e.g. PositionBarrier, BodySphericalBarrier). */
            void addBarrier(std::unique_ptr<Barrier> barrier);
            /** @brief Remove and destroy a previously added barrier. */
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

            /**
             * @brief Get the current joint velocities.
             * @return Velocity vector \f$\dot{q} \in \mathbb{R}^{n_v}\f$.
             *         For real hardware this is a finite-difference estimate.
             */
            Eigen::VectorXd getJointVelocities();
            /** @} */

        private:
            mutable Logger log_;
            std::unique_ptr<HardwareInterface> hardware_;
            std::unique_ptr<KinematicsEngine> kinematics_;
            std::unique_ptr<Controller> controller_;

            /// Optional MuJoCo scene that mirrors real joint positions for the GUI only
            /// (`driver_type == "serial_servo"` and non-empty `scene_path`). Not stepped for
            /// dynamics; exposed alongside the sim driver through mujocoVisualizationDriver().
            std::unique_ptr<MujocoDriver> mujoco_display_mirror_;

            /// User-added tasks/limits/barriers (RobotSystem owns these)
            std::vector<std::unique_ptr<Task>> user_tasks_;
            std::vector<std::unique_ptr<Limit>> user_limits_;
            std::vector<std::unique_ptr<Barrier>> user_barriers_;

            Eigen::VectorXd home_position_;
            bool home_set_ = false;
            double jog_linear_step_ = 0.001;
            double jog_angular_step_ = 0.01;
            std::string end_effector_frame_;

            Eigen::Matrix4d persistent_target_pose_;
            bool target_initialized_ = false;
            std::string last_jog_frame_;
            double max_tracking_error_ = 0.05;
            bool active_control_ = false;  ///< When false (passive mode), no commands sent to real hardware.
    };
}

#endif // ROBOT_SYSTEM_HPP
