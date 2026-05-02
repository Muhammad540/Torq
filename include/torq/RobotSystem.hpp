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
     * DampingTask, VelocityLimit, ConfigurationLimit). Users do not access the
     * Controller or IK solver directly; all interaction is through RobotSystem.
     *
     * Call `update()` whenever your application needs a new control / physics tick
     * Differential IK integrates with the hardware driver's `getTimestep()` (e.g. MuJoCo
     * `opt.timestep`); that timestep is independent of how often you call `update()`.
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
             * Performs one tick: `hardware_->step()`, then (if active control is on)
             * `Controller::update()`. Task-space IK uses `hardware_->getTimestep()` for
             * integration (MuJoCo model timestep or the real driver's period), not the
             * interval between your calls to `update()`.
             */
            void update();
            
            /**
             * @brief MuJoCo backend that owns `mjModel` / `mjData` for this robot, if any.
             *
             * This is **not** the abstract hardware used for control (that is internal).
             * It is the `MujocoDriver` instance whose scene you can render or introspect:
             *
             * - **Simulation** (`driver_type == "mujoco"`): returns the same driver that
             *   steps physics and receives commands.
             * - **Real hardware** with `scene_path` set: returns a **display-only** driver
             *   loaded from that XML; joint poses are copied from the real robot each
             *   `update()` for visualization. It is never stepped for dynamics.
             * - **Real hardware** without `scene_path`: returns nullptr (no MuJoCo scene).
             *
             * Typical uses: attach `Gui` (viewport), or read MuJoCo-only metadata such as
             * actuator `ctrlrange` during initialize(). Do not assume this pointer is the
             * control plant when using serial servos.
             *
             * @return nullptr if no MuJoCo scene is loaded.
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
             * @brief Load collision geometry for self-collision avoidance.
             *
             * Forwards to KinematicsEngine::loadCollisionModel(). Call after
             * initialize(). Required before adding a SelfCollisionBarrier.
             * Accepts both URDF and MJCF files (auto-detected by extension).
             *
             * @param model_path Path to the URDF or MJCF file with collision geometry.
             * @param srdf_path  Optional SRDF for filtering collision pairs.
             * @return True on success.
             */
            bool loadCollisionModel(const std::string& model_path,
                                    const std::string& srdf_path = "");

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
             * @param limit  Limit to add (e.g. a custom Limit subclass).
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
