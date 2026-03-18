#ifndef TORQ_CONTROLLER_HPP
#define TORQ_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "torq/InverseKinematics.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
#include "torq/Barriers.hpp"
#include "torq/HardwareInterface.hpp"
#include "torq/PinocchioModel.hpp"
#include "torq/logger.hpp"

namespace torq{

  /**
   * @brief Control mode for the Controller state machine.
   */
  enum class ControlMode{
    IDLE,        ///< No active control — robot holds current position.
    JOINT_SPACE, ///< Direct joint-angle commands.
    TASK_SPACE   ///< Cartesian IK — tasks and limits are active.
  };
  
  /**
   * @brief Centralised store for all tunable IK parameters with sensible defaults.
   *
   * Owned by the Controller. Library users get read-only access via
   * RobotSystem::ikConfig(); runtime setters are called through RobotSystem
   * (e.g. setFrameTaskPositionCost, setConfigLimitGain).
   *
   * @see @ref tuning_guide for detailed descriptions and tuning recipes.
   */
  struct IKConfig {
    double frame_position_cost   = 1.0;   ///< FrameTask translational weight [cost/m].
    double frame_orientation_cost = 1.0;  ///< FrameTask rotational weight [cost/rad].
    double frame_gain            = 1.0;   ///< FrameTask gain \f$\alpha \in (0,1]\f$.
    double frame_lm_damping      = 0.0;   ///< FrameTask Levenberg–Marquardt damping \f$\mu\f$.
    double posture_cost          = 1e-3;  ///< PostureTask weight [cost/rad].
    double posture_gain          = 1.0;   ///< PostureTask gain \f$\alpha\f$.
    double posture_lm_damping    = 0.0;   ///< PostureTask LM damping.
    double damping_cost          = 1e-4;  ///< DampingTask weight [cost·s/rad].
    double solver_damping        = 1e-12; ///< Tikhonov damping \f$\lambda\f$ on the QP Hessian.
    double config_limit_gain     = 0.5;   ///< ConfigurationLimit gain \f$\gamma \in (0,1]\f$.
    bool   has_floating_base     = false; ///< Auto-attach FloatingBaseVelocityLimit when true.
    double max_base_linear_vel   = 1.0;   ///< FloatingBaseVelocityLimit linear bound [m/s].
    double max_base_angular_vel  = 1.0;   ///< FloatingBaseVelocityLimit angular bound [rad/s].
  };

  /**
   * @brief Internal component: control modes, built-in IK tasks/limits, QP solver.
   *
   * **Ownership:** Controller owns the InverseKinematics solver and all
   * *built-in* tasks and limits (FrameTask, PostureTask, DampingTask,
   * VelocityLimit, ConfigurationLimit, optional FloatingBaseVelocityLimit and
   * AccelerationLimit). It does **not** own user-added tasks/limits/barriers;
   * those are owned by RobotSystem and passed into update() each tick.
   *
   * **Separation of concern:** Library users interact only with RobotSystem.
   * Controller is not part of the public API; RobotSystem forwards
   * setTaskSpaceTarget, setJointSpaceTarget, IK tuning, and update().
   *
   * Control loop: update() is invoked by RobotSystem at the control rate
   * (200–1000 Hz). In TASK_SPACE mode it builds the QP from built-in plus
   * user-provided task/limit/barrier lists and solves via OSQP.
   *
   * @see RobotSystem, InverseKinematics, Task, Limit, Barrier
   */
  class Controller {

  public:
    /**
     * @brief Construct a Controller.
     * @param kinematics  Non-owning pointer to the KinematicsEngine (must outlive the Controller).
     * @param hardware    Non-owning pointer to the HardwareInterface (must outlive the Controller).
     */
    Controller(KinematicsEngine* kinematics, HardwareInterface* hardware);
    ~Controller();

    /** @brief Set the hardware interface (e.g. when switching from sim to real driver). Must outlive the Controller. */
    void setHardwareInterface(HardwareInterface* hardware) { hardware_ = hardware; }

    /**
     * @brief Set a Cartesian target and switch to TASK_SPACE mode.
     *
     * Lazily initialises the IK subsystem on the first call.
     *
     * @param target_pose  Desired SE(3) pose as a 4×4 homogeneous matrix.
     * @param frame_name   Name of the frame to control.
     */
    void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose,
                            const std::string& frame_name);

    /**
     * @brief Set a joint-angle target and switch to JOINT_SPACE mode.
     * @param target_joints  Desired joint positions (reduced model nq).
     */
    void setJointSpaceTarget(const Eigen::VectorXd& target_joints);

    /** @brief Reset IK kinematic state (call when leaving TASK_SPACE). */
    void resetIKState();

    /**
     * @brief Configure the gripper actuator.
     * @param actuator_idx  Index into the MuJoCo actuator array.
     * @param open_val      Actuator value for the open position.
     * @param close_val     Actuator value for the closed position.
     * @param current_val   Current actuator value (to detect initial state).
     */
    void setGripperConfig(int actuator_idx, double open_val, double close_val, double current_val);

    /** @brief True if the gripper is currently open. */
    bool isGripperOpen() const { return gripper_open_; }

    /** @brief Toggle gripper between open and closed. */
    void toggleGripper();

    /**
     * @brief Execute one control tick.
     *
     * Built-in tasks/limits are always used; user_tasks, user_limits, and
     * user_barriers are appended for this solve only (owned by RobotSystem).
     *
     * @param user_tasks   User-added tasks (non-owning; may be empty).
     * @param user_limits  User-added limits (non-owning; may be empty).
     * @param user_barriers User-added barriers (non-owning; may be empty).
     */
    void update(const std::vector<Task*>& user_tasks = {},
                const std::vector<Limit*>& user_limits = {},
                const std::vector<Barrier*>& user_barriers = {});

    /** @name Runtime IK parameter tuning
     *  These apply immediately to the live task/limit objects (if initialised)
     *  and update the stored IKConfig.
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
    /** @} */

    /** @brief Read-only access to the current IK parameter set. */
    const IKConfig& ikConfig() const { return ik_config_; }

    /** @brief True if the IK subsystem has been initialised. */
    bool ikReady() const { return ik_ready_; }

  private:
    void initIK();

    KinematicsEngine* kinematics_;
    HardwareInterface* hardware_;

    InverseKinematics ik_solver_;
    IKConfig ik_config_;

    // Built-in tasks
    std::unique_ptr<FrameTask> frame_task_;
    std::unique_ptr<PostureTask> posture_task_;
    std::unique_ptr<DampingTask> damping_task_;

    // Built-in limits
    std::unique_ptr<VelocityLimit> vel_limit_;
    std::unique_ptr<ConfigurationLimit> cfg_limit_;
    std::unique_ptr<FloatingBaseVelocityLimit> floating_base_limit_;
    std::unique_ptr<AccelerationLimit> accel_limit_;

    ControlMode mode_ = ControlMode::IDLE;
    Eigen::VectorXd target_joints_;
    bool ik_ready_ = false;

    Eigen::VectorXd ik_configuration_;
    Eigen::VectorXd prev_velocity_;
    bool ik_config_initialized_ = false;
    double gripper_open_val_ = 0.0;
    double gripper_close_val_ = 0.0;
    bool gripper_open_ = true;
    int gripper_actuator_idx_ = 0;

    Logger log_;
  };
} // namespace torq
#endif // TORQ_CONTROLLER_HPP
