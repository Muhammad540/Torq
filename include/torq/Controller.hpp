#ifndef TORQ_CONTROLLER_HPP
#define TORQ_CONTROLLER_HPP

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>

#include "torq/logger.hpp"

namespace torq{

  class HardwareInterface;
  class KinematicsEngine;
  class Task;
  class Limit;
  class Barrier;
  class FrameTask;
  class PostureTask;
  class DampingTask;
  class VelocityLimit;
  class ConfigurationLimit;
  class InverseKinematics;

  /**
   * @brief Control mode for the Controller.
   */
  enum class ControlMode{
    JOINT_SPACE, ///< Direct joint-angle commands via the hardware interface.
    TASK_SPACE   ///< Cartesian differential IK (built-in tasks + QP).
  };

  /**
   * @brief Centralised store for all tunable IK parameters
   *
   * @see @ref tuning_guide for detailed descriptions and tuning recipes.
   */
  struct IKConfig {
    double frame_position_cost   = 1.0;   ///< FrameTask translational weight [cost/m].
    double frame_orientation_cost = 1.0;  ///< FrameTask rotational weight [cost/rad].
    double frame_gain            = 1.0;   ///< FrameTask gain \f$\alpha \in (0,1]\f$.
    double frame_lm_damping      = 0.5;   ///< FrameTask Levenberg–Marquardt damping \f$\mu\f$.
    double posture_cost          = 1e-3;  ///< PostureTask weight [cost/rad].
    double posture_gain          = 1.0;   ///< PostureTask gain \f$\alpha\f$.
    double posture_lm_damping    = 0.5;   ///< PostureTask LM damping.
    double damping_cost          = 1e-2;  ///< DampingTask weight [cost·s/rad].
    double solver_damping        = 1e-12; ///< Tikhonov damping \f$\lambda\f$ on the QP Hessian.
    double config_limit_gain     = 0.5;   ///< ConfigurationLimit gain \f$\gamma \in (0,1]\f$.
  };

  /**
   * @brief Internal: control modes, built-in IK tasks/limits, OSQP solver.
   *
   * Owns the InverseKinematics solver and the built-in FrameTask,
   * PostureTask, DampingTask, VelocityLimit, ConfigurationLimit. User-added
   * tasks/limits/barriers are owned by RobotSystem and passed in by raw
   * pointer each tick.
   *
   * Not part of the public API — library users interact only with
   * RobotSystem, which forwards the relevant methods.
   *
   * @see RobotSystem, InverseKinematics, Task, Limit, Barrier
   */
  class Controller {

  public:
    /**
     * @brief Construct a Controller.
     * @param kinematics  Non-owning pointer to the KinematicsEngine (must outlive the Controller).
     * @param hardware    Non-owning pointer to the HardwareInterface (nullptr until initialize).
     */
    Controller(KinematicsEngine* kinematics, HardwareInterface* hardware);
    ~Controller();

    Controller(const Controller&)            = delete;
    Controller& operator=(const Controller&) = delete;
    Controller(Controller&&)                 = delete;
    Controller& operator=(Controller&&)      = delete;

    /** @brief Set the hardware interface after construction. Must outlive the Controller. */
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

    /**
     * @brief After leaving task-space IK, clear the cached internal configuration
     *        so the next TASK_SPACE tick seeds \f$q\f$ from the hardware state.
     *
     * Call when switching to JOINT_SPACE (or any mode that moves the plant without
     * updating the IK integrator) so task-space IK does not continue from a stale \f$q\f$.
     */
    void invalidateTaskSpaceIkState();

    /**
     * @brief Configure the gripper actuator (command-space index for toggleGripper).
     * @param actuator_idx  Index into the reduced command / joint vector used by setJointPositions.
     * @param open_val      Actuator value for the open position.
     * @param close_val     Actuator value for the closed position.
     * @param current_val   Current actuator value (to detect initial state).
     */
    void setGripperConfig(int actuator_idx, double open_val, double close_val, double current_val);

    /** @brief True if the gripper is currently open. */
    bool isGripperOpen() const { return gripper_open_; }

    /** @brief Toggle gripper between open and closed */
    void toggleGripper();

    /**
     * @brief Execute one control tick.
     *
     * Built-in tasks/limits are always used in TASK_SPACE; user_tasks, user_limits, and
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

    void sendHardwareCommand(const Eigen::Ref<const Eigen::VectorXd>& q_cmd);

    KinematicsEngine* kinematics_;
    HardwareInterface* hardware_;

    std::unique_ptr<InverseKinematics> ik_solver_;
    IKConfig ik_config_;

    // Built-in tasks
    std::unique_ptr<FrameTask> frame_task_;
    std::unique_ptr<PostureTask> posture_task_;
    std::unique_ptr<DampingTask> damping_task_;

    // Built-in limits
    std::unique_ptr<VelocityLimit> vel_limit_;
    std::unique_ptr<ConfigurationLimit> cfg_limit_;

    ControlMode mode_ = ControlMode::JOINT_SPACE;
    Eigen::VectorXd target_joints_;
    bool ik_ready_ = false;

    Eigen::VectorXd reduced_configuration_;
    bool ik_config_initialized_ = false;

    int gripper_actuator_idx_ = -1;
    double gripper_open_val_ = 0.0;
    double gripper_close_val_ = 0.0;
    bool gripper_open_ = true;

    std::vector<Task*>  active_tasks_;
    std::vector<Limit*> active_limits_;
    Eigen::VectorXd     q_cmd_buffer_;

    Logger log_;
  };
} // namespace torq
#endif // TORQ_CONTROLLER_HPP
