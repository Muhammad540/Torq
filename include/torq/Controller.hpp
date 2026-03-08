#ifndef TORQ_CONTROLLER_HPP
#define TORQ_CONTROLLER_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>

#include "torq/InverseKinematics.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
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
   * Owned by the Controller; read-only access is available through
   * `RobotSystem::ikConfig()`.  Every field has a corresponding runtime setter
   * on both Controller and RobotSystem.
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
  };

  /**
   * @brief Manages control modes, IK tasks/limits, and the QP solver.
   *
   * The Controller owns the InverseKinematics solver and all task/limit instances.
   * It switches between IDLE, JOINT_SPACE, and TASK_SPACE modes.  In TASK_SPACE
   * mode it builds and solves the QP every tick via update().
   *
   * IK tasks and limits are lazily initialised on the first call to
   * setTaskSpaceTarget().  Parameters from IKConfig are applied both at
   * construction time and via the runtime setters (which update the live
   * task/limit objects immediately).
   *
   * @see RobotSystem (user-facing wrapper), InverseKinematics, Task, Limit
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
     * In TASK_SPACE mode: builds a Configuration, solves the QP, integrates
     * the result, and writes the new joint positions to hardware.
     * In JOINT_SPACE mode: writes target_joints_ directly.
     * In IDLE mode: does nothing.
     */
    void update();

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

    /** @brief Direct access to the live FrameTask (nullptr before first IK init). */
    FrameTask* frameTask() { return frame_task_.get(); }
    /** @brief Direct access to the live PostureTask. */
    PostureTask* postureTask() { return posture_task_.get(); }
    /** @brief Direct access to the live DampingTask. */
    DampingTask* dampingTask() { return damping_task_.get(); }
    /** @brief Direct access to the live VelocityLimit. */
    VelocityLimit* velLimit() { return vel_limit_.get(); }
    /** @brief Direct access to the live ConfigurationLimit. */
    ConfigurationLimit* cfgLimit() { return cfg_limit_.get(); }

  private:
    void initIK();

    KinematicsEngine* kinematics_;
    HardwareInterface* hardware_;

    InverseKinematics ik_solver_;
    IKConfig ik_config_;

    std::unique_ptr<FrameTask> frame_task_;
    std::unique_ptr<PostureTask> posture_task_;
    std::unique_ptr<DampingTask> damping_task_;
    std::unique_ptr<VelocityLimit> vel_limit_;
    std::unique_ptr<ConfigurationLimit> cfg_limit_;

    ControlMode mode_ = ControlMode::IDLE;
    Eigen::VectorXd target_joints_;
    bool ik_ready_ = false;
    double gripper_open_val_ = 0.0;
    double gripper_close_val_ = 0.0;
    bool gripper_open_ = true;
    int gripper_actuator_idx_ = 0;

    Logger log_;
  };
} // namespace torq
#endif // TORQ_CONTROLLER_HPP
