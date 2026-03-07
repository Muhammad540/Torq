#ifndef OPENMANIP_CONTROLLER_HPP
#define OPENMANIP_CONTROLLER_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>

#include "openmanip/InverseKinematics.hpp"
#include "openmanip/Tasks.hpp"
#include "openmanip/Limits.hpp"
#include "openmanip/HardwareInterface.hpp"
#include "openmanip/PinocchioModel.hpp"
#include "openmanip/logger.hpp"

namespace openmanip{
  enum class ControlMode{IDLE, JOINT_SPACE, TASK_SPACE};
  
  struct IKConfig {
    double frame_position_cost = 1.0;
    double frame_orientation_cost = 1.0;
    double frame_gain = 1.0;
    double frame_lm_damping = 0.0;
    double posture_cost = 1e-3;
    double posture_gain = 1.0;
    double posture_lm_damping = 0.0;
    double damping_cost = 1e-4;
    double solver_damping = 1e-12;
    double config_limit_gain = 0.5;
  };

  class Controller {
  
  public:
    Controller(KinematicsEngine* kinematics, HardwareInterface* hardware);
    ~Controller();
    
    void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose,
                            const std::string& frame_name);
    void setJointSpaceTarget(const Eigen::VectorXd& target_joints);
    
    void setGripperConfig(int actuator_idx, double open_val, double close_val, double current_val);
    bool isGripperOpen() const { return gripper_open_; }
    void toggleGripper();

    void update();

    /* Runtime IK parameter tuning - applies immediately to live tasks/limits */
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

    const IKConfig& ikConfig() const { return ik_config_; }
    bool ikReady() const { return ik_ready_; }

    FrameTask* frameTask() { return frame_task_.get(); }
    PostureTask* postureTask() { return posture_task_.get(); }
    DampingTask* dampingTask() { return damping_task_.get(); }
    VelocityLimit* velLimit() { return vel_limit_.get(); }
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
} // namespace openmanip
#endif // OPENMANIP_CONTROLLER_HPP
