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
  
  class Controller {
  
  public:
    Controller(KinematicsEngine* kinematics, HardwareInterface* hardware);
    ~Controller();
    
    /* Set an SE3 target for the end effector frame. Creates/updates the FrameTask and switches to TASK_SPACE mode. */
    void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose,
                            const std::string& frame_name);

    /* Direct joint space target. Bypasses IK entirely. */
    void setJointSpaceTarget(const Eigen::VectorXd& target_joints);
    
    /* Initialize gripper state*/
    void setGripperConfig(int actuator_idx, double open_val, double close_val);
    bool isGripperOpen() const { return gripper_open_; }
    
    /* Binary gripper control*/
    void toggleGripper();

    /* Called every control tick by RobotSystem. */
    void update();
  private:
    void initIK();

    KinematicsEngine* kinematics_;
    HardwareInterface* hardware_;

    InverseKinematics ik_solver_;

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
