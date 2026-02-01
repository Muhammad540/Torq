#ifndef OPENMANIP_CONTROLLER_HPP
#define OPENMANIP_CONTROLLER_HPP

#include <memory>
#include <string>
#include <Eigen/Dense>

#include "openmanip/InverseKinematics.hpp"
#include "openmanip/HardwareInterface.hpp"
#include "openmanip/PinocchioModel.hpp"
#include "openmanip/logger.hpp"

namespace openmanip{
  enum class ControlMode{
    IDLE,
    JOINT_SPACE,
    TASK_SPACE
  };
  
  struct ControllerConfig{
    float dt = 1;
  };
  
  class Controller {
  // TODO: Add collision/safety checking to this class before sending to IK  
  public:
    Controller(KinematicsEngine* kinematics, HardwareInterface* hardware);
    ~Controller();

    void setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, const std::string& frame_name);
    void setJointSpaceTarget(const Eigen::VectorXd& target_joints);
    void update();
    
  private:
    KinematicsEngine* kinematics_;
    HardwareInterface* hardware_;
    // NOTE(Ahmed) This class owns the IK solver
    std::unique_ptr<InverseKinematics> ik_solver_;

    ControlMode mode_ = ControlMode::IDLE;

    Eigen::Matrix4d target_pose_;
    std::string end_effector_frame_;
    Eigen::VectorXd target_joints_;

    Logger logger;  
  };
}
#endif // OPENMANIP_CONTROLLER_HPP
