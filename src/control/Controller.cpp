#include "openmanip/Controller.hpp"
#include <cmath>

namespace openmanip{
  Controller::Controller(KinematicsEngine* kinematics, HardwareInterface* hardware): kinematics_(kinematics), hardware_(hardware){
    log_.info() << "[Controller] Initialized";
  }

  Controller::~Controller(){
    log_.info() << "[Controller] cleaned up";  
  }

  void Controller::initIK() {
      if (ik_ready_ || !kinematics_ || !kinematics_->isReady()) return;

      const auto& model = kinematics_->model();

      // LIMITS FOR QP OBJECTIVE 
      vel_limit_ = std::make_unique<VelocityLimit>(model);
      cfg_limit_ = std::make_unique<ConfigurationLimit>(model);
      // TASKS FOR QP OBJECTIVE
      posture_task_ = std::make_unique<PostureTask>(1e-3);
      damping_task_ = std::make_unique<DampingTask>(1e-4);

      // TODO(Ahmed) this should be set by the user (neutral posture as the regularization target)
      Eigen::VectorXd q0_full = hardware_->getJointPositions();
      Eigen::VectorXd q0 = kinematics_->fullToReducedQ(q0_full);
      // NOTE: we make config from the reduced model
      auto config = kinematics_->makeConfiguration(q0);
      posture_task_->setTargetFromConfiguration(config);

      ik_ready_ = true;
      log_.info() << "[Controller] IK tasks and limits initialized";
  }
  
  void Controller::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, const std::string& frame_name){
    initIK();

    if (!frame_task_ || frame_task_->frame() != frame_name) {
        frame_task_ = std::make_unique<FrameTask>(frame_name, 1.0, 1.0);
    }

    pinocchio::SE3 se3_target(
        target_pose.block<3,3>(0,0),
        target_pose.block<3,1>(0,3));
    frame_task_->setTarget(se3_target);

    mode_ = ControlMode::TASK_SPACE;
  }

  void Controller::setJointSpaceTarget(const Eigen::VectorXd& target_joints){
    target_joints_ = target_joints;
    mode_ = ControlMode::JOINT_SPACE;
  }

  void Controller::setGripperConfig(int actuator_idx, double open_val, double close_val, double current_val){
    gripper_actuator_idx_ = actuator_idx;
    gripper_open_val_ = open_val;
    gripper_close_val_ = close_val;
    double dist_to_open  = std::abs(current_val - open_val);
    double dist_to_close = std::abs(current_val - close_val);
    gripper_open_ = (dist_to_open <= dist_to_close);
  }

  void Controller::toggleGripper(){
      if (gripper_actuator_idx_ < 0) {
        log_.warning() << "[Controller] gripper index is not correctly setup";
      }
      
      gripper_open_ = !gripper_open_;
      double val = gripper_open_ ? gripper_open_val_ : gripper_close_val_;

      if (static_cast<int>(target_joints_.size()) > gripper_actuator_idx_){
        target_joints_[gripper_actuator_idx_] = val;
      }
  }

  void Controller::update(){
    if (!hardware_ || !kinematics_ || !kinematics_->isReady()) {
        log_.error() << "[Controller] Hardware or kinematics not ready";
        return;
    }

    if (mode_ == ControlMode::TASK_SPACE) {
        if (!ik_ready_ || !frame_task_) return;

        double dt = hardware_->getTimestep();
        Eigen::VectorXd q_full = hardware_->getJointPositions();
        Eigen::VectorXd q = kinematics_->fullToReducedQ(q_full);
        auto config = kinematics_->makeConfiguration(q);

        std::vector<Task*> tasks = {
            frame_task_.get(),
            posture_task_.get(),
            damping_task_.get()
        };
        std::vector<Limit*> limits = {
            vel_limit_.get(),
            cfg_limit_.get()
        };
	
        //TODO (damping should be configurable)
        Eigen::VectorXd velocity = ik_solver_.solve(config, tasks, dt, /*damping=*/1e-12, limits);

        config.integrateInplace(velocity, dt);
        Eigen::VectorXd q_cmd = config.q();
        if (gripper_actuator_idx_ >= static_cast<int>(q_cmd.size())) {
          Eigen::VectorXd q_full(gripper_actuator_idx_ + 1);
          q_full.head(q_cmd.size()) = q_cmd;
          q_full[gripper_actuator_idx_] = gripper_open_ ? gripper_open_val_ : gripper_close_val_;
          hardware_->setJointPositions(q_full);
        } else {
          hardware_->setJointPositions(q_cmd);
        }
    } else if (mode_ == ControlMode::JOINT_SPACE) {
        hardware_->setJointPositions(target_joints_);
    }
  }
}
