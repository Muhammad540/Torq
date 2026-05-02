#include "torq/Controller.hpp"

#include "torq/HardwareInterface.hpp"
#include "torq/PinocchioModel.hpp"
#include "torq/InverseKinematics.hpp"
#include "torq/Tasks.hpp"
#include "torq/Limits.hpp"
#include "torq/Barriers.hpp"

#include <algorithm>
#include <cmath>

namespace torq{

  Controller::Controller(KinematicsEngine* kinematics, HardwareInterface* hardware)
      : kinematics_(kinematics),
        hardware_(hardware),
        ik_solver_(std::make_unique<InverseKinematics>()) {
    active_tasks_.reserve(8);
    active_limits_.reserve(8);
    log_.info() << "[Controller] Initialized";
  }

  Controller::~Controller(){
    log_.info() << "[Controller] cleaned up";
  }

  void Controller::initIK() {
    if (ik_ready_ || !hardware_ || !kinematics_ || !kinematics_->isReady()) return;

    const auto& model = kinematics_->model();

    vel_limit_    = std::make_unique<VelocityLimit>(model);
    cfg_limit_    = std::make_unique<ConfigurationLimit>(model, ik_config_.config_limit_gain);
    posture_task_ = std::make_unique<PostureTask>(ik_config_.posture_cost,
                                                  ik_config_.posture_lm_damping,
                                                  ik_config_.posture_gain);
    damping_task_ = std::make_unique<DampingTask>(ik_config_.damping_cost);

    Eigen::VectorXd q0 = kinematics_->fullToReducedQ(hardware_->getJointPositions());
    auto config = kinematics_->makeConfiguration(q0);
    posture_task_->setTargetFromConfiguration(config);

    ik_ready_ = true;
    log_.info() << "[Controller] IK tasks and limits initialized";
  }

  void Controller::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose,
                                      const std::string& frame_name){
    initIK();

    if (!frame_task_ || frame_task_->frame() != frame_name) {
      frame_task_ = std::make_unique<FrameTask>(
          frame_name,
          ik_config_.frame_position_cost,
          ik_config_.frame_orientation_cost,
          ik_config_.frame_lm_damping,
          ik_config_.frame_gain);
    }

    frame_task_->setTarget(pinocchio::SE3(target_pose));
    mode_ = ControlMode::TASK_SPACE;
  }

  void Controller::setJointSpaceTarget(const Eigen::VectorXd& target_joints) {
    target_joints_ = target_joints;
    mode_ = ControlMode::JOINT_SPACE;
  }

  void Controller::invalidateTaskSpaceIkState() {
    ik_config_initialized_ = false;
  }

  void Controller::setGripperConfig(int actuator_idx, double open_val, double close_val, double current_val){
    gripper_actuator_idx_ = actuator_idx;
    gripper_open_val_     = open_val;
    gripper_close_val_    = close_val;
    const double dist_to_open  = std::abs(current_val - open_val);
    const double dist_to_close = std::abs(current_val - close_val);
    gripper_open_ = (dist_to_open <= dist_to_close);
  }

  void Controller::toggleGripper(){
    if (gripper_actuator_idx_ < 0) {
      log_.warning() << "[Controller] gripper not configured; toggle ignored";
      return;
    }

    gripper_open_ = !gripper_open_;
    const double val = gripper_open_ ? gripper_open_val_ : gripper_close_val_;

    if (static_cast<int>(target_joints_.size()) > gripper_actuator_idx_) {
      target_joints_[gripper_actuator_idx_] = val;
    }
  }

  void Controller::sendHardwareCommand(const Eigen::Ref<const Eigen::VectorXd>& q_cmd) {
    if (gripper_actuator_idx_ < 0) {
      hardware_->setJointPositions(q_cmd);
      return;
    }

    const double desired_gripper_val = gripper_open_ ? gripper_open_val_ : gripper_close_val_;
    const int q_in_size  = static_cast<int>(q_cmd.size());
    const int needed_size = std::max(q_in_size, gripper_actuator_idx_ + 1);

    if (q_cmd_buffer_.size() != needed_size) {
      q_cmd_buffer_.setZero(needed_size);
    }
    q_cmd_buffer_.head(q_in_size) = q_cmd;
    q_cmd_buffer_(gripper_actuator_idx_) = desired_gripper_val;

    hardware_->setJointPositions(q_cmd_buffer_);
  }

  void Controller::update(const std::vector<Task*>& user_tasks,
                          const std::vector<Limit*>& user_limits,
                          const std::vector<Barrier*>& user_barriers) {
    if (!hardware_ || !kinematics_ || !kinematics_->isReady()) {
      log_.error() << "[Controller] Hardware or kinematics not ready";
      return;
    }

    if (mode_ == ControlMode::TASK_SPACE) {
      if (!ik_ready_ || !frame_task_) return;

      const double dt = hardware_->getTimestep();

      if (!ik_config_initialized_) {
        reduced_configuration_ = kinematics_->fullToReducedQ(hardware_->getJointPositions());
        ik_config_initialized_ = true;
      }

      auto config = kinematics_->makeConfiguration(reduced_configuration_);

      active_tasks_.clear();
      active_tasks_.push_back(frame_task_.get());
      active_tasks_.push_back(posture_task_.get());
      active_tasks_.push_back(damping_task_.get());
      active_tasks_.insert(active_tasks_.end(), user_tasks.begin(), user_tasks.end());

      active_limits_.clear();
      active_limits_.push_back(vel_limit_.get());
      active_limits_.push_back(cfg_limit_.get());
      active_limits_.insert(active_limits_.end(), user_limits.begin(), user_limits.end());

      Eigen::VectorXd velocity = ik_solver_->solve(config, active_tasks_, dt, ik_config_.solver_damping,active_limits_, user_barriers);
      config.integrateInplace(velocity, dt);
      reduced_configuration_ = config.q();

      sendHardwareCommand(reduced_configuration_);
    } else if (mode_ == ControlMode::JOINT_SPACE) {
      if (target_joints_.size() == 0) return;
      hardware_->setJointPositions(target_joints_);
    }
  }

  void Controller::setFrameTaskPositionCost(double cost) {
    ik_config_.frame_position_cost = cost;
    if (frame_task_) frame_task_->setPositionCost(cost);
  }

  void Controller::setFrameTaskOrientationCost(double cost) {
    ik_config_.frame_orientation_cost = cost;
    if (frame_task_) frame_task_->setOrientationCost(cost);
  }

  void Controller::setFrameTaskGain(double gain) {
    ik_config_.frame_gain = gain;
    if (frame_task_) frame_task_->setGain(gain);
  }

  void Controller::setFrameTaskLMDamping(double lm_damping) {
    ik_config_.frame_lm_damping = lm_damping;
    if (frame_task_) frame_task_->setLMDamping(lm_damping);
  }

  void Controller::setPostureTaskCost(double cost) {
    ik_config_.posture_cost = cost;
    if (posture_task_) posture_task_->setCost(cost);
  }

  void Controller::setPostureTaskGain(double gain) {
    ik_config_.posture_gain = gain;
    if (posture_task_) posture_task_->setGain(gain);
  }

  void Controller::setPostureTaskLMDamping(double lm_damping) {
    ik_config_.posture_lm_damping = lm_damping;
    if (posture_task_) posture_task_->setLMDamping(lm_damping);
  }

  void Controller::setDampingTaskCost(double cost) {
    ik_config_.damping_cost = cost;
    if (damping_task_) damping_task_->setCost(cost);
  }

  void Controller::setSolverDamping(double damping) {
    ik_config_.solver_damping = damping;
  }

  void Controller::setConfigLimitGain(double gain) {
    ik_config_.config_limit_gain = gain;
    if (cfg_limit_) cfg_limit_->setConfigLimitGain(gain);
  }

}
