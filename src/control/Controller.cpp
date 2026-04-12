#include "torq/Controller.hpp"
#include "torq/PinocchioModel.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

static constexpr bool IK_DIAGNOSTICS = false;

namespace torq{
  Controller::Controller(KinematicsEngine* kinematics, HardwareInterface* hardware): kinematics_(kinematics), hardware_(hardware){
    log_.info() << "[Controller] Initialized";
  }

  Controller::~Controller(){
    log_.info() << "[Controller] cleaned up";  
  }

  void Controller::initIK() {
      if (ik_ready_ || !hardware_ || !kinematics_ || !kinematics_->isReady()) return;

      const auto& model = kinematics_->model();

      vel_limit_ = std::make_unique<VelocityLimit>(model);
      cfg_limit_ = std::make_unique<ConfigurationLimit>(model, ik_config_.config_limit_gain);
      posture_task_ = std::make_unique<PostureTask>(ik_config_.posture_cost, ik_config_.posture_lm_damping, ik_config_.posture_gain);
      damping_task_ = std::make_unique<DampingTask>(ik_config_.damping_cost);

      // Auto-detect floating base and create limit if configured
      if (ik_config_.has_floating_base && model.existJointName("root_joint")) {
          try {
              floating_base_limit_ = std::make_unique<FloatingBaseVelocityLimit>(
                  model, "",
                  ik_config_.max_base_linear_vel,
                  ik_config_.max_base_angular_vel);
              log_.info() << "[Controller] FloatingBaseVelocityLimit created";
          } catch (const std::exception& e) {
              log_.warning() << "[Controller] Could not create FloatingBaseVelocityLimit: " << e.what();
          }
      }

      Eigen::VectorXd q0_full = hardware_->getJointPositions();
      Eigen::VectorXd q0 = kinematics_->fullToReducedQ(q0_full);
      auto config = kinematics_->makeConfiguration(q0);
      posture_task_->setTargetFromConfiguration(config);

      prev_velocity_ = Eigen::VectorXd::Zero(model.nv);

      ik_ready_ = true;
      log_.info() << "[Controller] IK tasks and limits initialized";
  }
  
  void Controller::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, const std::string& frame_name){
    initIK();

    if (!frame_task_ || frame_task_->frame() != frame_name) {
        frame_task_ = std::make_unique<FrameTask>(
            frame_name,
            ik_config_.frame_position_cost,
            ik_config_.frame_orientation_cost,
            ik_config_.frame_lm_damping,
            ik_config_.frame_gain);
    }

    pinocchio::SE3 se3_target(
        target_pose.block<3,3>(0,0),
        target_pose.block<3,1>(0,3));
    frame_task_->setTarget(se3_target);

    mode_ = ControlMode::TASK_SPACE;
  }

  void Controller::setJointSpaceTarget(const Eigen::VectorXd& target_joints) {
    target_joints_ = target_joints;
    mode_ = ControlMode::JOINT_SPACE;
  }

  void Controller::resetIKState() {
    ik_config_initialized_ = false;
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

  void Controller::update(const std::vector<Task*>& user_tasks,
                          const std::vector<Limit*>& user_limits,
                          const std::vector<Barrier*>& user_barriers) {
    if (!hardware_ || !kinematics_ || !kinematics_->isReady()) {
        log_.error() << "[Controller] Hardware or kinematics not ready";
        return;
    }

    if (mode_ == ControlMode::TASK_SPACE) {
        if (!ik_ready_ || !frame_task_) return;

        double dt = hardware_->getTimestep();

        if (!ik_config_initialized_) {
          Eigen::VectorXd q_full = hardware_->getJointPositions();
          ik_configuration_ = kinematics_->fullToReducedQ(q_full);
          ik_config_initialized_ = true;
        }

        auto config = kinematics_->makeConfiguration(ik_configuration_);

        // Assemble built-in + user tasks
        std::vector<Task*> tasks = {
            frame_task_.get(),
            posture_task_.get(),
            damping_task_.get()
        };
        tasks.insert(tasks.end(), user_tasks.begin(), user_tasks.end());

        // Assemble built-in + user limits
        std::vector<Limit*> limits = {
            vel_limit_.get(),
            cfg_limit_.get()
        };
        if (floating_base_limit_)
            limits.push_back(floating_base_limit_.get());
        if (accel_limit_)
            limits.push_back(accel_limit_.get());
        limits.insert(limits.end(), user_limits.begin(), user_limits.end());

        // Barriers (user-provided only; no built-in barriers)
        const std::vector<Barrier*>& barriers = user_barriers;

        // Feed previous velocity to acceleration limit
        if (accel_limit_)
            accel_limit_->setLastIntegration(prev_velocity_, dt);

        Eigen::VectorXd velocity = ik_solver_.solve(config, tasks, dt, ik_config_.solver_damping, limits, barriers);
        prev_velocity_ = velocity;

        if (IK_DIAGNOSTICS) {
            const std::string& frame_name = frame_task_->frame();
            Eigen::MatrixXd J = config.getFrameJacobian(frame_name);
            Eigen::VectorXd body_twist = J * velocity;
            std::cout << "[IK diag] frame=" << frame_name
                      << " dt=" << dt
                      << " |dq|=" << velocity.norm()
                      << " dq=[" << velocity.transpose() << "]"
                      << " body_vel_xyz=[" << body_twist.head<3>().transpose() << "]"
                      << " body_dz=" << body_twist(2)
                      << std::endl;
        }

        config.integrateInplace(velocity, dt);
        ik_configuration_ = config.q();

        // Optional: slow re-sync to prevent kinematic/physics drift over long runs
        constexpr bool IK_ENABLE_SLOW_RESYNC = false;
        constexpr double IK_RESYNC_ALPHA = 0.01;
        if (IK_ENABLE_SLOW_RESYNC) {
          Eigen::VectorXd q_mujoco =
              kinematics_->fullToReducedQ(hardware_->getJointPositions());
          ik_configuration_ =
              (1.0 - IK_RESYNC_ALPHA) * ik_configuration_ +
              IK_RESYNC_ALPHA * q_mujoco;
        }

        Eigen::VectorXd q_cmd = ik_configuration_;
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
