#include "torq/RobotSystem.hpp"
#include "torq/MujocoDriver.hpp"
#include "torq/PinocchioModel.hpp"
#include <Eigen/Geometry>

#include <iostream>
#include <memory>

// Set to true to print actual end-effector displacement after each physics step.
static constexpr bool IK_ACTUAL_MOTION_DIAGNOSTICS = false;

namespace torq {
    RobotSystem::RobotSystem() {
        hardware_ = std::make_unique<MujocoDriver>();
        kinematics_ = std::make_unique<KinematicsEngine>();
        controller_ = std::make_unique<Controller>(kinematics_.get(), hardware_.get());
    }
    RobotSystem::~RobotSystem() {
        log_.info() << "[RobotSystem] cleaned up";
    }

    bool RobotSystem::initialize(const RobotConfig& config) {
        if (!hardware_) {
            log_.error() << "[RobotSystem] Hardware abstraction not initialized";
            return false;
        }
        if (!hardware_->connect(config.scene_path)) {
            log_.error() << "[RobotSystem] Failed to connect Hardware";
            return false;
        }
        if (!kinematics_->initialize(config.robot_model_path, config.locked_joints)) {
            log_.error() << "[RobotSystem] Failed to initialize the Kinematics Engine";
            return false;
        }

        end_effector_frame_ = config.end_effector_frame;
        max_tracking_error_ = config.max_tracking_error;
        control_frequency_hz_ = config.control_frequency_hz;

        auto* physics = getPhysics();
        mjModel* m = physics->getModel();
        mjData*  d = physics->getData();
        int gripper_idx = config.gripper_actuator_idx;
        if (gripper_idx < 0) gripper_idx = m->nu - 1;
        double low  = m->actuator_ctrlrange[2 * gripper_idx];
        double high = m->actuator_ctrlrange[2 * gripper_idx + 1];
        double current = d->ctrl[gripper_idx];
        controller_->setGripperConfig(gripper_idx, high, low, current);
        return true;
    }

    void RobotSystem::setTaskSpaceTarget(const Eigen::Matrix4d& target_pose, std::string frame_name) {
      if (controller_) {
        persistent_target_pose_ = target_pose;
        target_initialized_ = true;
        controller_->setTaskSpaceTarget(target_pose, frame_name);
      } else {
        log_.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::setJointSpaceTarget(const Eigen::VectorXd& target_pose) {
      if (controller_) {
        controller_->resetIKState();
        target_initialized_ = false;
        controller_->setJointSpaceTarget(target_pose);
      } else {
        log_.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::update() {
       if (hardware_ && kinematics_) {
          Eigen::Vector3d pos_before;
          Eigen::Vector3d pos_after;
          std::string diag_frame;
          bool capture_actual = false;
          if (IK_ACTUAL_MOTION_DIAGNOSTICS && controller_ && controller_->ikReady() && controller_->frameTask()) {
             diag_frame = controller_->frameTask()->frame();
             pos_before = getFramePose(diag_frame).block<3, 1>(0, 3);
             capture_actual = true;
          }

          // Stepping Simulation (applies ctrl set in previous update)
          hardware_->step();

          if (capture_actual) {
             pos_after = getFramePose(diag_frame).block<3, 1>(0, 3);
             Eigen::Vector3d actual_dp = pos_after - pos_before;
             std::cout << "[IK diag] actual world dp (from prev cmd): ["
                       << actual_dp.transpose() << "] dz=" << actual_dp(2) << std::endl;
          }

          if (controller_) {
            // updating physics
            controller_->update();
          }
       }
    }

    MujocoDriver* RobotSystem::getPhysics() {
        return dynamic_cast<MujocoDriver*>(hardware_.get());
    }


    Eigen::Matrix4d RobotSystem::getFramePose(std::string frame_name){
      Eigen::VectorXd q_full = hardware_->getJointPositions();
      Eigen::VectorXd q = kinematics_->fullToReducedQ(q_full);
      auto config = kinematics_->makeConfiguration(q);
      return config.getTransformFrameToWorld(frame_name).toHomogeneousMatrix();
    }

    void RobotSystem::setHomePosition(){
      if (hardware_){
        home_position_ = hardware_->getCtrl();
        home_set_ = true;
        log_.info() << "[RobotSystem] Home position saved (" << home_position_.transpose() << ")";
      }
    }

    void RobotSystem::moveToHome() {
      if (!home_set_) {
        log_.warning() << "[RobotSystem] Home position not set";
        return;
      }
      if (controller_) {
        setJointSpaceTarget(home_position_);
      }
    }

    bool RobotSystem::hasHomePosition() const {
      return home_set_;
    }

    void RobotSystem::setJogStep(double linear_step, double angular_step) {
      jog_linear_step_ = linear_step;
      jog_angular_step_ = angular_step;
    }

    void RobotSystem::setMaxTrackingError(double max_error) {
      max_tracking_error_ = max_error;
    }

    void RobotSystem::setControlFrequencyHz(double hz) {
      if (hz > 0.0) control_frequency_hz_ = hz;
    }

    void RobotSystem::jogCartesian(int axis, double sign, const std::string& frame_name) {
      if (frame_name != last_jog_frame_) {
        target_initialized_ = false;
        last_jog_frame_ = frame_name;
      }
      if (!target_initialized_) {
        persistent_target_pose_ = getFramePose(frame_name);
        target_initialized_ = true;
      }

      Eigen::Matrix4d candidate = persistent_target_pose_;
      if (axis < 3) {
        candidate(axis, 3) += sign * jog_linear_step_;
      } else {
        Eigen::Vector3d rot_axis = Eigen::Vector3d::Zero();
        rot_axis(axis - 3) = 1.0;
        Eigen::AngleAxisd delta(sign * jog_angular_step_, rot_axis);
        candidate.block<3, 3>(0, 0) =
            delta.toRotationMatrix() * candidate.block<3, 3>(0, 0);
      }

      Eigen::Vector3d actual_pos = getFramePose(frame_name).block<3, 1>(0, 3);
      Eigen::Vector3d target_pos = candidate.block<3, 1>(0, 3);
      if ((target_pos - actual_pos).norm() < max_tracking_error_) {
        persistent_target_pose_ = candidate;
      }

      setTaskSpaceTarget(persistent_target_pose_, frame_name);
    }

    void RobotSystem::resetTaskSpaceTargetToCurrentPose(const std::string& frame_name) {
      persistent_target_pose_ = getFramePose(frame_name);
      target_initialized_ = true;
      last_jog_frame_ = frame_name;
      if (controller_) {
        controller_->setTaskSpaceTarget(persistent_target_pose_, frame_name);
      }
    }

    void RobotSystem::toggleGripper(){
      if (controller_) controller_->toggleGripper();
    }

    bool RobotSystem::isGripperOpen() const {
      if (controller_) return controller_->isGripperOpen();
      log_.warning() << "[RobotSystem] controller not set up properly";
      return true;
    }

    void RobotSystem::setFrameTaskPositionCost(double cost) {
      if (controller_) controller_->setFrameTaskPositionCost(cost);
    }

    void RobotSystem::setFrameTaskOrientationCost(double cost) {
      if (controller_) controller_->setFrameTaskOrientationCost(cost);
    }

    void RobotSystem::setFrameTaskGain(double gain) {
      if (controller_) controller_->setFrameTaskGain(gain);
    }

    void RobotSystem::setFrameTaskLMDamping(double lm_damping) {
      if (controller_) controller_->setFrameTaskLMDamping(lm_damping);
    }

    void RobotSystem::setPostureTaskCost(double cost) {
      if (controller_) controller_->setPostureTaskCost(cost);
    }

    void RobotSystem::setPostureTaskGain(double gain) {
      if (controller_) controller_->setPostureTaskGain(gain);
    }

    void RobotSystem::setPostureTaskLMDamping(double lm_damping) {
      if (controller_) controller_->setPostureTaskLMDamping(lm_damping);
    }

    void RobotSystem::setDampingTaskCost(double cost) {
      if (controller_) controller_->setDampingTaskCost(cost);
    }

    void RobotSystem::setSolverDamping(double damping) {
      if (controller_) controller_->setSolverDamping(damping);
    }

    void RobotSystem::setConfigLimitGain(double gain) {
      if (controller_) controller_->setConfigLimitGain(gain);
    }

    const IKConfig& RobotSystem::ikConfig() const {
      return controller_->ikConfig();
    }

    bool RobotSystem::ikReady() const {
      return controller_ && controller_->ikReady();
    }

    Eigen::VectorXd RobotSystem::getJointPositions(){
      return hardware_->getJointPositions();
    }
  
}
