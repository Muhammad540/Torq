#include "torq/RobotSystem.hpp"
#include "torq/MujocoDriver.hpp"
#include "torq/ServoDriver.hpp"
#include "torq/PinocchioModel.hpp"
#include <Eigen/Geometry>

#include <algorithm>
#include <iostream>
#include <memory>
namespace torq {
    RobotSystem::RobotSystem() {
        kinematics_ = std::make_unique<KinematicsEngine>();
        controller_ = std::make_unique<Controller>(kinematics_.get(), nullptr);
    }
    RobotSystem::~RobotSystem() {
        log_.info() << "[RobotSystem] cleaned up";
    }

    bool RobotSystem::initialize(const RobotConfig& config) {
        const bool use_real_driver = (config.driver_type == "serial_servo");

        if (use_real_driver) {
            if (config.robot_calib_file.empty()) {
                log_.error() << "[RobotSystem] robot_calib_file is required when driver_type is " << config.driver_type;
                return false;
            }
            hardware_.reset();
            hardware_ = std::make_unique<ServoDriver>();
        } else {
            hardware_.reset();
            hardware_ = std::make_unique<MujocoDriver>();
        }
        controller_->setHardwareInterface(hardware_.get());

        if (!hardware_) {
            log_.error() << "[RobotSystem] Hardware abstraction not initialized";
            return false;
        }

        const std::string robot_calib_file = use_real_driver ? config.robot_calib_file : config.scene_path;
        if (!hardware_->connect(robot_calib_file)) {
            log_.error() << "[RobotSystem] Failed to connect Hardware";
            return false;
        }
        if (!kinematics_->initialize(config.robot_model_path, config.locked_joints,
                                    config.joint_velocity_limit_rad_s)) {
            log_.error() << "[RobotSystem] Failed to initialize the Kinematics Engine";
            return false;
        }

        end_effector_frame_ = config.end_effector_frame;
        max_tracking_error_ = config.max_tracking_error;
        active_control_ = use_real_driver ? config.active_control : true;

        // When using real hardware, optionally load MuJoCo scene as a display
        // mirror so the GUI can still render the 3D robot model. The display
        // model is never stepped — only its qpos is overwritten from the real
        // robot each update().
        if (use_real_driver && !config.scene_path.empty()) {
            mujoco_display_mirror_ = std::make_unique<MujocoDriver>();
            if (!mujoco_display_mirror_->connect(config.scene_path)) {
                log_.warning() << "[RobotSystem] Could not load display scene '" << config.scene_path
                               << "'; GUI 3D view will be unavailable.";
                mujoco_display_mirror_.reset();
            } else {
                Eigen::VectorXd q_real = hardware_->getJointPositions();
                int nq_display = mujoco_display_mirror_->getModel()->nq;
                if (q_real.size() == nq_display) {
                    mujoco_display_mirror_->overrideJointPositions(q_real);
                } else if (q_real.size() < nq_display) {
                    Eigen::VectorXd q_padded = Eigen::VectorXd::Zero(nq_display);
                    q_padded.head(q_real.size()) = q_real;
                    mujoco_display_mirror_->overrideJointPositions(q_padded);
                } else {
                    mujoco_display_mirror_->overrideJointPositions(q_real.head(nq_display));
                }
                log_.info() << "[RobotSystem] Display mirror loaded from '" << config.scene_path << "'";
            }
        }

        MujocoDriver* mj_vis = mujocoVisualizationDriver();
        if (mj_vis && mj_vis->getModel() && mj_vis->getData()) {
            mjModel* m = mj_vis->getModel();
            mjData*  d = mj_vis->getData();
            int gripper_idx = config.gripper_actuator_idx;
            if (gripper_idx < 0) gripper_idx = m->nu - 1;
            double low  = m->actuator_ctrlrange[2 * gripper_idx];
            double high = m->actuator_ctrlrange[2 * gripper_idx + 1];
            double current = d->ctrl[gripper_idx];
            controller_->setGripperConfig(gripper_idx, high, low, current);
        }

        controller_->setJointSpaceTarget(
            kinematics_->fullToReducedQ(hardware_->getJointPositions()));

        return true;
    }

    bool RobotSystem::loadCollisionModel(const std::string& model_path,
                                          const std::string& srdf_path) {
        if (!kinematics_) {
            log_.error() << "[RobotSystem] KinematicsEngine not initialized";
            return false;
        }
        return kinematics_->loadCollisionModel(model_path, srdf_path);
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
        controller_->invalidateTaskSpaceIkState();
        target_initialized_ = false;
        controller_->setJointSpaceTarget(target_pose);
      } else {
        log_.error() << "[RobotSystem] Failed to initialize the Controller";
      }
    }

    void RobotSystem::update() {
       if (hardware_ && kinematics_) {
          hardware_->step();

          if (controller_ && active_control_) {
            std::vector<Task*> user_task_ptrs;
            user_task_ptrs.reserve(user_tasks_.size());
            for (const auto& p : user_tasks_) user_task_ptrs.push_back(p.get());
            std::vector<Limit*> user_limit_ptrs;
            user_limit_ptrs.reserve(user_limits_.size());
            for (const auto& p : user_limits_) user_limit_ptrs.push_back(p.get());
            std::vector<Barrier*> user_barrier_ptrs;
            user_barrier_ptrs.reserve(user_barriers_.size());
            for (const auto& p : user_barriers_) user_barrier_ptrs.push_back(p.get());
            controller_->update(user_task_ptrs, user_limit_ptrs, user_barrier_ptrs);
          }

          if (mujoco_display_mirror_) {
             Eigen::VectorXd q_real = hardware_->getJointPositions();
             int nq_display = mujoco_display_mirror_->getModel()->nq;
             if (q_real.size() == nq_display) {
                mujoco_display_mirror_->overrideJointPositions(q_real);
             } else if (q_real.size() < nq_display) {
                Eigen::VectorXd q_padded = Eigen::VectorXd::Zero(nq_display);
                q_padded.head(q_real.size()) = q_real;
                mujoco_display_mirror_->overrideJointPositions(q_padded);
             } else {
                mujoco_display_mirror_->overrideJointPositions(q_real.head(nq_display));
             }
          }
       }
    }

    MujocoDriver* RobotSystem::mujocoVisualizationDriver() {
        if (mujoco_display_mirror_)
            return mujoco_display_mirror_.get();
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

    void RobotSystem::setHomePosition(const Eigen::VectorXd& home_position) {
      home_position_ = home_position;
      home_set_ = true;
      log_.info() << "[RobotSystem] Home position saved (" << home_position_.transpose() << ")";
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

    Eigen::VectorXd RobotSystem::getJointVelocities(){
      return hardware_->getJointVelocities();
    }

    void RobotSystem::addTask(std::unique_ptr<Task> task) {
        if (task)
            user_tasks_.push_back(std::move(task));
    }

    void RobotSystem::removeTask(Task* task) {
        if (!task) return;
        user_tasks_.erase(
            std::remove_if(user_tasks_.begin(), user_tasks_.end(),
                [task](const std::unique_ptr<Task>& p) { return p.get() == task; }),
            user_tasks_.end());
    }

    void RobotSystem::addLimit(std::unique_ptr<Limit> limit) {
        if (limit)
            user_limits_.push_back(std::move(limit));
    }

    void RobotSystem::removeLimit(Limit* limit) {
        if (!limit) return;
        user_limits_.erase(
            std::remove_if(user_limits_.begin(), user_limits_.end(),
                [limit](const std::unique_ptr<Limit>& p) { return p.get() == limit; }),
            user_limits_.end());
    }

    void RobotSystem::addBarrier(std::unique_ptr<Barrier> barrier) {
        if (barrier)
            user_barriers_.push_back(std::move(barrier));
    }

    void RobotSystem::removeBarrier(Barrier* barrier) {
        if (!barrier) return;
        user_barriers_.erase(
            std::remove_if(user_barriers_.begin(), user_barriers_.end(),
                [barrier](const std::unique_ptr<Barrier>& p) { return p.get() == barrier; }),
            user_barriers_.end());
    }
}
