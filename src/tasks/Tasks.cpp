#include "torq/Tasks.hpp"
#include "torq/utils.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include <Eigen/Dense>
#include "torq/PinocchioModel.hpp"
#include <pinocchio/spatial/explog.hpp>
#include <stdexcept>

namespace torq{
    Task::Task(const Eigen::VectorXd& cost, double gain, double lm_damping)
        : cost_(cost)
        , gain_(gain)
        , lm_damping_(lm_damping)
        , scalar_cost_(false){}
    
    Task::Task(double cost, double gain, double lm_damping)
        : cost_(Eigen::VectorXd::Constant(1, cost))
        , gain_(gain)
        , lm_damping_(lm_damping)
        , scalar_cost_(true){}
    
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> Task::computeQPObjective(const Configuration &config) const{
        Eigen::MatrixXd J = computeJacobian(config);      // (k x nv)
        Eigen::VectorXd e = computeError(config);         // (k)
        int nv = config.nv();
        int k  = static_cast<int>(J.rows());

        Eigen::VectorXd w;
        if (scalar_cost_) {
            w = Eigen::VectorXd::Constant(k, cost_(0));
        } else {
            w = cost_;
        }

        // Weighted quantities
        //   WJ = diag(w) * J           (k x nv)
        //   We = diag(w) * (-alpha*e)  (k)
        Eigen::VectorXd minus_alpha_e = -gain_ * e;
        Eigen::MatrixXd WJ = w.asDiagonal() * J;
        Eigen::VectorXd We = w.asDiagonal() * minus_alpha_e;

        // Levenberg-Marquardt damping: mu = lm_damping * ||We||^2
        double mu = lm_damping_ * We.squaredNorm();

        // H = WJ^T * WJ + mu * I
        Eigen::MatrixXd H = WJ.transpose() * WJ;
        if (mu > 0.0) {
            H.diagonal().array() += mu;
        }

        // c = -WJ^T * We
        Eigen::VectorXd c = -(WJ.transpose() * We);

        return {H, c};
    }

    FrameTask::FrameTask(const std::string& frame,
                         double position_cost,
                         double orientation_cost,
                         double lm_damping,
                         double gain)
        : Task(Eigen::VectorXd::Ones(6), gain, lm_damping), frame_(frame){
            setPositionCost(position_cost);
            setOrientationCost(orientation_cost);
        }
    
    void FrameTask::setTarget(const pinocchio::SE3& target) {
        target_ = target;
    }

    void FrameTask::setTargetFromConfiguration(const Configuration &config) {
        setTarget(config.getTransformFrameToWorld(frame_));  // transform_target_to_world
    }

    void FrameTask::setPositionCost(double cost){
        cost_.head<3>().setConstant(cost);
    }
    
    void FrameTask::setPositionCost(const Eigen::Vector3d& cost){
        cost_.head<3>() = cost;
    }

    void FrameTask::setOrientationCost(double cost){
        cost_.tail<3>().setConstant(cost);
    }

    void FrameTask::setOrientationCost(const Eigen::Vector3d& cost){
        cost_.tail<3>() = cost;
    }

    Eigen::VectorXd FrameTask::computeError(const Configuration& config) const {
        if (!target_.has_value()) {
            return Eigen::VectorXd::Zero(6);
        }

        pinocchio::SE3 transform_frame_to_world = config.getTransformFrameToWorld(frame_); // T_current in world frame
        pinocchio::SE3 transform_target_to_frame = transform_frame_to_world.actInv(target_.value());   // T_current,target
        return pinocchio::log6(transform_target_to_frame).toVector();
    }
    
    Eigen::MatrixXd FrameTask::computeJacobian(const Configuration &config) const {
        if (!target_.has_value()) {
            return Eigen::MatrixXd::Zero(6, config.nv());
        }

        pinocchio::SE3 transform_frame_to_world = config.getTransformFrameToWorld(frame_); // T_current in world frame
        pinocchio::SE3 transform_frame_to_target = target_.value().actInv(transform_frame_to_world);   // T_current,target

        Eigen::Matrix<double, 6, 6> Jlog;
        pinocchio::Jlog6(transform_frame_to_target, Jlog);

        Eigen::MatrixXd jacobian_in_frame = config.getFrameJacobian(frame_);
        return -Jlog * jacobian_in_frame;
    }

    PostureTask::PostureTask(double cost, double lm_damping, double gain)
        : Task(cost, gain, lm_damping) {}
    
    void PostureTask::setTarget(const Eigen::VectorXd &target_q) {
        target_q_ = target_q;
    }

    void PostureTask::setTargetFromConfiguration(const Configuration &config) {
        target_q_ = config.q();
    }

    /*
        Error: get the tangent vector from target to 1, the gain then drives q back.
    */
    Eigen::VectorXd PostureTask::computeError(const Configuration &config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int task_nv = config.nv() - root_nv; // remove the tangent dims related to floating base like humanoids or quads

        if (!target_q_.has_value()){
            return Eigen::VectorXd::Zero(task_nv);
        }

        Eigen::VectorXd diff = pinocchio::difference(
            config.model(), target_q_.value(), config.q()
        );
        return diff.tail(task_nv);
    }

    Eigen::MatrixXd PostureTask::computeJacobian(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        const int nv = config.nv();
        const int task_nv = nv - root_nv;
    
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(task_nv, nv);
        J.rightCols(task_nv).setIdentity();
        return J;
    }

    DampingTask::DampingTask(double cost): Task(cost, 1.0 /*gain*/, 0.0 /*no LM damping*/) {}
    
    Eigen::VectorXd DampingTask::computeError(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        return Eigen::VectorXd::Zero(config.nv() - root_nv);
    }

    Eigen::MatrixXd DampingTask::computeJacobian(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        const int nv = config.nv();
        const int task_nv = nv - root_nv;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(task_nv, nv);
        J.rightCols(task_nv).setIdentity();
        return J;
    }

} // namespace torq
