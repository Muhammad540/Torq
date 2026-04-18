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
        setTarget(config.getTransformFrameToWorld(frame_));
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

        pinocchio::SE3 T_frame = config.getTransformFrameToWorld(frame_);
        pinocchio::SE3 T_target_to_frame = T_frame.actInv(target_.value());
        return pinocchio::log6(T_target_to_frame).toVector();
    }
    
    Eigen::MatrixXd FrameTask::computeJacobian(const Configuration &config) const {
        if (!target_.has_value()) {
            return Eigen::MatrixXd::Zero(6, config.nv());
        }

        pinocchio::SE3 T_frame = config.getTransformFrameToWorld(frame_);
        pinocchio::SE3 T_frame_to_target = target_.value().actInv(T_frame);

        Eigen::Matrix<double, 6, 6> Jlog;
        pinocchio::Jlog6(T_frame_to_target, Jlog);

        Eigen::MatrixXd J_frame = config.getFrameJacobian(frame_);
        return -Jlog * J_frame;
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
        int task_nv = config.nv() - root_nv;

        if (!target_q_.has_value()){
            return Eigen::VectorXd::Zero(task_nv);
        }

        Eigen::VectorXd diff = pinocchio::difference(
            config.model(), target_q_.value(), config.q()
        );
        return diff.tail(task_nv);
    }

    Eigen::MatrixXd PostureTask::computeJacobian(const Configuration &config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int nv = config.nv();
        return Eigen::MatrixXd::Identity(nv, nv).bottomRows(nv - root_nv);
    }

    DampingTask::DampingTask(double cost): Task(cost, 1.0 /*gain*/, 0.0 /*no LM damping*/) {}
    
    Eigen::VectorXd DampingTask::computeError(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        return Eigen::VectorXd::Zero(config.nv() - root_nv);
    }

    Eigen::MatrixXd DampingTask::computeJacobian(const Configuration &config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int nv = config.nv();
        return Eigen::MatrixXd::Identity(nv, nv).bottomRows(nv - root_nv);
    }  

    // ── ComTask ──

    ComTask::ComTask(double cost, double lm_damping, double gain)
        : Task(Eigen::VectorXd::Constant(3, cost), gain, lm_damping) {}

    ComTask::ComTask(const Eigen::Vector3d& cost, double lm_damping, double gain)
        : Task(cost, gain, lm_damping) {}

    void ComTask::setTarget(const Eigen::Vector3d& target_com) {
        target_com_ = target_com;
    }

    void ComTask::setTargetFromConfiguration(const Configuration& config) {
        Eigen::Vector3d com = pinocchio::centerOfMass(
            config.model(), const_cast<pinocchio::Data&>(config.data()), config.q());
        target_com_ = com;
    }

    Eigen::VectorXd ComTask::computeError(const Configuration& config) const {
        if (!target_com_.has_value())
            return Eigen::VectorXd::Zero(3);

        Eigen::Vector3d actual = pinocchio::centerOfMass(
            config.model(), const_cast<pinocchio::Data&>(config.data()), config.q());
        return actual - target_com_.value();
    }

    Eigen::MatrixXd ComTask::computeJacobian(const Configuration& config) const {
        if (!target_com_.has_value())
            return Eigen::MatrixXd::Zero(3, config.nv());

        return pinocchio::jacobianCenterOfMass(
            config.model(), const_cast<pinocchio::Data&>(config.data()), config.q());
    }

    // ── RelativeFrameTask ──

    RelativeFrameTask::RelativeFrameTask(const std::string& frame,
                                         const std::string& root,
                                         double position_cost,
                                         double orientation_cost,
                                         double lm_damping,
                                         double gain)
        : Task(Eigen::VectorXd::Ones(6), gain, lm_damping)
        , frame_(frame)
        , root_(root)
    {
        setPositionCost(position_cost);
        setOrientationCost(orientation_cost);
    }

    void RelativeFrameTask::setTarget(const pinocchio::SE3& transform_target_to_root) {
        target_ = transform_target_to_root;
    }

    void RelativeFrameTask::setTargetFromConfiguration(const Configuration& config) {
        setTarget(config.getTransform(frame_, root_));
    }

    void RelativeFrameTask::setPositionCost(double cost) {
        cost_.head<3>().setConstant(cost);
    }
    void RelativeFrameTask::setPositionCost(const Eigen::Vector3d& cost) {
        cost_.head<3>() = cost;
    }
    void RelativeFrameTask::setOrientationCost(double cost) {
        cost_.tail<3>().setConstant(cost);
    }
    void RelativeFrameTask::setOrientationCost(const Eigen::Vector3d& cost) {
        cost_.tail<3>() = cost;
    }

    Eigen::VectorXd RelativeFrameTask::computeError(const Configuration& config) const {
        if (!target_.has_value())
            return Eigen::VectorXd::Zero(6);

        pinocchio::SE3 T_frame_to_root = config.getTransform(frame_, root_);
        pinocchio::SE3 T_frame_to_target = target_.value().actInv(T_frame_to_root);
        return pinocchio::log6(T_frame_to_target).toVector();
    }

    Eigen::MatrixXd RelativeFrameTask::computeJacobian(const Configuration& config) const {
        if (!target_.has_value())
            return Eigen::MatrixXd::Zero(6, config.nv());

        pinocchio::SE3 T_frame_to_root = config.getTransform(frame_, root_);
        pinocchio::SE3 T_frame_to_target = target_.value().actInv(T_frame_to_root);

        Eigen::Matrix<double, 6, 6> Jlog;
        pinocchio::Jlog6(T_frame_to_target, Jlog);

        Eigen::MatrixXd J_frame = config.getFrameJacobian(frame_);
        Eigen::MatrixXd J_root = config.getFrameJacobian(root_);
        Eigen::Matrix<double, 6, 6> Ad_fr = T_frame_to_root.toActionMatrixInverse();

        return Jlog * (J_frame - Ad_fr * J_root);
    }

    // ── LinearHolonomicTask ──

    LinearHolonomicTask::LinearHolonomicTask(const Eigen::MatrixXd& A,
                                             const Eigen::VectorXd& b,
                                             const Eigen::VectorXd& q0,
                                             double cost,
                                             double lm_damping,
                                             double gain)
        : Task(cost, gain, lm_damping)
        , A_(A)
        , b_(b)
        , q0_(q0)
    {
        if (b.size() != A.rows())
            throw std::invalid_argument("LinearHolonomicTask: A and b row mismatch.");
    }

    Eigen::VectorXd LinearHolonomicTask::computeError(const Configuration& config) const {
        if (A_.cols() != config.nv())
            throw std::runtime_error("LinearHolonomicTask: A columns != model.nv");

        Eigen::VectorXd diff = pinocchio::difference(config.model(), q0_, config.q());
        return A_ * diff - b_;
    }

    Eigen::MatrixXd LinearHolonomicTask::computeJacobian(const Configuration& config) const {
        if (A_.cols() != config.nv())
            throw std::runtime_error("LinearHolonomicTask: A columns != model.nv");

        int nv = config.nv();
        Eigen::MatrixXd dDiff(nv, nv);
        dDiff.setZero();
        pinocchio::dDifference(config.model(), q0_, config.q(), dDiff, pinocchio::ARG1);
        return A_ * dDiff;
    }

    // ── JointCouplingTask ──

    JointCouplingTask::JointCouplingTask(const std::vector<std::string>& joint_names,
                                         const std::vector<double>& ratios,
                                         double cost,
                                         const Configuration& config,
                                         double lm_damping,
                                         double gain)
        : LinearHolonomicTask(
              Eigen::MatrixXd::Zero(1, config.nv()),
              Eigen::VectorXd::Zero(1),
              pinocchio::neutral(config.model()),
              cost, lm_damping, gain)
        , joint_names_(joint_names)
        , ratios_(ratios)
    {
        if (joint_names.size() != ratios.size())
            throw std::invalid_argument("JointCouplingTask: joint_names and ratios size mismatch.");

        for (size_t i = 0; i < joint_names.size(); ++i) {
            auto jid = config.model().getJointId(joint_names[i]);
            const auto& joint = config.model().joints[jid];
            int start = joint.idx_v();
            int nv = joint.nv();
            for (int k = 0; k < nv; ++k)
                A_(0, start + k) = ratios[i];
        }
    }

    // ── JointVelocityTask ──

    JointVelocityTask::JointVelocityTask(double cost)
        : Task(cost, 1.0, 0.0) {}

    void JointVelocityTask::setTarget(const Eigen::VectorXd& target_v, double dt) {
        target_delta_q_ = target_v * dt;
    }

    Eigen::VectorXd JointVelocityTask::computeError(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int task_nv = config.nv() - root_nv;

        if (!target_delta_q_.has_value())
            return Eigen::VectorXd::Zero(task_nv);

        if (target_delta_q_->size() != task_nv)
            throw std::runtime_error("JointVelocityTask: target size mismatch.");

        return target_delta_q_.value();
    }

    Eigen::MatrixXd JointVelocityTask::computeJacobian(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int nv = config.nv();
        return Eigen::MatrixXd::Identity(nv, nv).bottomRows(nv - root_nv);
    }

    // ── LowAccelerationTask ──

    LowAccelerationTask::LowAccelerationTask(double cost)
        : Task(cost, 1.0, 0.0) {}

    void LowAccelerationTask::setLastIntegration(const Eigen::VectorXd& v_prev, double dt) {
        delta_q_prev_ = v_prev * dt;
    }

    Eigen::VectorXd LowAccelerationTask::computeError(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int task_nv = config.nv() - root_nv;

        if (!delta_q_prev_.has_value())
            return Eigen::VectorXd::Zero(task_nv);

        // Return -Delta_q_prev so that J*dq = -alpha*e => dq = Delta_q_prev => v = v_prev
        return -delta_q_prev_.value();
    }

    Eigen::MatrixXd LowAccelerationTask::computeJacobian(const Configuration& config) const {
        auto [root_nq, root_nv] = get_root_joint_dim(config.model());
        int nv = config.nv();
        return Eigen::MatrixXd::Identity(nv, nv).bottomRows(nv - root_nv);
    }

} // namespace torq
