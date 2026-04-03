#include "torq/Limits.hpp"
#include "torq/PinocchioModel.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include <optional>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace torq {
    VelocityLimit::VelocityLimit(const pinocchio::Model& model)
        : model_(model)
    {
        const auto& v_limit = model_.velocityLimit;

        std::vector<int> index_list;
        for (size_t j = 1; j < model_.joints.size(); ++j) {
            const auto& joint = model_.joints[j];
            int idx_v = joint.idx_v();
            int nv    = joint.nv();

            bool all_bounded = true;
            for (int k = 0; k < nv; ++k) {
                double vl = v_limit(idx_v + k);
                if (vl >= 1e20 || vl <= 1e-10) {
                    all_bounded = false;
                    break;
                }
            }

            if (all_bounded) {
                for (int k = 0; k < nv; ++k) {
                    index_list.push_back(idx_v + k);
                }
            }
        }

        indices_ = index_list;

        if (!indices_.empty()) {
            int dim = static_cast<int>(indices_.size());
            projection_matrix_.resize(dim, model_.nv);
            projection_matrix_.setZero();
            for (int i = 0; i < dim; ++i) {
                projection_matrix_(i, indices_[i]) = 1.0;
            }
        }
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>VelocityLimit::computeQPInequalities(const Configuration& /*config*/, double dt) const
    {
        if (indices_.empty()) {
            return std::nullopt;
        }

        int dim = static_cast<int>(indices_.size());

        Eigen::VectorXd v_max(dim);
        for (int i = 0; i < dim; ++i) {
            v_max(i) = model_.velocityLimit(indices_[i]);
        }

        Eigen::MatrixXd G(2 * dim, model_.nv);
        G.topRows(dim) = projection_matrix_;
        G.bottomRows(dim) = -projection_matrix_;

        Eigen::VectorXd h(2 * dim);
        h.head(dim) = dt * v_max;
        h.tail(dim) = dt * v_max;

        return std::make_pair(G, h);
    }

    ConfigurationLimit::ConfigurationLimit(const pinocchio::Model& model,double config_limit_gain)
        : model_(model), config_limit_gain_(config_limit_gain){
        const auto& q_upper = model_.upperPositionLimit;
        const auto& q_lower = model_.lowerPositionLimit;

        std::vector<int> index_list;
        for (size_t j = 1; j < model_.joints.size(); ++j) {
            const auto& joint = model_.joints[j];
            int idx_q = joint.idx_q();
            int idx_v = joint.idx_v();
            int nq    = joint.nq();
            int nv    = joint.nv();

            bool all_bounded = true;
            for (int k = 0; k < nq; ++k) {
                double upper = q_upper(idx_q + k);
                double lower = q_lower(idx_q + k);
                if (upper >= 1e20 || upper <= lower + 1e-10) {
                    all_bounded = false;
                    break;
                }
            }

            if (all_bounded) {
                for (int k = 0; k < nv; ++k) {
                    index_list.push_back(idx_v + k);
                }
            }
        }

        indices_ = index_list;

        if (!indices_.empty()) {
            int dim = static_cast<int>(indices_.size());
            projection_matrix_.resize(dim, model_.nv);
            projection_matrix_.setZero();
            for (int i = 0; i < dim; ++i) {
                projection_matrix_(i, indices_[i]) = 1.0;
            }
        }
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> ConfigurationLimit::computeQPInequalities(const Configuration &config, double dt) const {
        if (indices_.empty()){
            return std::nullopt;
        }

        int dim = static_cast<int>(indices_.size());
        Eigen::VectorXd delta_q_max = pinocchio::difference(model_, config.q(), model_.upperPositionLimit);
        Eigen::VectorXd delta_q_min = pinocchio::difference(model_, config.q(), model_.lowerPositionLimit);
        
        Eigen::VectorXd p_max(dim), p_min(dim);
        for (int i=0; i<dim; ++i){
            p_max(i) = config_limit_gain_ * delta_q_max(indices_[i]);
            p_min(i) = config_limit_gain_ * delta_q_min(indices_[i]);
        }

        Eigen::MatrixXd G(2 * dim, model_.nv);
        G.topRows(dim) = projection_matrix_;
        G.bottomRows(dim) = -projection_matrix_;

        Eigen::VectorXd h(2 * dim);
        h.head(dim) = p_max;
        h.tail(dim) = -p_min;

        return std::make_pair(G, h);
    }

    // ── FloatingBaseVelocityLimit ──

    static pinocchio::FrameIndex findBaseFrame(const pinocchio::Model& model,
                                               const std::string& base_frame,
                                               pinocchio::JointIndex root_joint_id) {
        if (!base_frame.empty()) {
            if (!model.existFrame(base_frame))
                throw std::invalid_argument("Frame '" + base_frame + "' does not exist in the model.");
            return model.getFrameId(base_frame);
        }
        for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(model.frames.size()); ++i) {
            if (model.frames[i].parentJoint == root_joint_id)
                return i;
        }
        throw std::invalid_argument("Model does not expose a frame attached to 'root_joint'.");
    }

    FloatingBaseVelocityLimit::FloatingBaseVelocityLimit(
        const pinocchio::Model& model,
        const std::string& base_frame,
        const Eigen::Vector3d& max_linear_velocity,
        const Eigen::Vector3d& max_angular_velocity)
        : model_(model)
    {
        if (!model_.existJointName("root_joint"))
            throw std::invalid_argument("FloatingBaseVelocityLimit requires a floating-base root joint.");

        root_joint_id_ = model_.getJointId("root_joint");
        const auto& root_joint = model_.joints[root_joint_id_];
        root_idx_v_ = root_joint.idx_v();
        root_nv_ = root_joint.nv();

        frame_id_ = findBaseFrame(model_, base_frame, root_joint_id_);
        base_frame_ = model_.frames[frame_id_].name;

        twist_max_.head<3>() = max_linear_velocity;
        twist_max_.tail<3>() = max_angular_velocity;
    }

    FloatingBaseVelocityLimit::FloatingBaseVelocityLimit(
        const pinocchio::Model& model,
        const std::string& base_frame,
        double max_linear_velocity,
        double max_angular_velocity)
        : FloatingBaseVelocityLimit(model, base_frame,
              Eigen::Vector3d::Constant(max_linear_velocity),
              Eigen::Vector3d::Constant(max_angular_velocity))
    {}

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    FloatingBaseVelocityLimit::computeQPInequalities(const Configuration& config, double dt) const {
        std::vector<int> finite_indices;
        for (int i = 0; i < 6; ++i) {
            if (std::isfinite(twist_max_(i)))
                finite_indices.push_back(i);
        }
        if (finite_indices.empty())
            return std::nullopt;

        Eigen::MatrixXd J_full = config.getFrameJacobian(base_frame_);

        // Zero out columns that don't belong to the root joint
        J_full.leftCols(root_idx_v_).setZero();
        int after = root_idx_v_ + root_nv_;
        if (after < model_.nv)
            J_full.rightCols(model_.nv - after).setZero();

        int dim = static_cast<int>(finite_indices.size());
        Eigen::MatrixXd active_rows(dim, model_.nv);
        Eigen::VectorXd bounds(dim);
        for (int i = 0; i < dim; ++i) {
            active_rows.row(i) = J_full.row(finite_indices[i]);
            bounds(i) = dt * twist_max_(finite_indices[i]);
        }

        Eigen::MatrixXd G(2 * dim, model_.nv);
        G.topRows(dim) = active_rows;
        G.bottomRows(dim) = -active_rows;

        Eigen::VectorXd h(2 * dim);
        h.head(dim) = bounds;
        h.tail(dim) = bounds;

        return std::make_pair(G, h);
    }

    // ── AccelerationLimit ──

    AccelerationLimit::AccelerationLimit(const pinocchio::Model& model,
                                         const Eigen::VectorXd& acceleration_limit)
        : model_(model)
    {
        Eigen::VectorXd a_lim = acceleration_limit;
        if (model_.nv > 0 && a_lim.size() != model_.nv)
            throw std::invalid_argument("acceleration_limit size must equal model.nv");

        std::vector<int> index_list;
        for (size_t j = 0; j < model_.joints.size(); ++j) {
            const auto& joint = model_.joints[j];
            int idx_v = joint.idx_v();
            int nv = joint.nv();
            if (idx_v < 0) continue;

            bool all_bounded = true;
            for (int k = 0; k < nv; ++k) {
                double val = a_lim(idx_v + k);
                if (val >= 1e20 || val <= 1e-10) {
                    all_bounded = false;
                    break;
                }
            }
            if (all_bounded) {
                for (int k = 0; k < nv; ++k)
                    index_list.push_back(idx_v + k);
            }
        }

        indices_ = index_list;
        int dim = static_cast<int>(indices_.size());

        if (dim > 0) {
            projection_matrix_.resize(dim, model_.nv);
            projection_matrix_.setZero();
            for (int i = 0; i < dim; ++i)
                projection_matrix_(i, indices_[i]) = 1.0;

            a_max_.resize(dim);
            for (int i = 0; i < dim; ++i)
                a_max_(i) = a_lim(indices_[i]);
        }

        delta_q_prev_ = Eigen::VectorXd::Zero(dim);
    }

    void AccelerationLimit::setLastIntegration(const Eigen::VectorXd& v_prev, double dt) {
        Eigen::VectorXd dq_full = v_prev * dt;
        int dim = static_cast<int>(indices_.size());
        for (int i = 0; i < dim; ++i)
            delta_q_prev_(i) = dq_full(indices_[i]);
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    AccelerationLimit::computeQPInequalities(const Configuration& config, double dt) const {
        if (indices_.empty())
            return std::nullopt;

        int dim = static_cast<int>(indices_.size());
        double dt_sq = dt * dt;

        Eigen::VectorXd delta_q_max_full = pinocchio::difference(model_, config.q(), model_.upperPositionLimit);
        Eigen::VectorXd delta_q_min_full = pinocchio::difference(model_, model_.lowerPositionLimit, config.q());

        Eigen::VectorXd delta_q_max(dim), delta_q_min(dim);
        for (int i = 0; i < dim; ++i) {
            delta_q_max(i) = delta_q_max_full(indices_[i]);
            delta_q_min(i) = delta_q_min_full(indices_[i]);
        }

        Eigen::MatrixXd G(2 * dim, model_.nv);
        G.topRows(dim) = projection_matrix_;
        G.bottomRows(dim) = -projection_matrix_;

        // Upper bound: min(accel_bound + prev_dq, braking_distance)
        Eigen::VectorXd h_upper(dim), h_lower(dim);
        for (int i = 0; i < dim; ++i) {
            double accel_bound = a_max_(i) * dt_sq + delta_q_prev_(i);
            double brake_bound = dt * std::sqrt(2.0 * a_max_(i) * std::max(delta_q_max(i), 0.0));
            h_upper(i) = std::min(accel_bound, brake_bound);
        }
        for (int i = 0; i < dim; ++i) {
            double accel_bound = a_max_(i) * dt_sq - delta_q_prev_(i);
            double brake_bound = dt * std::sqrt(2.0 * a_max_(i) * std::max(delta_q_min(i), 0.0));
            h_lower(i) = std::min(accel_bound, brake_bound);
        }

        Eigen::VectorXd h(2 * dim);
        h.head(dim) = h_upper;
        h.tail(dim) = h_lower;

        return std::make_pair(G, h);
    }
}
