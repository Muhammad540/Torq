#include "torq/Limits.hpp"
#include "torq/PinocchioModel.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <optional>
#include <ostream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <cmath>

namespace torq {
    VelocityLimit::VelocityLimit(const pinocchio::Model& model)
        : model_(model)
    {
        const auto& v_limit = model_.velocityLimit;

        std::vector<int> index_list;
        for (size_t j = 1; j < model_.joints.size(); ++j) {
            const auto& joint = model_.joints[j];
            int idx_v = joint.idx_v();
            // each joint has its dof
            int nv    = joint.nv();

            bool all_bounded = true;
            for (int k = 0; k < nv; ++k) {
                double vl = v_limit(idx_v + k);
                // checking for each joint's dof if it is between the sane limits
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
            if (dim > 0){
                v_max_.resize(dim);
                G_.resize(2 * dim, model_.nv);
                G_.setZero();
                h_.resize(2 * dim);
            }
            for (int i = 0; i < dim; ++i) {
                int tangent_idx = indices_[i];
                v_max_(i) = model_.velocityLimit(tangent_idx);
                // Projection matrix helps extract only the joints where limits are actually defined
                // P*Deltaq  <= h
                G_(i, tangent_idx) = 1.0;            // top half (P)
                G_(dim + i, tangent_idx) = -1.0;     // Bottom half (-P)
            }
        }
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    VelocityLimit::computeQPInequalities(const Configuration& /*config*/, double dt) const
    {
        if (indices_.empty()) {
            return std::nullopt;
        }

        int dim = static_cast<int>(indices_.size());

        h_.head(dim) = dt * v_max_;
        h_.tail(dim) = dt * v_max_;

        return std::make_pair(G_, h_);
    }

    ConfigurationLimit::ConfigurationLimit(const pinocchio::Model& model, double config_limit_gain)
        : model_(model), config_limit_gain_(config_limit_gain)
    {
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

        int dim = static_cast<int>(indices_.size());
        if (dim > 0) {
            G_.resize(2 * dim, model_.nv);
            G_.setZero();
            h_.resize(2 * dim);
            
            delta_q_max_.resize(model_.nv);
            delta_q_min_.resize(model_.nv);

            for (int i = 0; i < dim; ++i) {
                int tangent_idx = indices_[i];
                G_(i, tangent_idx) = 1.0;            // Top half (P)
                G_(dim + i, tangent_idx) = -1.0;     // Bottom half (-P)
            }
        }
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    ConfigurationLimit::computeQPInequalities(const Configuration& config, double dt) const
    {
        if (indices_.empty()) {
            return std::nullopt;
        }
    
        // 1. Zero allocation manifold subtraction
        pinocchio::difference(model_, config.q(), model_.upperPositionLimit, delta_q_max_);
        pinocchio::difference(model_, config.q(), model_.lowerPositionLimit, delta_q_min_);
    
        int dim = static_cast<int>(indices_.size());
        
        for (int i = 0; i < dim; ++i) {
            int idx = indices_[i];
            h_(i) = config_limit_gain_ * delta_q_max_(idx);
            h_(dim + i) = -config_limit_gain_ * delta_q_min_(idx);
        }
    
        return std::make_pair(G_, h_);
    }
}
