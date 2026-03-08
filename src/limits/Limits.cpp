#include "torq/Limits.hpp"
#include "torq/PinocchioModel.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include <optional>
#include <vector>

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
}
