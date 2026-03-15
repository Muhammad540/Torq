#include "torq/Barriers.hpp"
#include "torq/PinocchioModel.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/spatial/skew.hpp>

namespace torq {

    // ── Barrier base class ──

    Barrier::Barrier(int dim, double gain, double safe_displacement_gain)
        : dim_(dim)
        , gain_(Eigen::VectorXd::Constant(dim, gain))
        , gain_function_([](double x) { return x; })
        , safe_displacement_gain_(safe_displacement_gain)
    {}

    Barrier::Barrier(int dim, const Eigen::VectorXd& gain, double safe_displacement_gain)
        : dim_(dim)
        , gain_(gain)
        , gain_function_([](double x) { return x; })
        , safe_displacement_gain_(safe_displacement_gain)
    {}

    Eigen::VectorXd Barrier::computeSafeDisplacement(const Configuration& config) const {
        return Eigen::VectorXd::Zero(config.nv());
    }

    std::pair<Eigen::MatrixXd, Eigen::VectorXd>
    Barrier::computeQPInequalities(const Configuration& config, double dt) const {
        Eigen::MatrixXd jac = computeJacobian(config);
        Eigen::VectorXd barrier = computeBarrier(config);

        Eigen::MatrixXd G = -jac / dt;

        Eigen::VectorXd h(dim_);
        for (int i = 0; i < dim_; ++i)
            h(i) = gain_(i) * gain_function_(barrier(i));

        return {G, h};
    }

    std::pair<Eigen::MatrixXd, Eigen::VectorXd>
    Barrier::computeQPObjective(const Configuration& config) const {
        int nv = config.nv();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nv, nv);
        Eigen::VectorXd c = Eigen::VectorXd::Zero(nv);

        if (safe_displacement_gain_ > 1e-6) {
            Eigen::MatrixXd jac = computeJacobian(config);
            Eigen::VectorXd safe_dq = computeSafeDisplacement(config);

            double jac_sq_norm = jac.squaredNorm();
            if (jac_sq_norm < 1e-12)
                return {H, c};

            double weight = safe_displacement_gain_ / jac_sq_norm;
            H.diagonal().array() += weight;
            c -= weight * safe_dq;
        }

        return {H, c};
    }

    // ── PositionBarrier ──

    PositionBarrier::PositionBarrier(const std::string& frame,
                                     const std::vector<int>& indices,
                                     const std::optional<Eigen::VectorXd>& p_min,
                                     const std::optional<Eigen::VectorXd>& p_max,
                                     double gain,
                                     double safe_displacement_gain)
        : Barrier(0, gain, safe_displacement_gain)
        , frame_(frame)
        , indices_(indices)
        , p_min_(p_min)
        , p_max_(p_max)
    {
        if (!p_min_.has_value() && !p_max_.has_value())
            throw std::invalid_argument("PositionBarrier requires at least one of p_min or p_max.");

        int d = 0;
        if (p_min_.has_value()) d += static_cast<int>(indices_.size());
        if (p_max_.has_value()) d += static_cast<int>(indices_.size());
        dim_ = d;
        gain_ = Eigen::VectorXd::Constant(dim_, gain);
    }

    Eigen::VectorXd PositionBarrier::computeBarrier(const Configuration& config) const {
        Eigen::Vector3d pos = config.getTransformFrameToWorld(frame_).translation();

        std::vector<double> vals;
        vals.reserve(dim_);
        if (p_min_.has_value()) {
            for (size_t i = 0; i < indices_.size(); ++i)
                vals.push_back(pos(indices_[i]) - (*p_min_)(i));
        }
        if (p_max_.has_value()) {
            for (size_t i = 0; i < indices_.size(); ++i)
                vals.push_back((*p_max_)(i) - pos(indices_[i]));
        }
        return Eigen::Map<Eigen::VectorXd>(vals.data(), dim_);
    }

    Eigen::MatrixXd PositionBarrier::computeJacobian(const Configuration& config) const {
        Eigen::MatrixXd J_local = config.getFrameJacobian(frame_).topRows<3>();
        Eigen::Matrix3d R = config.getTransformFrameToWorld(frame_).rotation();
        Eigen::MatrixXd J_world = R * J_local;

        int nv = config.nv();
        int n_idx = static_cast<int>(indices_.size());
        Eigen::MatrixXd J_sel(n_idx, nv);
        for (int i = 0; i < n_idx; ++i)
            J_sel.row(i) = J_world.row(indices_[i]);

        Eigen::MatrixXd J(dim_, nv);
        int row = 0;
        if (p_min_.has_value()) {
            J.middleRows(row, n_idx) = J_sel;
            row += n_idx;
        }
        if (p_max_.has_value()) {
            J.middleRows(row, n_idx) = -J_sel;
        }
        return J;
    }

    // ── BodySphericalBarrier ──

    BodySphericalBarrier::BodySphericalBarrier(const std::string& frame1,
                                               const std::string& frame2,
                                               double d_min,
                                               double gain,
                                               double safe_displacement_gain)
        : Barrier(1, gain, safe_displacement_gain)
        , frame1_(frame1)
        , frame2_(frame2)
        , d_min_(d_min)
    {
        if (d_min < 0.0)
            throw std::invalid_argument("Minimum distance must be non-negative.");
        // Use h / (1 + |h|) gain function for smooth approach behaviour
        gain_function_ = [](double h) { return h / (1.0 + std::abs(h)); };
    }

    Eigen::VectorXd BodySphericalBarrier::computeBarrier(const Configuration& config) const {
        Eigen::Vector3d p1 = config.getTransformFrameToWorld(frame1_).translation();
        Eigen::Vector3d p2 = config.getTransformFrameToWorld(frame2_).translation();
        Eigen::Vector3d diff = p1 - p2;
        Eigen::VectorXd result(1);
        result(0) = diff.squaredNorm() - d_min_ * d_min_;
        return result;
    }

    Eigen::MatrixXd BodySphericalBarrier::computeJacobian(const Configuration& config) const {
        Eigen::Vector3d p1 = config.getTransformFrameToWorld(frame1_).translation();
        Eigen::Vector3d p2 = config.getTransformFrameToWorld(frame2_).translation();

        Eigen::MatrixXd J1_local = config.getFrameJacobian(frame1_).topRows<3>();
        Eigen::Matrix3d R1 = config.getTransformFrameToWorld(frame1_).rotation();
        Eigen::MatrixXd J1_world = R1 * J1_local;

        Eigen::MatrixXd J2_local = config.getFrameJacobian(frame2_).topRows<3>();
        Eigen::Matrix3d R2 = config.getTransformFrameToWorld(frame2_).rotation();
        Eigen::MatrixXd J2_world = R2 * J2_local;

        Eigen::Vector3d dh = 2.0 * (p1 - p2);
        Eigen::MatrixXd J = dh.transpose() * (J1_world - J2_world);
        return J;
    }

    // ── SelfCollisionBarrier ──

    SelfCollisionBarrier::SelfCollisionBarrier(int n_collision_pairs,
                                               double gain,
                                               double safe_displacement_gain,
                                               double d_min)
        : Barrier(n_collision_pairs, gain, safe_displacement_gain)
        , d_min_(d_min)
    {
        if (d_min < 0.0)
            throw std::invalid_argument("Minimum distance must be non-negative.");
        if (n_collision_pairs < 0)
            throw std::invalid_argument("Number of collision pairs must be non-negative.");
    }

    Eigen::VectorXd SelfCollisionBarrier::computeBarrier(const Configuration& config) const {
        if (!config.hasCollisionModel())
            throw std::runtime_error("SelfCollisionBarrier requires a collision model in Configuration.");

        const auto& collision_model = *config.collisionModel();
        const auto& collision_data = *config.collisionData();
        int n_pairs = static_cast<int>(collision_model.collisionPairs.size());

        if (n_pairs < dim_)
            throw std::runtime_error("Collision pairs (" + std::to_string(n_pairs) +
                                     ") less than barrier dimension (" + std::to_string(dim_) + ").");

        Eigen::VectorXd distances(n_pairs);
        for (int k = 0; k < n_pairs; ++k)
            distances(k) = collision_data.distanceResults[k].min_distance - d_min_;

        // Find the dim_ closest (smallest distance) pairs
        std::vector<int> idx(n_pairs);
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + dim_, idx.end(),
            [&](int a, int b) { return distances(a) < distances(b); });

        Eigen::VectorXd result(dim_);
        for (int i = 0; i < dim_; ++i)
            result(i) = distances(idx[i]);
        return result;
    }

    Eigen::MatrixXd SelfCollisionBarrier::computeJacobian(const Configuration& config) const {
        if (!config.hasCollisionModel())
            throw std::runtime_error("SelfCollisionBarrier requires a collision model in Configuration.");

        const auto& model = config.model();
        const auto& data = config.data();
        const auto& collision_model = *config.collisionModel();
        const auto& collision_data = *config.collisionData();
        int n_pairs = static_cast<int>(collision_model.collisionPairs.size());
        int nv = model.nv;

        Eigen::VectorXd distances(n_pairs);
        for (int k = 0; k < n_pairs; ++k)
            distances(k) = collision_data.distanceResults[k].min_distance;

        std::vector<int> idx(n_pairs);
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + dim_, idx.end(),
            [&](int a, int b) { return distances(a) < distances(b); });

        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(dim_, nv);

        for (int i = 0; i < dim_; ++i) {
            int k = idx[i];
            const auto& cp = collision_model.collisionPairs[k];
            const auto& dr = collision_data.distanceResults[k];

            const auto& go1 = collision_model.geometryObjects[cp.first];
            const auto& go2 = collision_model.geometryObjects[cp.second];

            auto j1_id = go1.parentJoint;
            auto j2_id = go2.parentJoint;

            Eigen::Vector3d w1 = dr.nearest_points[0];
            Eigen::Vector3d w2 = dr.nearest_points[1];

            if ((w1 - w2).squaredNorm() < 1e-20)
                continue;

            Eigen::Vector3d n = (w1 - w2).normalized();
            Eigen::Vector3d r1 = w1 - data.oMi[j1_id].translation();
            Eigen::Vector3d r2 = w2 - data.oMi[j2_id].translation();

            Eigen::MatrixXd J1(6, nv);
            J1.setZero();
            pinocchio::getJointJacobian(model, const_cast<pinocchio::Data&>(data),
                                        j1_id, pinocchio::LOCAL_WORLD_ALIGNED, J1);

            Eigen::RowVectorXd Jrow = n.transpose() * J1.topRows<3>()
                                    + (pinocchio::skew(r1) * n).transpose() * J1.bottomRows<3>();

            Eigen::MatrixXd J2(6, nv);
            J2.setZero();
            pinocchio::getJointJacobian(model, const_cast<pinocchio::Data&>(data),
                                        j2_id, pinocchio::LOCAL_WORLD_ALIGNED, J2);

            Jrow -= n.transpose() * J2.topRows<3>()
                  + (pinocchio::skew(r2) * n).transpose() * J2.bottomRows<3>();

            J.row(i) = Jrow;
        }

        // Replace NaN with zero (can occur at collision)
        J = J.unaryExpr([](double v) { return std::isnan(v) ? 0.0 : v; });
        return J;
    }

} // namespace torq
