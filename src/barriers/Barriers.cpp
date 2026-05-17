#include "torq/Barriers.hpp"
#include "torq/PinocchioModel.hpp"

#include <cmath>
#include <stdexcept>

namespace torq {
    Barrier::Barrier(int dim, double gain, double safe_displacement_gain)
        : dim_(dim)
        , gain_(Eigen::VectorXd::Constant(dim, gain))
        , safe_displacement_gain_(safe_displacement_gain)
    {}

    Barrier::Barrier(int dim, const Eigen::VectorXd& gain, double safe_displacement_gain)
        : dim_(dim)
        , gain_(gain)
        , safe_displacement_gain_(safe_displacement_gain)
    {}

    void Barrier::computeSafeDisplacement(const Configuration& /*config*/, Eigen::Ref<Eigen::VectorXd> out_dq) const {
        out_dq.setZero();
    }

    void Barrier::setGain(double gain) {
        gain_.setConstant(dim_, gain);
    }

    void Barrier::setSafeDisplacementGain(double gain) {
        safe_displacement_gain_ = gain;
    }

    void Barrier::updateQP(const Configuration& config, double dt) {
        if (dt <= 0.0)
            throw std::invalid_argument("Barrier::updateQP requires dt > 0.");

        int nv = config.nv();
    
        if (!is_initialized_ || H_.rows() != nv) {
            barrier_cache_.resize(dim_);
            jac_cache_.resize(dim_, nv);
            safe_dq_cache_.resize(nv);
            G_.resize(dim_, nv);
            h_qp_.resize(dim_);
            H_.resize(nv, nv);
            c_.resize(nv);
            is_initialized_ = true;
        }
    
        computeBarrier(config, barrier_cache_);
        computeJacobian(config, jac_cache_);
    
        // QP variable is joint displacement dq; discrete CBF: h(q) + J Δq ≥ 0  ⇔  -J Δq ≤ h(q).
        G_.noalias() = -jac_cache_;
        for (int i = 0; i < dim_; ++i)
            h_qp_(i) = gain_(i) * barrier_cache_(i);

        H_.setZero();
        c_.setZero();
        if (safe_displacement_gain_ > 1e-6) {
            computeSafeDisplacement(config, safe_dq_cache_);
            H_.diagonal().setConstant(safe_displacement_gain_);
            c_.noalias() = -safe_displacement_gain_ * safe_dq_cache_;
        }
    }

    namespace {
        int positionBarrierDim(const std::optional<Eigen::VectorXd>& p_min,
                               const std::optional<Eigen::VectorXd>& p_max,
                               std::size_t num_indices) {
            int d = 0;
            if (p_min.has_value()) d += static_cast<int>(num_indices);
            if (p_max.has_value()) d += static_cast<int>(num_indices);
            return d;
        }
    }

    PositionBarrier::PositionBarrier(const std::string& frame,
                                     const std::vector<int>& indices,
                                     const std::optional<Eigen::VectorXd>& p_min,
                                     const std::optional<Eigen::VectorXd>& p_max,
                                     double gain,
                                     double safe_displacement_gain)
        : Barrier(positionBarrierDim(p_min, p_max, indices.size()), gain, safe_displacement_gain)
        , frame_(frame)
        , indices_(indices)
        , p_min_(p_min)
        , p_max_(p_max)
    {
        if (!p_min_.has_value() && !p_max_.has_value())
            throw std::invalid_argument("PositionBarrier requires at least one of p_min or p_max.");
    }

    void PositionBarrier::computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const {
        Eigen::Vector3d pos = config.getTransformFrameToWorld(frame_).translation();

        int row = 0;
        if (p_min_.has_value()) {
            for (size_t i = 0; i < indices_.size(); ++i) {
                out_h(row++) = pos(indices_[i]) - (*p_min_)(i);
            }
        }
        if (p_max_.has_value()) {
            for (size_t i = 0; i < indices_.size(); ++i) {
                out_h(row++) = (*p_max_)(i) - pos(indices_[i]);
            }
        }
    }

    void PositionBarrier::computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const {
        int nv = config.nv();
        
        if (J_world_.cols() != nv) J_world_.resize(3, nv);

        auto J_local = config.getFrameJacobian(frame_).topRows<3>();
        Eigen::Matrix3d R = config.getTransformFrameToWorld(frame_).rotation();
        
        // .noalias() avoids a temporary heap allocation for the matrix multiplication
        J_world_.noalias() = R * J_local;

        int row = 0;
        int n_idx = static_cast<int>(indices_.size());
        
        if (p_min_.has_value()) {
            for (int i = 0; i < n_idx; ++i) {
                out_J.row(row++) = J_world_.row(indices_[i]);
            }
        }
        if (p_max_.has_value()) {
            for (int i = 0; i < n_idx; ++i) {
                out_J.row(row++) = -J_world_.row(indices_[i]);
            }
        }
    }

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
    }

    void BodySphericalBarrier::computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const {
        Eigen::Vector3d p1 = config.getTransformFrameToWorld(frame1_).translation();
        Eigen::Vector3d p2 = config.getTransformFrameToWorld(frame2_).translation();
        
        out_h(0) = (p1 - p2).squaredNorm() - (d_min_ * d_min_);
    }

    void BodySphericalBarrier::computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const {
        int nv = config.nv();

        if (J1_world_.cols() != nv) {
            J1_world_.resize(3, nv);
            J2_world_.resize(3, nv);
        }

        auto J1_local = config.getFrameJacobian(frame1_).topRows<3>();
        Eigen::Matrix3d R1 = config.getTransformFrameToWorld(frame1_).rotation();
        J1_world_.noalias() = R1 * J1_local;

        auto J2_local = config.getFrameJacobian(frame2_).topRows<3>();
        Eigen::Matrix3d R2 = config.getTransformFrameToWorld(frame2_).rotation();
        J2_world_.noalias() = R2 * J2_local;

        Eigen::Vector3d p1 = config.getTransformFrameToWorld(frame1_).translation();
        Eigen::Vector3d p2 = config.getTransformFrameToWorld(frame2_).translation();
        Eigen::Vector3d dh = 2.0 * (p1 - p2);

        out_J.noalias() = dh.transpose() * (J1_world_ - J2_world_);
    }
} // namespace torq
