#ifndef TORQ_BARRIERS_HPP
#define TORQ_BARRIERS_HPP

#include <vector>
#include <string>
#include <utility>
#include <optional>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace torq {
  class Configuration;

  /**
   * @brief Abstract base for Control Barrier Function safety constraints.
   *
   * A subclass defines a barrier value \f$h(q) \ge 0\f$ on the safe set and
   * its Jacobian. The base assembles a QP inequality (CBF condition) and
   * an optional safe-displacement objective.
   *
   * @see @ref barriers_page, @ref qp_formulation
   */
  class Barrier {
    public:
      Barrier(int dim, double gain = 1.0, double safe_displacement_gain = 0.0);
      Barrier(int dim, const Eigen::VectorXd& gain, double safe_displacement_gain = 0.0);
      virtual ~Barrier() = default;

      /** @brief Recompute G, h, H, c from the current configuration. */
      void updateQP(const Configuration& config, double dt);

      /** @brief Inequality matrix \f$G\f$ (size dim × nv). */
      const Eigen::MatrixXd& G() const { return G_; }
      /** @brief Inequality upper bound \f$h\f$ (size dim). */
      const Eigen::VectorXd& h() const { return h_qp_; }
      /** @brief Safe-displacement Hessian (zero unless `safe_displacement_gain` > 0). */
      const Eigen::MatrixXd& H() const { return H_; }
      /** @brief Safe-displacement gradient. */
      const Eigen::VectorXd& c() const { return c_; }

      int dim() const { return dim_; }
      const Eigen::VectorXd& gain() const { return gain_; }
      double safeDisplacementGain() const { return safe_displacement_gain_; }

      /** @brief Set the same CBF gain on every constraint row. */
      void setGain(double gain);
      void setSafeDisplacementGain(double gain);

    protected:
      virtual void computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const = 0;
      virtual void computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const = 0;
      /** @brief Optional: a known-safe displacement to bias the solver toward. Default: zero. */
      virtual void computeSafeDisplacement(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_dq) const;

      int dim_;                                ///< Number of constraint rows.
      Eigen::VectorXd gain_;                   ///< CBF gain \f$\alpha\f$ (per dim).
      double safe_displacement_gain_;          ///< \f$\kappa\f$; 0 disables the objective term.

    private:
      bool is_initialized_ = false;

      Eigen::VectorXd barrier_cache_;
      Eigen::MatrixXd jac_cache_;
      Eigen::VectorXd safe_dq_cache_;

      Eigen::MatrixXd G_;
      Eigen::VectorXd h_qp_;
      Eigen::MatrixXd H_;
      Eigen::VectorXd c_;
  };

  /**
   * @brief Box bounds on a frame's Cartesian position.
   *
   * At least one of `p_min` or `p_max` must be provided. The barrier
   * dimension equals the number of active sides times the number of
   * selected axes (`indices`).
   *
   * @see @ref barriers_page
   */
  class PositionBarrier : public Barrier {
    public:
      PositionBarrier(const std::string& frame,
                      const std::vector<int>& indices = {0, 1, 2},
                      const std::optional<Eigen::VectorXd>& p_min = std::nullopt,
                      const std::optional<Eigen::VectorXd>& p_max = std::nullopt,
                      double gain = 1.0,
                      double safe_displacement_gain = 0.0);

    protected:
      void computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const override;
      void computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const override;

    private:
      std::string frame_;
      std::vector<int> indices_;
      std::optional<Eigen::VectorXd> p_min_;
      std::optional<Eigen::VectorXd> p_max_;

      mutable Eigen::MatrixXd J_world_;
  };

  /**
   * @brief Minimum distance between two frames using a point to point constraint.
   *
   * Keeps two frames at least `d_min` apart (squared distance barrier).
   *
   * @see @ref barriers_page
   */
  class BodySphericalBarrier : public Barrier {
    public:
      BodySphericalBarrier(const std::string& frame1,
                           const std::string& frame2,
                           double d_min,
                           double gain = 1.0,
                           double safe_displacement_gain = 3.0);

    protected:
      void computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const override;
      void computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const override;

    private:
      std::string frame1_, frame2_;
      double d_min_;

      mutable Eigen::MatrixXd J1_world_;
      mutable Eigen::MatrixXd J2_world_;
  };

} // namespace torq

#endif // TORQ_BARRIERS_HPP
