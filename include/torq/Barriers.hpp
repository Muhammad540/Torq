#ifndef TORQ_BARRIERS_HPP
#define TORQ_BARRIERS_HPP

#include <vector>
#include <string>
#include <functional>
#include <utility>
#include <optional>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace torq {
  class Configuration;

  /**
   * @brief Abstract base class for Control Barrier Functions (CBFs).
   *
   */
  class Barrier {
    public:
      Barrier(int dim, double gain = 1.0, double safe_displacement_gain = 0.0);
      Barrier(int dim, const Eigen::VectorXd& gain, double safe_displacement_gain = 0.0);
      virtual ~Barrier() = default;

      /** 
       * @brief Computes QP inequalities and QP objectives
       */
      void updateQP(const Configuration& config, double dt);

      const Eigen::MatrixXd& G() const { return G_; }
      const Eigen::VectorXd& h() const { return h_qp_; }
      const Eigen::MatrixXd& H() const { return H_; }
      const Eigen::VectorXd& c() const { return c_; }

      void setGainFunction(std::function<double(double)> fn) { gain_function_ = std::move(fn); }
      int dim() const { return dim_; }
      const Eigen::VectorXd& gain() const { return gain_; }
      double safeDisplacementGain() const { return safe_displacement_gain_; }

    protected:
      virtual void computeBarrier(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_h) const = 0;
      virtual void computeJacobian(const Configuration& config, Eigen::Ref<Eigen::MatrixXd> out_J) const = 0;
      virtual void computeSafeDisplacement(const Configuration& config, Eigen::Ref<Eigen::VectorXd> out_dq) const;
      // number of constraints the barriers add to the QP solver
      int dim_;
      // gain defines the aggressiveness of deceleration when approaching a boundary ( higher = stiffer )
      Eigen::VectorXd gain_;
      std::function<double(double)> gain_function_;
      // to pull the robot to 0 velocity when squashed close to a barrier
      double safe_displacement_gain_;

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
   * @brief Cartesian position barrier on a robot frame.
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
   * @brief Minimum-distance barrier between two frames
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
