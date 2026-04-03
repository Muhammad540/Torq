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
   * A barrier defines a function \f$h(q) \ge 0\f$ over the safe region of
   * configuration space.  The CBF condition:
   * \f[
   *   \frac{\partial h}{\partial q}\,\dot{q} + \alpha\,h(q) \ge 0
   * \f]
   * is enforced as a QP inequality, and an optional safe-displacement cost
   * regularises the objective toward a known-safe velocity.
   *
   * @see @ref barriers_page for mathematical details.
   */
  class Barrier {
    public:
      /**
       * @brief Construct a barrier.
       * @param dim                    Dimension of the barrier function output.
       * @param gain                   Scalar gain \f$\alpha\f$ (broadcast to all dimensions).
       * @param safe_displacement_gain Gain for the safe-displacement regularisation cost. 0 = disabled.
       */
      Barrier(int dim, double gain = 1.0, double safe_displacement_gain = 0.0);

      /**
       * @brief Construct a barrier with per-dimension gain.
       * @param dim                    Dimension of the barrier function output.
       * @param gain                   Per-dimension gain vector \f$\alpha \in \mathbb{R}^{\text{dim}}\f$.
       * @param safe_displacement_gain Gain for the safe-displacement regularisation cost. 0 = disabled.
       */
      Barrier(int dim, const Eigen::VectorXd& gain, double safe_displacement_gain = 0.0);

      virtual ~Barrier() = default;

      /** @brief Evaluate the barrier function \f$h(q) \in \mathbb{R}^{\text{dim}}\f$. */
      virtual Eigen::VectorXd computeBarrier(const Configuration& config) const = 0;

      /** @brief Barrier Jacobian \f$\partial h / \partial q \in \mathbb{R}^{\text{dim} \times n_v}\f$. */
      virtual Eigen::MatrixXd computeJacobian(const Configuration& config) const = 0;

      /**
       * @brief QP inequality: \f$-J_h / dt\;\Delta q \le \alpha\,h(q)\f$.
       * @return Pair (G, h) where \f$G\,\Delta q \le h\f$.
       */
      std::pair<Eigen::MatrixXd, Eigen::VectorXd> computeQPInequalities(
          const Configuration& config, double dt) const;

      /**
       * @brief QP objective contribution from safe-displacement regularisation.
       * @return Pair (H, c) added to the global QP objective.
       */
      std::pair<Eigen::MatrixXd, Eigen::VectorXd> computeQPObjective(
          const Configuration& config) const;

      /** @brief Set a custom gain function \f$\alpha(h)\f$ replacing the linear gain. */
      void setGainFunction(std::function<double(double)> fn) { gain_function_ = std::move(fn); }

      int dim() const { return dim_; }
      const Eigen::VectorXd& gain() const { return gain_; }
      double safeDisplacementGain() const { return safe_displacement_gain_; }

    protected:
      /** @brief Override to provide a non-zero safe-displacement backup. Default returns zero. */
      virtual Eigen::VectorXd computeSafeDisplacement(const Configuration& config) const;

      int dim_;
      Eigen::VectorXd gain_;
      std::function<double(double)> gain_function_;
      double safe_displacement_gain_;
  };

  /**
   * @brief Cartesian position barrier on a robot frame.
   *
   * Constrains the world-frame position of a named frame to lie within
   * \f$[p_{\min}, p_{\max}]\f$ along selected axes.  Useful for workspace
   * bounds (e.g. CoM height for squat, end-effector Z for table avoidance).
   *
   * \f[
   *   h(q) = \begin{bmatrix} p - p_{\min} \\ p_{\max} - p \end{bmatrix}
   * \f]
   */
  class PositionBarrier : public Barrier {
    public:
      /**
       * @param frame   Frame name to monitor.
       * @param indices Cartesian axes to constrain (0=x, 1=y, 2=z). Default: all.
       * @param p_min   Minimum position bound (per selected axis). Empty = no lower bound.
       * @param p_max   Maximum position bound (per selected axis). Empty = no upper bound.
       * @param gain    Barrier gain.
       * @param safe_displacement_gain  Safe displacement cost gain.
       */
      PositionBarrier(const std::string& frame,
                      const std::vector<int>& indices = {0, 1, 2},
                      const std::optional<Eigen::VectorXd>& p_min = std::nullopt,
                      const std::optional<Eigen::VectorXd>& p_max = std::nullopt,
                      double gain = 1.0,
                      double safe_displacement_gain = 0.0);

      Eigen::VectorXd computeBarrier(const Configuration& config) const override;
      Eigen::MatrixXd computeJacobian(const Configuration& config) const override;

    private:
      std::string frame_;
      std::vector<int> indices_;
      std::optional<Eigen::VectorXd> p_min_;
      std::optional<Eigen::VectorXd> p_max_;
  };

  /**
   * @brief Minimum-distance barrier between two frames (spherical approximation).
   *
   * \f[
   *   h(q) = \|p_1 - p_2\|^2 - d_{\min}^2
   * \f]
   *
   * The Jacobian uses position Jacobians in the world frame:
   * \f$\partial h / \partial q = 2(p_1 - p_2)^\top (J_1 - J_2)\f$.
   */
  class BodySphericalBarrier : public Barrier {
    public:
      /**
       * @param frame1  First frame name.
       * @param frame2  Second frame name.
       * @param d_min   Minimum distance threshold [m].
       * @param gain    Barrier gain.
       * @param safe_displacement_gain  Safe displacement cost gain.
       */
      BodySphericalBarrier(const std::string& frame1,
                           const std::string& frame2,
                           double d_min,
                           double gain = 1.0,
                           double safe_displacement_gain = 3.0);

      Eigen::VectorXd computeBarrier(const Configuration& config) const override;
      Eigen::MatrixXd computeJacobian(const Configuration& config) const override;

    private:
      std::string frame1_, frame2_;
      double d_min_;
  };

  /**
   * @brief Self-collision avoidance barrier using Pinocchio geometry distances.
   *
   * Monitors the \f$N\f$ closest collision pairs and keeps their distances above
   * \f$d_{\min}\f$.  Requires the Configuration to have a collision model loaded.
   *
   * \f[
   *   h_i(q) = d(p^1_i, p^2_i) - d_{\min}, \quad i = 1 \ldots N
   * \f]
   */
  class SelfCollisionBarrier : public Barrier {
    public:
      /**
       * @param n_collision_pairs Number of closest collision pairs to monitor.
       * @param gain              Barrier gain.
       * @param safe_displacement_gain Safe displacement cost gain.
       * @param d_min             Minimum distance threshold [m].
       */
      SelfCollisionBarrier(int n_collision_pairs,
                           double gain = 1.0,
                           double safe_displacement_gain = 1.0,
                           double d_min = 0.02);

      Eigen::VectorXd computeBarrier(const Configuration& config) const override;
      Eigen::MatrixXd computeJacobian(const Configuration& config) const override;

    private:
      double d_min_;
  };

} // namespace torq

#endif // TORQ_BARRIERS_HPP
