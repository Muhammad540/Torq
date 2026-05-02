#ifndef TORQ_LIMITS_HPP
#define TORQ_LIMITS_HPP

#include <iosfwd>
#include <string>
#include <vector>
#include <optional>
#include <utility>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>

namespace torq {
  class Configuration;

  /**
   * @brief Abstract base class for kinematic limits (QP inequality constraints).
   *
   * Each limit produces rows \f$(G, h)\f$ such that:
   * \f[
   *   G\,\Delta q \le h
   * \f]
   * These rows are stacked with those from other limits and passed to the
   * OSQP solver as the constraint matrix.
   *
   * Returns `std::nullopt` if there are no active constraints (e.g. no finite
   * bounds in the model).
   *
   * @see @ref limits_page for per-limit mathematical derivations.
   * @see @ref qp_formulation for how constraints compose in the QP.
   */
  class Limit {
    public:
      virtual ~Limit() = default;

      /**
       * @brief Compute inequality constraint rows \f$(G, h)\f$.
       * @param config  Current robot configuration (FK computed).
       * @param dt      Integration timestep [s].
       * @return Pair (G, h) or `std::nullopt` if no active constraints.
       */
      virtual std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const = 0;
  };

  /**
   * @brief Joint velocity limits.
   *
   * Constrains displacements so that the implied velocity stays within hardware bounds:
   * \f[
   *   -\Delta t\;\dot{q}_{\max} \;\le\; \Delta q \;\le\; \Delta t\;\dot{q}_{\max}
   * \f]
   *
   * Reads `model.velocityLimit` and only activates for joints with finite bounds.
   *
   * @see @ref limits_page
   */
  class VelocityLimit : public Limit {
    public:
      /**
       * @brief Construct from a Pinocchio model.
       * @param model  Model whose `velocityLimit` vector provides the bounds.
       */
      explicit VelocityLimit(const pinocchio::Model& model);

        std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;


    private:
        const pinocchio::Model& model_;
        std::vector<int> indices_;

        // pre-allocated 
        Eigen::MatrixXd G_;          // Constant matrix [P; -P]
        Eigen::VectorXd v_max_;      // Constant velocity limits
        mutable Eigen::VectorXd h_;  // Buffer for dt * v_max
  };

  /**
   * @brief Joint position (configuration) limits.
   *
   * Constrains \f$\Delta q\f$ so that \f$q + \Delta q\f$ stays within
   * \f$[q_{\min}, q_{\max}]\f$, with a proportional gain \f$\gamma\f$ that
   * smoothly decelerates the robot as it approaches a limit:
   * \f[
   *   \Delta q_j \le \gamma\,(q_j^{\max} - q_j), \quad
   *   -\Delta q_j \le \gamma\,(q_j - q_j^{\min})
   * \f]
   *
   * @see @ref limits_page for a detailed discussion of the gain parameter.
   */
  class ConfigurationLimit: public Limit {
    public:
      /**
       * @brief Construct from a Pinocchio model.
       * @param model             Model whose `lowerPositionLimit` / `upperPositionLimit` provide bounds.
       * @param config_limit_gain Gain \f$\gamma \in (0, 1]\f$.  Lower values cause earlier deceleration.
       */
      explicit ConfigurationLimit(const pinocchio::Model& model, double config_limit_gain = 0.5);

      std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;

      /** @brief Set the configuration limit gain \f$\gamma\f$ at runtime. */
      void setConfigLimitGain(double gain) { config_limit_gain_ = gain; }
      /** @brief Current configuration limit gain. */
      double configLimitGain() const { return config_limit_gain_; }

    private:
      const pinocchio::Model& model_;
      double config_limit_gain_;
      std::vector<int> indices_;

      // pre-allocate
      Eigen::MatrixXd G_;                   // Constant matrix [P; -P]
      mutable Eigen::VectorXd delta_q_max_; // Buffer for max difference
      mutable Eigen::VectorXd delta_q_min_; // Buffer for min difference
      mutable Eigen::VectorXd h_;           // Buffer for scaled limits
  };

} // namespace torq

#endif // TORQ_LIMITS_HPP
