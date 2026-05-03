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
   * @brief Abstract base for QP inequality constraints.
   *
   * Each limit returns rows \f$(G, h)\f$ such that \f$G\,\Delta q \le h\f$,
   * or `std::nullopt` if no constraints are active this tick.
   *
   * @see @ref limits_page, @ref qp_formulation
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
   * @brief Joint velocity bounds.
   *
   * Enforces \f$-\Delta t\,\dot q_\text{max} \le \Delta q \le \Delta t\,\dot q_\text{max}\f$
   * using `model.velocityLimit`. Joints with non-finite bounds are skipped.
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
   * @brief Joint position bounds with smooth deceleration.
   *
   * Keeps \f$q + \Delta q \in [q_\text{min}, q_\text{max}]\f$ using a
   * proportional gain \f$\gamma \in (0, 1]\f$ so the robot decelerates
   * smoothly before reaching a limit.
   *
   * @see @ref limits_page
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
