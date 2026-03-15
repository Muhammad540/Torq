#ifndef TORQ_LIMITS_HPP
#define TORQ_LIMITS_HPP

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
      Eigen::MatrixXd projection_matrix_;
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
      Eigen::MatrixXd projection_matrix_;
  };

  /**
   * @brief Floating-base twist limits for mobile/legged robots.
   *
   * Constrains the base frame's linear and angular velocity so that
   * the floating-base joint stays within user-specified bounds:
   * \f[
   *   -\Delta t\,v_{\max} \;\le\; J_{\text{root}}\,\Delta q \;\le\; \Delta t\,v_{\max}
   * \f]
   *
   * Only the Jacobian columns belonging to the root joint are retained;
   * all other columns are zeroed so that only the floating-base DOFs
   * are constrained.
   *
   * Requires a model with a `root_joint` (floating base).
   *
   * @see @ref limits_page
   */
  class FloatingBaseVelocityLimit : public Limit {
    public:
      /**
       * @brief Construct floating-base velocity limits.
       * @param model               Model with a floating-base root joint.
       * @param base_frame          Name of the base frame (empty = auto-detect first frame on root_joint).
       * @param max_linear_velocity  Linear velocity bound [m/s]. Scalar broadcasts to all 3 axes.
       * @param max_angular_velocity Angular velocity bound [rad/s]. Scalar broadcasts to all 3 axes.
       */
      FloatingBaseVelocityLimit(const pinocchio::Model& model,
                                const std::string& base_frame,
                                const Eigen::Vector3d& max_linear_velocity,
                                const Eigen::Vector3d& max_angular_velocity);

      FloatingBaseVelocityLimit(const pinocchio::Model& model,
                                const std::string& base_frame,
                                double max_linear_velocity,
                                double max_angular_velocity);

      std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;

    private:
      const pinocchio::Model& model_;
      std::string base_frame_;
      pinocchio::FrameIndex frame_id_;
      pinocchio::JointIndex root_joint_id_;
      int root_idx_v_;
      int root_nv_;
      Eigen::Matrix<double, 6, 1> twist_max_;
  };

  /**
   * @brief Joint acceleration limits with braking-distance bounds.
   *
   * Two constraint types per joint:
   * 1. Finite-difference acceleration bound:
   *    \f$ -a_{\max}\,\Delta t^2 + \Delta q_{\text{prev}} \;\le\; \Delta q \;\le\; a_{\max}\,\Delta t^2 + \Delta q_{\text{prev}} \f$
   *
   * 2. Braking-distance bound (Flacco2015, DelPrete2018):
   *    \f$ |\dot{q}| \le \sqrt{2\,a_{\max}\,\text{dist\_to\_limit}} \f$
   *
   * The tighter of both constraints is enforced per joint.
   *
   * @see @ref limits_page
   */
  class AccelerationLimit : public Limit {
    public:
      /**
       * @brief Construct acceleration limits.
       * @param model              Pinocchio model.
       * @param acceleration_limit Vector of per-joint acceleration limits (dimension nv).
       */
      AccelerationLimit(const pinocchio::Model& model,
                        const Eigen::VectorXd& acceleration_limit);

      /**
       * @brief Update with the previous velocity for finite-difference acceleration.
       *
       * Must be called each tick before solve() with the velocity from the previous IK solution.
       *
       * @param v_prev Previous velocity vector.
       * @param dt     Integration timestep [s].
       */
      void setLastIntegration(const Eigen::VectorXd& v_prev, double dt);

      std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;

    private:
      const pinocchio::Model& model_;
      std::vector<int> indices_;
      Eigen::MatrixXd projection_matrix_;
      Eigen::VectorXd a_max_;
      Eigen::VectorXd delta_q_prev_;
  };

} // namespace torq
 
#endif // TORQ_LIMITS_HPP
