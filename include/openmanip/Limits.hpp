#ifndef OPENMANIP_LIMITS_HPP
#define OPENMANIP_LIMITS_HPP

#include <vector>
#include <optional>
#include <utility>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>

namespace openmanip {
  class Configuration;

  /*
    Abstract base class for enforcing kinematic limits
    
    Each limit produces inequality constraints G * dq <= h
    for the QP solver. Returns nullopt if there are no active 
    contraints
  */
  class Limit {
    public:
      virtual ~Limit() = default;
      virtual std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const = 0;
  };

  /*
    Joint velocity limits: -dt * v_max <= dq <= dt * v_max
    Reads model.velocityLimit and indetifies which joints have finite velocity bounds.
  */
  class VelocityLimit : public Limit {
    public:
      explicit VelocityLimit(const pinocchio::Model& model);
      std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;
      
    private:
      const pinocchio::Model& model_;
      std::vector<int> indices_;
      Eigen::MatrixXd projection_matrix_;
  };

  /*
    Joint position (configuration) limits
    Constraints the dq so that q + dq stays within [q_min, q_max]
  */
  class ConfigurationLimit: public Limit {
    public:
      explicit ConfigurationLimit(const pinocchio::Model& model, double config_limit_gain = 0.5);
      std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> computeQPInequalities(const Configuration& config, double dt) const override;

      void setConfigLimitGain(double gain) { config_limit_gain_ = gain; }
      double configLimitGain() const { return config_limit_gain_; }
      
    private:
      const pinocchio::Model& model_;
      double config_limit_gain_;
      std::vector<int> indices_;
      Eigen::MatrixXd projection_matrix_;
  };
} // namespace openmanip
 
#endif // OPENMANIP_LIMITS_HPP
