#ifndef OPENMANIP_PINOCCHIOMODEL_HPP
#define OPENMANIP_PINOCCHIOMODEL_HPP

#include "Eigen/src/Core/Matrix.h"
#include "openmanip/logger.hpp"
#include "openmanip/utils.hpp"
#include "pinocchio/multibody/fwd.hpp"

#include <optional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace openmanip {
    /*
        Configuration of a robot model.
    
        Holds a Pinocchio model and data where forward kinematics have been
        computed. Frame transforms and Jacobians are available after construction.
    */
    class Configuration {
        public:
           Configuration(const pinocchio::Model& model,
                         const pinocchio::Data& data,   
                         const Eigen::VectorXd& q,
                         bool forward_kinematics = true,
                         std::shared_ptr<pinocchio::GeometryModel> collision_model = nullptr,
                         std::shared_ptr<pinocchio::GeometryData> collision_data = nullptr
           );
           ~Configuration() = default;
           Configuration(const Configuration&) = delete;
           Configuration& operator=(const Configuration&) = delete;
           Configuration(Configuration&&) = default;
           Configuration& operator=(Configuration&&) = default;

           /*   Updates the configuration vector and recompute the kinematics 
                if q_in is nullopt, it recomputes with the current q_
           */
           void update(const std::optional<Eigen::VectorXd>& q_in = std::nullopt);
        
           /* Validates that the current q is within the joint limits */
           void checklimits(double tol, bool safety_break);

           /* Frame Jacobian in LOCAL frame (6 x nv)*/
           Eigen::MatrixXd getFrameJacobian(const std::string& frame) const;

           /* SE3 transform of a frame in world coordinates */
           pinocchio::SE3 getTransformFrameToWorld(const std::string& frame) const;

           /* SE3 transform of source frame expressed in dest frame */
           pinocchio::SE3 getTransform(const std::string& source, const std::string& dest) const;
           
           /* Integrate velocity for dt, return new q (non-mutating) */
           Eigen::VectorXd integrate(const Eigen::VectorXd& velocity, double dt) const;

           /* Integrate velocity for dt, update internal state */
           void integrate_inplace(const Eigen::VectorXd& velocity, double dt);
           
           /* tangent space dim */
           int nv() const { return model_.nv; }

           /* configuration space dim */
           int nq() const { return model_.nq; }

           /* Read only access to the current configuration vector */
           const Eigen::VectorXd& q() const { return q_; }

           /* Read only access to the model */
           const pinocchio::Model& model() const { return model_; }

           /* last error code, for non-exception error reporting*/
           ErrorCode geterror() const { return last_error_; }

           /* Clear error code back to None*/
           void clearError() { last_error_ = ErrorCode::None; }
           
        private:
            mutable Logger log;
            const pinocchio::Model& model_;
            pinocchio::Data data_;
            Eigen::VectorXd q_;

            std::shared_ptr<pinocchio::GeometryModel> collision_model_;
            std::shared_ptr<pinocchio::GeometryData> collision_data_;

            mutable ErrorCode last_error_ = ErrorCode::None;
	};
  
    /*
        KinematicsEngine loads and owns the pinocchio model.
        Use makeConfiguration() to create Configuration objects from it.
    */
    class KinematicsEngine {
        public:
            KinematicsEngine() = default;
            ~KinematicsEngine() = default;

            /* Load the urdf and build the pinocchio model */
            bool initialize(const std::string& urdf_path);

            /* True if the model has been loaded successfully */
            bool isReady() const { return model_ != nullptr; }

            /* Create a configuration from this engine's model at joint config q */
            Configuration makeConfiguration(const Eigen::VectorXd& q) const;

            /* Read only access to the underlying pinocchio model. */
            const pinocchio::Model& model() const { return *model_; }

            /* Print all the frame names */
            void printFrames() const;
        private:
            mutable Logger log;
            std::unique_ptr<pinocchio::Model> model_ = nullptr;
    };
}

#endif // OPENMANIP_PINOCCHIOMODEL_HPP
