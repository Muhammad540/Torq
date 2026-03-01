#ifndef OPENMANIP_PINOCCHIOMODEL_HPP
#define OPENMANIP_PINOCCHIOMODEL_HPP

#include "openmanip/logger.hpp"
#include "openmanip/utils.hpp"

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
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
 
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
           void integrateInplace(const Eigen::VectorXd& velocity, double dt);
           
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
            mutable Logger log_;
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

        Supports loading from both URDF (.urdf) and MJCF (.xml) files.
        Joints listed in locked_joint_names are frozen via
        pinocchio::buildReducedModel so that IK only operates on the remaining DOFs.
    */
    class KinematicsEngine {
        public:
            KinematicsEngine() = default;
            ~KinematicsEngine() = default;

            /* Load model (URDF or MJCF) and optionally lock the named joints */
            bool initialize(const std::string& model_path, const std::vector<std::string>& locked_joint_names = {});

            /* True if the model has been loaded successfully */
            bool isReady() const { return model_ != nullptr; }

            /* Create a configuration from this engine's model at joint config q (reduced model) */
            Configuration makeConfiguration(const Eigen::VectorXd& q) const;

            /* Read only access to the underlying reduced pinocchio model (locked joints frozen) */
            const pinocchio::Model& model() const { return *model_; }

            /* Read only access to the underlying full pinocchio model */
            const pinocchio::Model& fullModel() const { return *full_model_; }

            /* Maps a full model q vector to the reduced model q vector */
            Eigen::VectorXd fullToReducedQ(const Eigen::VectorXd& q_full) const;

            int reducedNq() const { return model_ ? model_->nq : 0; }
            int fullNq() const { return full_model_ ? full_model_->nq : 0; }

            /* Print all the frame names */
            void printFrames() const;
        private:
            void buildQMapping();

            mutable Logger log_;
            std::unique_ptr<pinocchio::Model> model_ = nullptr;
            std::unique_ptr<pinocchio::Model> full_model_ = nullptr;
            std::vector<pinocchio::JointIndex> locked_joint_ids_;
            //NOTE: map to record which indices from the full model survive in the reduced model
            std::vector<int> q_idx_map_;
    };
}

#endif // OPENMANIP_PINOCCHIOMODEL_HPP
