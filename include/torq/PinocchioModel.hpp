#ifndef TORQ_PINOCCHIOMODEL_HPP
#define TORQ_PINOCCHIOMODEL_HPP

#include "torq/logger.hpp"
#include "torq/utils.hpp"

#include <optional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
 
namespace torq {

    /**
     * @brief Snapshot of a robot configuration with pre-computed forward kinematics.
     *
     * A Configuration holds a Pinocchio Model reference and a Data instance where
     * FK has been computed for a given \f$q\f$.  It provides frame transforms,
     * Jacobians, and manifold-aware integration.
     *
     * Configurations are created by KinematicsEngine::makeConfiguration().
     *
     * @see KinematicsEngine
     */
    class Configuration {
        public:
           /**
            * @brief Construct a Configuration.
            * @param model              Reference to the (reduced) Pinocchio model.
            * @param data               Pinocchio data (will be copied).
            * @param q                  Configuration vector \f$q \in \mathbb{R}^{n_q}\f$.
            * @param forward_kinematics If true, compute FK immediately.
            * @param collision_model    Optional collision geometry model.
            * @param collision_data     Optional collision geometry data.
            */
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

           /**
            * @brief Update the configuration vector and recompute FK.
            * @param q_in  New configuration vector; if `std::nullopt`, recomputes with the current \f$q\f$.
            */
           void update(const std::optional<Eigen::VectorXd>& q_in = std::nullopt);
        
           /**
            * @brief Validate that the current \f$q\f$ is within joint limits.
            * @param tol          Tolerance for limit checks.
            * @param safety_break If true, set an error code on violation.
            */
           void checklimits(double tol, bool safety_break);

           /**
            * @brief Frame Jacobian in the LOCAL reference frame.
            * @param frame  Frame name in the Pinocchio model.
            * @return Jacobian \f$J \in \mathbb{R}^{6 \times n_v}\f$.
            */
           Eigen::MatrixXd getFrameJacobian(const std::string& frame) const;

           /**
            * @brief Frame Jacobian in the LOCAL_WORLD_ALIGNED reference frame.
            * @param frame  Frame name in the Pinocchio model.
            * @return Jacobian \f$J \in \mathbb{R}^{6 \times n_v}\f$ with columns
            *         expressed at the frame origin but aligned with world axes.
            */
           Eigen::MatrixXd getFrameJacobianWorldAligned(const std::string& frame) const;

           /**
            * @brief SE(3) transform of a frame in world coordinates.
            * @param frame  Frame name.
            * @return \f$T_{\text{frame}}^{\text{world}} \in SE(3)\f$.
            */
           pinocchio::SE3 getTransformFrameToWorld(const std::string& frame) const;

           /**
            * @brief SE(3) transform of source frame expressed in the dest frame.
            * @param source  Source frame name.
            * @param dest    Destination frame name.
            * @return \f$T_{\text{source}}^{\text{dest}}\f$.
            */
           pinocchio::SE3 getTransform(const std::string& source, const std::string& dest) const;
           
           /**
            * @brief Integrate a velocity for \f$\Delta t\f$ and return the new \f$q\f$ (non-mutating).
            *
            * Uses `pinocchio::integrate`: \f$q_{\text{new}} = q \oplus v\,\Delta t\f$.
            *
            * @param velocity  Tangent-space velocity \f$v \in \mathbb{R}^{n_v}\f$.
            * @param dt        Integration timestep [s].
            * @return New configuration vector.
            */
           Eigen::VectorXd integrate(const Eigen::VectorXd& velocity, double dt) const;

           /**
            * @brief Integrate a velocity and update the internal state in place.
            * @param velocity  Tangent-space velocity.
            * @param dt        Integration timestep [s].
            */
           void integrateInplace(const Eigen::VectorXd& velocity, double dt);
           
           /** @brief Tangent-space dimension \f$n_v\f$. */
           int nv() const { return model_.nv; }

           /** @brief Configuration-space dimension \f$n_q\f$. */
           int nq() const { return model_.nq; }

           /** @brief Read-only access to the current configuration vector \f$q\f$. */
           const Eigen::VectorXd& q() const { return q_; }

           /** @brief Read-only access to the underlying Pinocchio model. */
           const pinocchio::Model& model() const { return model_; }

           /** @brief Read-only access to the underlying Pinocchio data. */
           const pinocchio::Data& data() const { return data_; }

           /** @brief Mutable access to the underlying Pinocchio data. */
           pinocchio::Data& data() { return data_; }

           /** @brief Collision geometry model (nullptr if not loaded). */
           const std::shared_ptr<pinocchio::GeometryModel>& collisionModel() const { return collision_model_; }

           /** @brief Collision geometry data (nullptr if not loaded). */
           const std::shared_ptr<pinocchio::GeometryData>& collisionData() const { return collision_data_; }

           /** @brief True if a collision model is loaded and ready. */
           bool hasCollisionModel() const { return collision_model_ != nullptr; }

           /** @brief Last error code (for non-exception error reporting). */
           ErrorCode geterror() const { return last_error_; }

           /** @brief Clear the error code back to None. */
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
  
    /**
     * @brief Loads and owns the Pinocchio model; creates Configuration objects.
     *
     * Supports loading from both URDF (.urdf) and MJCF (.xml) files.
     * Joints listed in `locked_joint_names` are frozen via
     * `pinocchio::buildReducedModel` so that IK only operates on the
     * remaining actuated DOFs.
     *
     * @see Configuration
     */
    class KinematicsEngine {
        public:
            KinematicsEngine() = default;
            ~KinematicsEngine() = default;

            /**
             * @brief Load a model and optionally lock named joints.
             * @param model_path         Path to a URDF or MJCF file.
             * @param locked_joint_names Joint names to freeze in the reduced model.
             * @return True on success.
             */
            bool initialize(const std::string& model_path, const std::vector<std::string>& locked_joint_names = {});

            /**
             * @brief Load collision geometry from a URDF or MJCF file.
             *
             * Call after initialize().  The geometry model is used by
             * SelfCollisionBarrier.  Optionally load an SRDF to filter
             * collision pairs.  File type is auto-detected by extension
             * (.urdf vs .xml).
             *
             * @param model_path Path to the URDF or MJCF file with collision geometry.
             * @param srdf_path  Optional SRDF file for collision pair filtering. Empty = use all pairs.
             * @return True on success.
             */
            bool loadCollisionModel(const std::string& model_path,
                                    const std::string& srdf_path = "");

            /** @brief True if a model has been loaded successfully. */
            bool isReady() const { return model_ != nullptr; }

            /** @brief True if collision geometry is loaded. */
            bool hasCollisionModel() const { return collision_model_ != nullptr; }

            /**
             * @brief Create a Configuration at the given joint angles.
             * @param q  Joint configuration vector for the reduced model.
             * @return A Configuration with FK computed.
             */
            Configuration makeConfiguration(const Eigen::VectorXd& q) const;

            /** @brief Read-only access to the reduced Pinocchio model. */
            const pinocchio::Model& model() const { return *model_; }

            /** @brief Read-only access to the full (unreduced) Pinocchio model. */
            const pinocchio::Model& fullModel() const { return *full_model_; }

            /**
             * @brief Map a full-model \f$q\f$ to the reduced-model \f$q\f$.
             * @param q_full  Configuration vector of the full model.
             * @return Reduced configuration vector (locked joints removed).
             */
            Eigen::VectorXd fullToReducedQ(const Eigen::VectorXd& q_full) const;

            /** @brief Reduced model configuration dimension. */
            int reducedNq() const { return model_ ? model_->nq : 0; }
            /** @brief Full model configuration dimension. */
            int fullNq() const { return full_model_ ? full_model_->nq : 0; }

            /** @brief Print all frame names in the model to stdout. */
            void printFrames() const;
        private:
            void buildQMapping();

            mutable Logger log_;
            std::unique_ptr<pinocchio::Model> model_ = nullptr;
            std::unique_ptr<pinocchio::Model> full_model_ = nullptr;
            std::shared_ptr<pinocchio::GeometryModel> collision_model_ = nullptr;
            std::shared_ptr<pinocchio::GeometryData>  collision_data_  = nullptr;
            std::vector<pinocchio::JointIndex> locked_joint_ids_;
            std::vector<int> q_idx_map_;
    };
}

#endif // TORQ_PINOCCHIOMODEL_HPP
