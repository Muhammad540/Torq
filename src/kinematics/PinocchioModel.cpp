#include "torq/PinocchioModel.hpp"
#include "torq/utils.hpp"

#include <algorithm>
#include <set>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/parsers/srdf.hpp>

namespace torq {
    Configuration::Configuration(
        const pinocchio::Model& model,
        const pinocchio::Data& data,
        const Eigen::VectorXd& q,
        bool forward_kinematics,
        std::shared_ptr<pinocchio::GeometryModel> collision_model,
        std::shared_ptr<pinocchio::GeometryData> collision_data)
        : model_(model)
        , data_(data)
        , q_(q)
        , collision_model_(std::move(collision_model))
        , collision_data_(std::move(collision_data)){
        if (collision_model_ && !collision_data_){
	        collision_data_ = std::make_shared<pinocchio::GeometryData>(*collision_model_);
            log_.info() << "[Configuration] Created collision data from collision model";
        }
        if(forward_kinematics){
	        update();
        }      
    }

    void Configuration::update(const std::optional<Eigen::VectorXd>& q_in){
        if (q_in.has_value()){
	        q_ = q_in.value();
        }
        
        if (collision_model_ && collision_data_){
	        pinocchio::computeCollisions(model_, data_, *collision_model_, *collision_data_, q_, false);
	        pinocchio::computeDistances(model_, data_, *collision_model_, *collision_data_, q_);
        }
        pinocchio::computeJointJacobians(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);
    }

    void Configuration::checklimits(double tol, bool safety_break) {
        const auto& q_max = model_.upperPositionLimit;
        const auto& q_min = model_.lowerPositionLimit;
        auto [root_nq, root_nv] = get_root_joint_dim(model_);

        for (int i = root_nq; i < model_.nq; i++){
            if (q_max(i) <= q_min(i) + tol){
                continue;
            }
            if (q_(i) < q_min(i) - tol || q_(i) > q_max(i) + tol){
                if (safety_break){
                    last_error_ = ErrorCode::NotWithinConfigurationLimits;
		    log_.warning() << "[Configuration] Joint " << i
				   << " violates limits: " << q_(i)
				   << " not in [" << q_min(i)
				   << ", " << q_max(i) << "]";
                    return;
                }
		log_.warning() << "[Configuration] Warning: Value " << q_(i)
                                  << " at index " << i
                                  << " is out of limits: [" << q_min(i)
                                  << ", " << q_max(i) << "]";
            }
        }
    }

    Eigen::MatrixXd Configuration::getFrameJacobian(const std::string& frame) const {
        if (!model_.existFrame(frame)){
            last_error_ = ErrorCode::FrameNotFound;
	    log_.warning() << "[Configuration] Frame not found";
	    return Eigen::MatrixXd::Zero(6, model_.nv);
        }
	
        auto frame_id = model_.getFrameId(frame);
	Eigen::MatrixXd J(6, model_.nv);
        J.setZero();
        pinocchio::getFrameJacobian(model_, const_cast<pinocchio::Data&>(data_), frame_id, pinocchio::LOCAL, J);
        return J;
    }

    Eigen::MatrixXd Configuration::getFrameJacobianWorldAligned(const std::string& frame) const {
        if (!model_.existFrame(frame)){
            last_error_ = ErrorCode::FrameNotFound;
            log_.warning() << "[Configuration] Frame not found";
            return Eigen::MatrixXd::Zero(6, model_.nv);
        }

        auto frame_id = model_.getFrameId(frame);
        Eigen::MatrixXd J(6, model_.nv);
        J.setZero();
        pinocchio::getFrameJacobian(model_, const_cast<pinocchio::Data&>(data_), frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

    pinocchio::SE3 Configuration::getTransformFrameToWorld(const std::string& frame) const {
      if (!model_.existFrame(frame)){
        last_error_ = ErrorCode::FrameNotFound;
        log_.warning() << "[Configuration] frame not found: " << frame;
        return pinocchio::SE3::Identity();
      }

      auto frame_id = model_.getFrameId(frame);
      return data_.oMf[frame_id];
    }

    pinocchio::SE3 Configuration::getTransform(const std::string& source, const std::string& dest) const {
        auto transform_source_to_world = getTransformFrameToWorld(source);
        auto transform_dest_to_world = getTransformFrameToWorld(dest);

        if (last_error_ != ErrorCode::None){
            return pinocchio::SE3::Identity();
        }

        return transform_dest_to_world.actInv(transform_source_to_world);    

    }
  
    Eigen::VectorXd Configuration::integrate(const Eigen::VectorXd& velocity, double dt) const {
        return pinocchio::integrate(model_, q_, velocity*dt);
    }

    
    void Configuration::integrateInplace(const Eigen::VectorXd& velocity, double dt){
        Eigen::VectorXd q_new = pinocchio::integrate(model_, q_, velocity*dt);
        this->update(q_new);
    }
    
    bool KinematicsEngine::initialize(const std::string& model_path,
                                      const std::vector<std::string>& locked_joint_names) {
        try {
            log_.info() << "[KinematicsEngine] Loading model: " << model_path;
            full_model_ = std::make_unique<pinocchio::Model>();

            bool is_urdf = (model_path.rfind(".urdf") != std::string::npos);

            if (is_urdf) {
                pinocchio::urdf::buildModel(model_path, *full_model_);
            } else {
                pinocchio::mjcf::buildModel(model_path, *full_model_);
            }

            log_.info() << "[KinematicsEngine] Full model loaded: "
                        << full_model_->nq << " config dims, "
                        << full_model_->nv << " tangent dims";

            locked_joint_ids_.clear();
            for (const auto& name : locked_joint_names) {
                if (full_model_->existJointName(name)) {
                    locked_joint_ids_.push_back(full_model_->getJointId(name));
                    log_.info() << "[KinematicsEngine] Locking joint: " << name;
                } else {
                    log_.warning() << "[KinematicsEngine] Joint to lock not found: " << name;
                }
            }

            if (!locked_joint_ids_.empty()) {
                Eigen::VectorXd q_ref = pinocchio::neutral(*full_model_);
                model_ = std::make_unique<pinocchio::Model>(pinocchio::buildReducedModel(*full_model_, locked_joint_ids_, q_ref));
                buildQMapping();
                log_.info() << "[KinematicsEngine] Reduced model ("
                            << locked_joint_ids_.size() << " joints locked): "
                            << model_->nq << " config dims, "
                            << model_->nv << " tangent dims";
            } else {
                model_ = std::make_unique<pinocchio::Model>(*full_model_);
                log_.info() << "[KinematicsEngine] No joints locked, using full model";
            }

            return true;
        } catch(const std::exception & e) {
            log_.error() << "[KinematicsEngine] Error loading model: " << e.what();
            full_model_.reset();
            model_.reset();
            return false;
        }
    }

    bool KinematicsEngine::loadCollisionModel(const std::string& model_path,
                                               const std::string& srdf_path) {
        if (!model_) {
            log_.error() << "[KinematicsEngine] Model must be loaded before collision model.";
            return false;
        }
        try {
            auto geom_model = std::make_shared<pinocchio::GeometryModel>();

            bool is_urdf = (model_path.rfind(".urdf") != std::string::npos);
            if (is_urdf) {
                pinocchio::urdf::buildGeom(*model_, model_path, pinocchio::COLLISION, *geom_model);
            } else {
                pinocchio::mjcf::buildGeom(*model_, model_path, pinocchio::COLLISION, *geom_model);
            }

            geom_model->addAllCollisionPairs();

            if (!srdf_path.empty()) {
                pinocchio::srdf::removeCollisionPairs(*model_, *geom_model, srdf_path);
                log_.info() << "[KinematicsEngine] Filtered collision pairs from SRDF: " << srdf_path;
            }
            collision_model_ = geom_model;
            collision_data_ = std::make_shared<pinocchio::GeometryData>(*collision_model_);
            log_.info() << "[KinematicsEngine] Collision model loaded with "
                        << collision_model_->collisionPairs.size() << " collision pairs";
            return true;
        } catch (const std::exception& e) {
            log_.error() << "[KinematicsEngine] Error loading collision model: " << e.what();
            collision_model_.reset();
            return false;
        }
    }

    Configuration KinematicsEngine::makeConfiguration(const Eigen::VectorXd& q) const {
        pinocchio::Data data(*model_);
        return Configuration(*model_, data, q, true, collision_model_, collision_data_);
    }

    void KinematicsEngine::buildQMapping() {
        q_idx_map_.clear();
        if (!full_model_ || !model_) return;
        // form a red-black tree (ologn insert/find/erase)
        std::set<pinocchio::JointIndex> locked_set(locked_joint_ids_.begin(), locked_joint_ids_.end());
        
        // njoints -> gives total number of joints    
        for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full_model_->njoints); ++j) {
            if (locked_set.count(j)) continue;
            // idx_q -> gives the starting index in the config vector q of joint j
            int idx_q  = full_model_->joints[j].idx_q();
            // number of dof's in cspace for that joint
            int nq_j   = full_model_->joints[j].nq();
            for (int k = 0; k < nq_j; ++k)
                q_idx_map_.push_back(idx_q + k);
        }
    }

    Eigen::VectorXd KinematicsEngine::fullToReducedQ(const Eigen::VectorXd& q_full) const {
      if (locked_joint_ids_.empty()) return q_full;

      Eigen::VectorXd q_reduced(static_cast<int>(q_idx_map_.size()));
      for (int i = 0; i < static_cast<int>(q_idx_map_.size()); ++i) {
          q_reduced[i] = q_full[q_idx_map_[i]];
      }
      return q_reduced;
    }

    void KinematicsEngine::printFrames() const {
        if (!model_) {
            log_.warning() << "[KinematicsEngine] Model not loaded";
            return;
        }
        log_.info() << "======= ROBOT FRAMES =======";
        for (const auto& frame : model_->frames) {
            log_.info() << "  " << frame.name;
        }
    }
    } // namespace torq
