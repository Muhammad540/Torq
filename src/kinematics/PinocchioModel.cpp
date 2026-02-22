#include "openmanip/PinocchioModel.hpp"
#include "openmanip/utils.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>

namespace openmanip {
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
    
    bool KinematicsEngine::initialize(const std::string& urdf_path) {
        try {
            log_.info() << "[KinematicsEngine] Loading URDF: " << urdf_path;
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path, *model_);
            log_.info() << "[KinematicsEngine] Model loaded: "
                        << model_->nq << " config dims, "
                        << model_->nv << " tangent dims";
            return true;
        } catch(const std::exception & e) {
            log_.error() << "[KinematicsEngine] Error loading URDF: " << e.what();
            model_.reset();
            return false;
        }
    }

    Configuration KinematicsEngine::makeConfiguration(const Eigen::VectorXd& q) const {
        pinocchio::Data data(*model_);
        return Configuration(*model_, data, q);
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
    } // namespace openmanip
