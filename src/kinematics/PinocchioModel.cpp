#include "openmanip/PinocchioModel.hpp"
#include <exception>
#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "Eigen/src/Core/Matrix.h"
#include "pinocchio/multibody/fwd.hpp"

namespace openmanip {
    KinematicsEngine::KinematicsEngine() {}
    KinematicsEngine::~KinematicsEngine() {}

    bool KinematicsEngine::initialize(const std::string& urdf_path) {
        try {
            log.info() << "[Pinocchio] Initializing model for Pinocchio";
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path, *model_);
            data_  = std::make_unique<pinocchio::Data>(*model_);
            log.info() << urdf_path;
            log.info() << "[Pinocchio] Read " << model_->nq << " Joints.";
            return true;
        } catch(const std::exception & e) {
            log.error() << "[Pinocchio] Error loading URDF: " << e.what();
            return false;
        }
    }

    void KinematicsEngine::update(const Eigen::VectorXd& q) {
        if (!model_ || !data_) return;
        pinocchio::forwardKinematics(*model_, *data_, q);
        // recomputing the frame position after fk 
        pinocchio::updateFramePlacements(*model_, *data_);
        pinocchio::computeJointJacobians(*model_, *data_);
    }

    /*Returns a homogeneous transformation matrix:
        [R R R tx]
        [R R R ty]
        [R R R tz]
        [0 0 0  1]
    */
    Eigen::Matrix4d KinematicsEngine::getFramePose(const std::string& frame_name) const{
        if (!model_ || !data_) return Eigen::Matrix4d::Identity();
        if (model_->existFrame(frame_name)){
            pinocchio::FrameIndex frameId = model_->getFrameId(frame_name);
            const auto& transform = data_->oMf[frameId];
            return transform.toHomogeneousMatrix();
        } else {
            log.error() << "[Pinocchio] Frame not found: " << frame_name;
            return Eigen::Matrix4d::Identity();
        }
    }
    
    // Jcb (6 x nq)
    Eigen::MatrixXd KinematicsEngine::getFrameJacobian(const std::string& frame_name) const{
        if (!model_ || !data_) return Eigen::MatrixXd::Zero(6, model_->nv);
        Eigen::MatrixXd J(6, model_->nv);
        J.setZero();

        if (model_->existFrame(frame_name)){
            pinocchio::FrameIndex frameId = model_->getFrameId(frame_name);
            pinocchio::getFrameJacobian(*model_, *data_, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);
        }
        return J;
    }

    Eigen::VectorXd KinematicsEngine::integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v, double dt){
      if (!model_) {
	    log.error() << "[Pinocchio] Model not setup";
	    return q;
      }
      Eigen::VectorXd q_next(model_->nq);
      pinocchio::integrate(*model_, q, v*dt, q_next);
      return q_next;
    }

    Eigen::VectorXd KinematicsEngine::computeTwistError(const std::string& frame_name, const Eigen::Matrix4d& target_pose){
        if (!model_ || !data_){
            log.error() << "[Pinocchio] Model and Data not setup";
            return Eigen::VectorXd::Zero(6);
        }

        if (model_->existFrame(frame_name)){
	        pinocchio::FrameIndex frameId = model_->getFrameId(frame_name);
	        // oMf -> stores transformations of the 'operational' frames wrt world origin
	        const pinocchio::SE3& T_curr = data_->oMf[frameId];
	        pinocchio::SE3 T_des( target_pose.block<3,3>(0,0), target_pose.block<3,1>(0,3) );
	        // Terr,body = Tcurr^-1 * Tdes
	        pinocchio::SE3 err_body = T_curr.actInv(T_des);
	        // SE(3) -> se(3) body frame 
	        pinocchio::Motion v_err_body = pinocchio::log6(err_body);
	        // lwa: local world alligned
	        pinocchio::Motion v_err_lwa  = T_curr.act(v_err_body);
	        return v_err_lwa.toVector();	
        }
        return Eigen::VectorXd::Zero(6);
    }
     
    void KinematicsEngine::printFrames(){
        if (!model_){
            log.info() << "[Pinocchio] Model not setup";
        }
        log.info() << "....... ROBOT FRAMES .....";
        for (const auto& frame: model_->frames){
            log.info() << "Frame: " << frame.name;
        }
    }
}