#include "openmanip/PinocchioModel.hpp"
#include <exception>
#include <memory>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "Eigen/src/Core/Matrix.h"
#include "openmanip/logger.hpp"
#include "pinocchio/multibody/fwd.hpp"

namespace openmanip {
    KinematicsEngine::KinematicsEngine();
    KinematicsEngine::~KinematicsEngine() {}

    bool KinematicsEngine::initialize(const std::string& urdf_path) {
        try {
            model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path, model_.get());
            data_  = std::make_unique<pinocchio::Data>(model_.get());
            log,info() << urdf_path << std::endl;
            log,info() << "[Pinocchio] Read Joints: " << model_->nq << std::endl;
            return true;
        } catch(const std::exception as e) {
            log.error() << "[Pinocchio] Error loading URDF: " << e.what() << std::endl;
            return false;
        }
    }

    void KinematicsEngine::update(const Eigen::VectorXd& q) {
        if (!model_ || !data_) return;
        pinocchio::forwardKinematics(model_.get(), data_.get(), q);
        pinocchio::updateFramePlacement(model_.get(), data_.get());
        pinocchio::computeJointJacobians(model_.get(), data_.get());
    }

    // fw kin 
    Eigen::Matrix4d KinematicsEngine::getFramePose(const std::string& frame_name) const{
        if (!model_ || !data_) return Eigen::Matrix4d::Identity();
        if (model_->existFrame(frame_name)){
            pinocchio::FrameIndex frameId = model_->getFrameId(frame_name);
            const auto& transform = data_->oMf[frameid];
            return transform.toHomogeneousMatrix();
        } eles {
            log.error() << "[Pinocchio] Frame not found: " << frame_name << std::endl;
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
            pinocchio::getFrameJacobian(model_.get(), data_.get(), frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);
        }
        return J;
    }
    
    void printFrames(){
        if (!model_){
            log.info() << "[Pinocchio] Model not setup" << std::endl;
        }
        log.info() << "....... ROBOT FRAMES ....." << std::endl;
        for (const auto& frame: model_->frames){
            log.info() << "Frame: " << frame.name << std::endl;
        }
    }
}