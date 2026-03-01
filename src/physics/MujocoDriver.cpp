#include "openmanip/MujocoDriver.hpp"
#include "openmanip/logger.hpp"

#include "Eigen/src/Core/Matrix.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

#include <iostream>

namespace openmanip {
    MujocoDriver::MujocoDriver() {
        logger.info() << "[MujocoDriver] Initializing  ...";
    }

    MujocoDriver::~MujocoDriver() {
        logger.info() << "[MujocoDriver] cleaned up";
    }

    bool MujocoDriver::loadModel(const std::string& model_path) {
        char error[1000];
        mjModel* raw_model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
        if (!raw_model_) {
            logger.error() << "[MujocoDriver] Error: " << error;
            return false;
        }
        model_.reset(raw_model_);

        mjData* raw_data = mj_makeData(model_.get());
        if (!raw_data) return false;
        data_.reset(raw_data);
        return true;
    }

    void MujocoDriver::step() {
        if (model_ && data_) {
            mj_step(model_.get(), data_.get());
        }
    }

    bool MujocoDriver::connect(const std::string& config_path) {
        return loadModel(config_path);
    }

    void MujocoDriver::disconnect() {
        logger.info() << "[MujocoDriver] Disconnected";
        model_.reset();
        data_.reset();
    }

    Eigen::VectorXd MujocoDriver::getJointPositions() const{
        if (!model_ || !data_){
            return Eigen::VectorXd::Zero(0);
        }
        // map the data->qpos values of size nq to eigen map
        return Eigen::Map<const Eigen::VectorXd>(data_->qpos, model_->nq);
    }

    Eigen::VectorXd MujocoDriver::getJointVelocities() const{
        if (!model_ || !data_){
            return Eigen::VectorXd::Zero(0);
        }
        // map the data->qvel values of size nv to eigen map
        return Eigen::Map<const Eigen::VectorXd>(data_->qvel, model_->nv);
    }
    
    void MujocoDriver::overrideJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q){
        if (!model_ || !data_) return;
        if (q.size() != model_->nq) {
            logger.error() << "[MujocoDriver] Size mismatch. "
                           << "Expected " << model_->nq << ", got " << q.size();
            return;
        }
        Eigen::Map<Eigen::VectorXd>(data_->qpos, model_->nq) = q;
        Eigen::Map<Eigen::VectorXd>(data_->ctrl, model_->nu) = q.head(model_->nu);
        Eigen::Map<Eigen::VectorXd>(data_->qvel, model_->nv).setZero();
        mj_forward(model_.get(), data_.get());
    }

    void MujocoDriver::overrideJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd){
        if (!model_ || !data_) return;
        if (qd.size() != model_->nv) {
            logger.error()  << "[MujocoDriver] Size mismatch. "
                            << "Expected " << model_->nv << ", got " << qd.size();
            return;
        }
        Eigen::Map<Eigen::VectorXd>(data_->qvel, model_->nv) = qd;
    }


    void MujocoDriver::setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q){
        if (!model_ || !data_) return;
	int n = static_cast<int>(q.size());
	if (n > model_->nu){
            logger.error() << "[MujocoDriver] Size mismatch. " << "Expected nu <= " << model_->nu << ", got " << n;
            return; 
        }
        Eigen::Map<Eigen::VectorXd>(data_->ctrl, model_->nu).head(n) = q;
    }
    
    void MujocoDriver::setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qdot){
        if (!model_ || !data_) return;
        if (qdot.size() != model_->nu){
            logger.error() << "[MujocoDriver] Size mismatch. " << "Expected nu = " << model_->nu << ", got " << qdot.size();
            return; 
        }
        // TODO: fix based on the actuators used in the sim model, currently assuming for the Position actuators
        const double dt = getTimestep();
        Eigen::Map<Eigen::VectorXd> ctrl_(data_->ctrl, model_->nu);
        ctrl_ += qdot * dt;
    }
    
    double MujocoDriver::getTimestep() const{
        if (!model_) return 0.0;
        return model_->opt.timestep;
    }
    int MujocoDriver::numJoints() const{
        if (!model_) return 0;
        return model_->njnt;
    }

    int MujocoDriver::numActuators() const{
        if (!model_) return 0;
        return model_->nu;
    }

    Eigen::VectorXd MujocoDriver::getCtrl() const{
        if (!model_ || !data_){
            return Eigen::VectorXd::Zero(0);
        }
        return Eigen::Map<const Eigen::VectorXd>(data_->ctrl, model_->nu);
    }
}
