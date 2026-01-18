#include "openmanip/MujocoDriver.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"

#include <iostream>

namespace openmanip {
    MujocoDriver::MujocoDriver() {}

    MujocoDriver::~MujocoDriver() {
        std::cout << "\033[32mMujocoDriver cleaned up\033[0m" << std::endl;
    }

    bool MujocoDriver::loadModel(const std::string& model_path) {
        char error[1000];
        mjModel* raw_model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
        if (!raw_model_) {
            std::cerr << "MujocoDriver::loadModel() Error: " << error << std::endl;
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
        std::cout << "\033[32mDisconnected\033[0m" << std::endl;
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
    
    void MujocoDriver::setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q){
        if (!model_ || !data_) return;
        if (q.size() != model_->nq) {
            std::cerr << "MujocoDriver::setJointPositions() Error: Size mismatch. "
                      << "Expected " << model_->nq << ", got " << q.size() << std::endl;
            return;
        }
        Eigen::Map<Eigen::VectorXd>(data_->qpos, model_->nq) = q;
    }

    void MujocoDriver::setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd){
        if (!model_ || !data_) return;
        if (qd.size() != model_->nv) {
            std::cerr << "MujocoDriver::setJointVelocities() Error: Size mismatch. "
                      << "Expected " << model_->nv << ", got " << qd.size() << std::endl;
            return;
        }
        Eigen::Map<Eigen::VectorXd>(data_->qvel, model_->nv) = qd;
    }
    
    double MujocoDriver::getTimestep() const{
        if (!model_) return 0.0;
        return model_->opt.timestep;
    }
    int MujocoDriver::numJoints() const{
        if (!model_) return 0;
        return model_->njnt;
    }
}