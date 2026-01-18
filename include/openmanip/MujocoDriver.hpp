#ifndef MUJOCO_DRIVER_HPP
#define MUJOCO_DRIVER_HPP

#include <memory>
#include <string>
#include <iostream>
#include <mujoco/mujoco.h>

#include "HardwareInterface.hpp"

namespace openmanip {

    using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
    using DataPtr  = std::unique_ptr<mjData,  decltype(&mj_deleteData)>;   
    class MujocoDriver : public HardwareInterface {
        public:
            MujocoDriver();
            ~MujocoDriver();
            MujocoDriver(const MujocoDriver&) = delete;

            bool loadModel(const std::string& model_path);

            mjModel* getModel() const { if (!model_) { return nullptr; } return model_.get(); }
            mjData* getData() const { if (!data_) { return nullptr; } return data_.get();}

            bool connect(const std::string& config_path) override;
            virtual void disconnect() override;

            virtual Eigen::VectorXd getJointPositions() const override;
            virtual Eigen::VectorXd getJointVelocities() const override;
            
            virtual void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) override;
            virtual void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) override;
            
            virtual void step() override;
            virtual double getTimestep() const override;

            virtual int numJoints() const override;
        private:
            ModelPtr model_{nullptr, &mj_deleteModel};
            DataPtr  data_{nullptr,  &mj_deleteData};
    };
}

#endif // MUJOCO_DRIVER_HPP