#ifndef MUJOCO_DRIVER_HPP
#define MUJOCO_DRIVER_HPP

#include <memory>
#include <string>
#include <iostream>
#include <mujoco/mujoco.h>

#include "HardwareInterface.hpp"
#include "torq/logger.hpp"

class Logger;

namespace torq {

    /** @brief RAII unique_ptr for mjModel with `mj_deleteModel` deleter. */
    using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

    /** @brief RAII unique_ptr for mjData with `mj_deleteData` deleter. */
    using DataPtr  = std::unique_ptr<mjData,  decltype(&mj_deleteData)>;

    /**
     * @brief MuJoCo simulation backend implementing HardwareInterface.
     *
     * Wraps `mjModel` / `mjData` with RAII smart pointers.  Provides both the
     * actuator-command interface (setJointPositions writes to `ctrl`) and
     * direct state overrides (overrideJointPositions writes to `qpos`).
     *
     * @see HardwareInterface
     */
    class MujocoDriver : public HardwareInterface {
        public:
            MujocoDriver();
            ~MujocoDriver();
            MujocoDriver(const MujocoDriver&) = delete;

            /**
             * @brief Load an MJCF model from disk.
             * @param model_path  Path to the .xml file.
             * @return True on success.
             */
            bool loadModel(const std::string& model_path);

            /** @brief Raw pointer to the MuJoCo model (nullptr if not loaded). */
            mjModel* getModel() const { if (!model_) { return nullptr; } return model_.get(); }

            /** @brief Raw pointer to the MuJoCo data (nullptr if not loaded). */
            mjData* getData() const { if (!data_) { return nullptr; } return data_.get();}

            bool connect(const std::string& config_path) override;
            virtual void disconnect() override;

            virtual Eigen::VectorXd getJointPositions() const override;
            virtual Eigen::VectorXd getJointVelocities() const override;
            virtual Eigen::VectorXd getCtrl() const override;

            /**
             * @brief Directly overwrite qpos (bypassing actuators).
             * @param q  Joint positions to write directly into the MuJoCo state.
             */
            virtual void overrideJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q);

            /**
             * @brief Directly overwrite qvel (bypassing actuators).
             * @param qd  Joint velocities to write directly into the MuJoCo state.
             */
            virtual void overrideJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd);
            
            virtual void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) override;
            virtual void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) override;
            
            virtual void step() override;
            virtual double getTimestep() const override;

            virtual int numJoints() const override;
            virtual int numActuators() const override;
        private:
            ModelPtr model_{nullptr, &mj_deleteModel};
            DataPtr  data_{nullptr,  &mj_deleteData};

            mutable Logger logger;
    };
}

#endif // MUJOCO_DRIVER_HPP
