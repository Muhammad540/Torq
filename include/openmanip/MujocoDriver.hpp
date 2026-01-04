#ifndef MUJOCO_DRIVER_HPP
#define MUJOCO_DRIVER_HPP

#include <string>
#include <iostream>
#include <mujoco/mujoco.h>

namespace openmanip {
    class MujocoDriver {
        public:
            MujocoDriver();
            ~MujocoDriver();
            MujocoDriver(const MujocoDriver&) = delete;

            bool loadModel(const std::string& model_path);
            void step();

            mjModel* getModel() const { if (!model_) { return nullptr; } return model_; }
            mjData* getData() const { if (!data_) { return nullptr; } return data_;}
        
        private:
            mjModel* model_ = nullptr;
            mjData* data_ = nullptr;
    };
}

#endif // MUJOCO_DRIVER_HPP