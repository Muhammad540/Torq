#include "openmanip/MujocoDriver.hpp"

#include <iostream>

namespace openmanip {
    MujocoDriver::MujocoDriver() {}

    MujocoDriver::~MujocoDriver() {
        if (model_) {mj_deleteModel(model_);}
        if (data_) {mj_deleteData(data_);}
        std::cout << "\033[32mMujocoDriver cleaned up\033[0m" << std::endl;
    }

    bool MujocoDriver::loadModel(const std::string& model_path) {
        char error[1000];
        model_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
        if (!model_) {
            std::cerr << "MujocoDriver::loadModel() Error: " << error << std::endl;
            return false;
        }
        data_ = mj_makeData(model_);
        return true;
    }

    void MujocoDriver::step() {
        if (model_ && data_) {
            mj_step(model_, data_);
        }
    }

}