#ifndef ROBOT_SYSTEM_HPP
#define ROBOT_SYSTEM_HPP

#include "MujocoDriver.hpp"

#include <string>
#include <memory>
namespace openmanip {
    class MujocoDriver;
    class RobotSystem {
        public:
            RobotSystem();
            ~RobotSystem();

            bool initialize(const std::string& model_path);
            void update();
            MujocoDriver* getPhysics();

        private:
            std::unique_ptr<MujocoDriver> physics_;
    };
}

#endif // ROBOT_SYSTEM_HPP