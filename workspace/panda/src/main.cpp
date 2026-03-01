#include "openmanip/RobotSystem.hpp"
#include "openmanip/Gui.hpp"
#include "openmanip/logger.hpp"
#include <filesystem>

static Logger logger;

int main(int argc, char** argv){
    std::filesystem::path root(PROJECT_ROOT);

    openmanip::RobotConfig config;
    config.scene_path = (root / "workspace/models/franka_emika_panda/scene.xml").string();
    config.robot_model_path = (root / "workspace/models/franka_emika_panda/panda.xml").string();
    config.end_effector_frame = "link8";
    config.locked_joints = {"finger_joint1", "finger_joint2"};

    openmanip::RobotSystem robot;
    if (!robot.initialize(config)){
        logger.error() << "Failed to load model";
        return 1;
    }

    openmanip::Gui gui;
    gui.initialize(&robot, "Franka Panda");

    while (gui.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        gui.render();
    }

    return 0;
}
