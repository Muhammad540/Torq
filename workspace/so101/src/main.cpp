#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include "torq/logger.hpp"
#include <filesystem>

static Logger logger;

int main(int argc, char** argv) {
    std::filesystem::path root(PROJECT_ROOT);

    torq::RobotConfig config;
    config.scene_path = (root / "workspace/models/SO101/scene.xml").string();
    config.robot_model_path = (root / "workspace/models/SO101/so101_new_calib.urdf").string();
    config.robot_calib_file = (root / "workspace/so101/calibration/calibration.txt").string();
    config.end_effector_frame = "gripper_frame_link";
    config.locked_joints = {"gripper"};
    config.driver_type = "mujoco";
    config.active_control = true;

    torq::RobotSystem robot;
    if (!robot.initialize(config)){
        logger.error() << "[S0101] Failed to load model";
        return 1;
    }

    torq::Gui gui;
    gui.initialize(&robot, "SO101 Arm");

    while (gui.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        gui.render();
    }

    return 0;
}
