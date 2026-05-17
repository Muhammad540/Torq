#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include "torq/logger.hpp"
#include <filesystem>

static Logger logger;

int main(int argc, char** argv){
    std::filesystem::path root(PROJECT_ROOT);

    torq::RobotConfig config;
    config.scene_path = (root / "workspace/models/franka_emika_panda/scene.xml").string();
    config.robot_model_path = (root / "workspace/models/franka_emika_panda/panda.xml").string();
    config.end_effector_frame = "hand";
    config.locked_joints = {"finger_joint1", "finger_joint2"};
    config.joint_velocity_limit_rad_s = 2.5;

    torq::RobotSystem robot;
    if (!robot.initialize(config)){
        logger.error() << "Failed to load model";
        return 1;
    }
    Eigen::VectorXd home_position(7);
    home_position << 0, 0, 0, -1.62, 0.03, 1.72, 0.88;
    robot.setHomePosition(home_position);
    robot.moveToHome();

    torq::Gui gui;
    gui.initialize(&robot, "Franka Panda");

    while (gui.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        gui.render();
    }

    return 0;
}
