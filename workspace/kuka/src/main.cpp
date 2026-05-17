#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include "torq/logger.hpp"
#include <filesystem>

static Logger logger;

int main(int argc, char** argv){
    std::filesystem::path root(PROJECT_ROOT);

    torq::RobotConfig config;
    config.scene_path = (root / "workspace/models/kuka_iiwa_14/scene.xml").string();
    config.robot_model_path = (root / "workspace/models/kuka_iiwa_14/iiwa14.xml").string();
    config.end_effector_frame = "attachment_site";
    config.locked_joints = {};                    
    config.joint_velocity_limit_rad_s = 2.5;

    torq::RobotSystem robot;
    if (!robot.initialize(config)){
        logger.error() << "Failed to load model";
        return 1;
    }
    Eigen::VectorXd home_position(7);
    home_position <<  2.07595 ,-0.715193     ,1.703   ,1.75389    ,-2.194 ,-0.758277   ,3.05433;
    robot.setHomePosition(home_position);
    robot.moveToHome();

    torq::Gui gui;
    gui.initialize(&robot, "Kuka");

    while (gui.windowIsOpen()){
        for (int i=0; i<10; ++i){
            robot.update();
        }
        gui.render();
    }

    return 0;
}
