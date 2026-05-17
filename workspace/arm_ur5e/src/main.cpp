#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include "torq/Barriers.hpp"
#include "torq/logger.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <filesystem>

static Logger logger;

int main(int argc, char** argv) {
    std::filesystem::path root(PROJECT_ROOT);

    torq::RobotConfig config;
    config.scene_path       = (root / "workspace/models/universal_robots_ur5e/scene.xml").string();
    config.robot_model_path = (root / "workspace/models/universal_robots_ur5e/ur5e.xml").string();
    config.joint_velocity_limit_rad_s = 2.5;
    config.end_effector_frame = "attachment_site";
    config.joint_velocity_limit_rad_s = 2.5;


    torq::RobotSystem robot;
    if (!robot.initialize(config)) {
        logger.error() << "Failed to initialize UR5e";
        return 1;
    }
    Eigen::VectorXd home_position(6);
    home_position <<  -3.98815 ,-1.32759 ,-1.99854 ,-1.39978  ,1.63068   ,6.2831;
    robot.setHomePosition(home_position);
    robot.moveToHome();

    torq::Gui gui;
    gui.initialize(&robot, "UR5e");

    while (gui.windowIsOpen()) {
        for (int i = 0; i < 10; ++i) {
            robot.update();
        }
        gui.render();
    }

    return 0;
}
