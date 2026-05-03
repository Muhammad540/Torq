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
    config.end_effector_frame = "wrist_3_link";

    torq::RobotSystem robot;
    if (!robot.initialize(config)) {
        logger.error() << "Failed to initialize UR5e";
        return 1;
    }
    Eigen::VectorXd home_position(6);
    home_position << 0.0212908, -2.12746, 1.96702, -1.40262, 4.71712, 6.2831;
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
