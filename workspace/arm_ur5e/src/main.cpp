/// @file arm_ur5e example
/// Demonstrates:
///   - PositionBarrier   (keep end-effector above the floor plane)
///   - BodySphericalBarrier (minimum distance between wrist and base)
///   - Built-in VelocityLimit + ConfigurationLimit (automatic)
///   - Built-in FrameTask + PostureTask + DampingTask (automatic)
///   - Interactive GUI with Cartesian jog controls

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
    config.end_effector_frame = "wrist_3_link";

    torq::RobotSystem robot;
    if (!robot.initialize(config)) {
        logger.error() << "Failed to initialize UR5e";
        return 1;
    }

    // ── Barriers ──────────────────────────────────────────────────────
    // 1. Keep the end-effector Z coordinate above the floor (z >= 0.01 m).
    Eigen::VectorXd z_min(1);
    z_min << 0.5;

    Eigen::VectorXd z_max(1);
    z_max << 1.0;
    robot.addBarrier(std::make_unique<torq::PositionBarrier>(
        "wrist_3_link",
        std::vector<int>{2},        // z-axis only
        z_min,                      // lower bound
        z_max,               // no upper bound
        10.0,                       // barrier gain
        1.0                         // safe-displacement gain
    ));

    // 2. Minimum spherical distance between wrist and base (>= 0.15 m).
    robot.addBarrier(std::make_unique<torq::BodySphericalBarrier>(
        "wrist_3_link",
        "base",
        0.15,                       // d_min [m]
        10.0,                       // barrier gain
        3.0                         // safe-displacement gain
    ));

    logger.info() << "UR5e loaded with PositionBarrier (floor) + BodySphericalBarrier (base clearance)";

    torq::Gui gui;
    gui.initialize(&robot, "UR5e – Barriers Demo");

    while (gui.windowIsOpen()) {
        for (int i = 0; i < 10; ++i) {
            robot.update();
        }
        gui.render();
    }

    return 0;
}
