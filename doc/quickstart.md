# Quick Start {#quickstart_page}

A minimal Torq application sets up a `RobotConfig`, initializes a
`RobotSystem`, and runs `update()` in a loop.

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include <filesystem>

int main() {
    std::filesystem::path root(PROJECT_ROOT);

    torq::RobotConfig cfg;
    cfg.scene_path         = (root / "workspace/models/franka_emika_panda/scene.xml").string();
    cfg.robot_model_path   = (root / "workspace/models/franka_emika_panda/panda.xml").string();
    cfg.end_effector_frame = "hand";
    cfg.locked_joints      = {"finger_joint1", "finger_joint2"};

    torq::RobotSystem robot;
    if (!robot.initialize(cfg)) return 1;

    torq::Gui gui;
    gui.initialize(&robot, "Panda");

    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
}
```

In `robot.update()`:

1. `HardwareInterface::step()` advances physics or read data from the hardware.
2. Build a `Configuration` (Pinocchio forward kinematics at current \f$q\f$).
3. Each task contributes \f$(H_i, c_i)\f$; each limit and barrier contributes \f$(G_j, h_j)\f$.
4. OSQP returns \f$\Delta q\f$.
5. \f$q \leftarrow q \oplus \Delta q\f$ and the result is sent to the
   hardware as the next position command.

## Control the End Effector (or any other frame)

Use the **Cartesian Jog** panel in the GUI, or set a target pose programmatically:

```cpp
pinocchio::SE3 target = pinocchio::SE3::Identity();
target.translation() = Eigen::Vector3d(0.4, 0.0, 0.5);
robot.setTaskSpaceTarget(target.toHomogeneousMatrix(), "hand");
```

Adjust the IK behaviour at runtime via the **IK Tuning** GUI panel or the
`setters` in the `RobotSystem` class (see @ref tuning_guide).

## Working examples

Three reference applications in `workspace/`:

- `workspace/panda/`     : Franka Emika Panda (simulation only)
- `workspace/arm_ur5e/`  : Universal Robots UR5e (simulation only)
- `workspace/so101/`     : SO-101 arm (simulation or real ST3215 servos)
