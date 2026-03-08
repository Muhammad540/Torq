# Torq {#mainpage}

@b High performance C++ framework for Robot Control and Training.

Torq combines [MuJoCo](https://mujoco.org/) (physics simulation) and
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body kinematics & dynamics)
behind a concise API.  The library solves differential inverse kinematics as a
Quadratic Program at every control tick, supporting configurable tasks, limits,
and (planned) barrier functions.

---

## Quick start

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"

int main() {
    torq::RobotConfig config;
    config.scene_path            = "path/to/scene.xml";
    config.robot_model_path      = "path/to/robot.urdf";
    config.end_effector_frame    = "ee_frame";
    config.locked_joints         = {"joint_6"};
    config.gripper_actuator_idx  = -1;

    torq::RobotSystem robot;
    robot.initialize(config);

    torq::Gui gui;
    gui.initialize(&robot);

    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
}
```

## Documentation topics

| Page | Contents |
|------|----------|
| @subpage architecture | System diagram, class hierarchy, data flow |
| @subpage qp_formulation | Full QP derivation — objective, constraints, solver |
| @subpage tasks_page | Task types with mathematical derivations |
| @subpage limits_page | Limit types with constraint formulations |
| @subpage barriers_page | Planned barrier (CBF) system |
| @subpage tuning_guide | Parameter reference and tuning recipes |

## Key design choices

- @b Library + workspace @b — `libtorq` is a shared library; robot-specific apps
  in `workspace/` link against it.
- @b HAL (Hardware Abstraction Layer) @b — control code targets
  `torq::HardwareInterface`; swap `MujocoDriver` for a real driver without
  touching control logic.
- @b Zero-allocation hot path @b — the IK solver reuses pre-allocated Eigen
  matrices; no heap allocation per tick.
- @b Runtime-configurable IK @b — every task cost, gain, damping, and limit gain
  is adjustable at runtime through the API and the built-in ImGui tuning panel.
