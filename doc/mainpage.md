# Torq {#mainpage}

@b A lightweight C++ library for robot control via differential inverse kinematics.

Torq is a small, focused robotics library written in C++. It exposes
a single high level API ([torq::RobotSystem](@ref torq::RobotSystem)) that
encapsulates kinematics, control, simulation, and visualization. It builds a **Quadratic Program** at every control tick and solves for the joint velocities that best track a Cartesian target while respecting physical and safety constraints.

The design is inspired by [Pink](https://github.com/stephane-caron/pink) and
[Mink](https://github.com/kevinzakka/mink), reformulated in C++:

| Library | Role |
|---------|------|
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | Rigid body kinematics, Jacobians, manifold operations |
| [MuJoCo](https://mujoco.org/) | Physics simulation and visualization scene |
| [OSQP](https://osqp.org/) | Sparse Quadratic Program solver |

---

## What happens at each Tick ?

Torq solves:

\f[
\min_{\Delta q}\; \tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q
\quad\text{s.t.}\quad G\,\Delta q \le h
\f]

where \f$P, c\f$ come from **tasks** (objectives like end effector tracking)
and \f$G, h\f$ come from **limits** (joint position/velocity bounds) and
**barriers** (Cartesian safety regions). The resulting \f$\Delta q\f$ is
integrated on the configuration manifold and sent to the hardware.

---

## Philosophy

- @b One @b API. The user only ever sees `torq::RobotSystem`. The QP solver,
  task composition, and hardware driver are encapsulated.
- @b Same @b code for @b sim @b or @b real. A `HardwareInterface` abstracts the
  driver. So it is easy to switch between MuJoCo and a real robot.
- @b Composable. Tasks, limits, and barriers all share an abstract base and
  are added through `addTask` / `addLimit` / `addBarrier`. So custom subclasses
  are first class citizens.

---

## What you can build today

- Cartesian end effector control of any robot.
- Real time joint and pose tuning through the built in GUI.
- Sim to real workflows: same control code drives a MuJoCo scene or a real robot given that you provide the right hardware driver.

## Future goals
- @b Collision support: Pinocchio based self collision avoidance.
- @b Trajectory planning: joint and Cartesian path generation (RRT, splines).
- @b Robot learning: batched simulation rollouts for imitation and reinforcement learning.

---

## Quick start

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"

int main() {
    torq::RobotConfig cfg;
    cfg.scene_path         = "workspace/models/franka_emika_panda/scene.xml";
    cfg.robot_model_path   = "workspace/models/franka_emika_panda/panda.xml";
    cfg.end_effector_frame = "hand";

    torq::RobotSystem robot;
    robot.initialize(cfg);

    torq::Gui gui;
    gui.initialize(&robot, "Panda");

    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
}
```

Working examples for the Franka Panda, UR5e, KUKA iiwa 14, and SO-101 arms are present under the `workspace/` folder. See [architecture](@ref architecture) for details.

---

## Documentation

| Page | Contents |
|------|----------|
| @subpage quickstart_page | A minimal example |
| @subpage architecture | Class hierarchy, ownership, control loop, adding new robots |
| @subpage qp_formulation | The Quadratic Program: how tasks, limits and barriers compose |
| @subpage tasks_page | Built in tasks and their parameters |
| @subpage limits_page | Built in limits |
| @subpage barriers_page | Built in safety barriers |
| @subpage extending_page | Writing your own task, limit or barrier |
| @subpage tuning_guide | Tuning the IK parameters |
| @subpage sim_to_real | Running on real hardware |
| @subpage conventions | Notation and frame conventions |

---

## Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
```

Then run any of the workspace examples, e.g. `./bin/panda`.

To regenerate this documentation:

```bash
doxygen Doxyfile
xdg-open docs/html/index.html
```
