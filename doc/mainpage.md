# Torq {#mainpage}

@b High-performance C++ framework for robot control and training.

Torq combines [MuJoCo](https://mujoco.org/) (physics simulation) and
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (rigid-body
kinematics & dynamics) behind a concise API. The library solves differential
inverse kinematics as a Quadratic Program at every control tick, supporting
configurable **tasks**, **limits**, and **barrier functions**.

At each control step, Torq formulates:

\f[
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q \\
\text{s.t.} \quad & G\,\Delta q \le h
\end{aligned}
\f]

**Tasks** specify what the robot should do (e.g. reach a pose, maintain
balance). **Limits** specify what it must not do (e.g. exceed joint bounds).
**Barriers** enforce safety constraints via Control Barrier Functions (e.g.
workspace bounds, collision avoidance).

Torq is designed for **sim-to-real** workflows: the same control code runs
in MuJoCo simulation and on real hardware by swapping only the hardware
driver.

---

## Key features

- @b Composable task abstraction @b — Specify multiple task-space objectives
  (end-effector tracking, posture regulation, CoM control, joint coupling)
  and combine them in a single QP solve.

- @b Hard constraint enforcement @b — Velocity limits, configuration limits,
  acceleration limits, and floating-base limits are enforced as QP inequality
  constraints.

- @b Safety via barrier functions @b — Control Barrier Functions (CBFs)
  for workspace bounds, minimum distance, and self-collision avoidance.

- @b Designed for real-time control @b — Zero-allocation hot path with
  pre-allocated Eigen matrices. No heap allocation per tick.

- @b Runtime-tunable @b — Every task cost, gain, damping, and limit parameter
  is adjustable at runtime through the API and the built-in ImGui tuning panel.

- @b Single entry point @b — Library users interact only with
  `torq::RobotSystem`. All control, task/limit/barrier composition, and
  tuning go through one class with a clear ownership model.

- @b Hardware abstraction @b — Control code targets `torq::HardwareInterface`;
  swap `MujocoDriver` (simulation) for `ServoDriver` (real robot) without
  changing any control logic.

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

<!-- Video placeholder: record a short demo of the SO101 arm tracking a target
     and embed here using raw HTML:

     \htmlonly
     <video width="600" controls style="display: block; margin: 1em auto;">
       <source src="path/to/demo.mp4" type="video/mp4">
       Your browser does not support the video tag.
     </video>
     \endhtmlonly
-->

---

## Documentation

| Page | Contents |
|------|----------|
| @subpage conventions | Notation, coordinate frames, manifold conventions |
| @subpage quickstart_page | Step-by-step getting started tutorial |
| @subpage tutorials_page | Tutorials with worked examples and video walkthroughs |
| @subpage architecture | System diagram, class hierarchy, data flow |
| @subpage qp_formulation | Complete QP derivation: objective, constraints, barriers, solver |
| @subpage tasks_page | All task types with mathematical derivations |
| @subpage limits_page | All limit types with constraint formulations |
| @subpage barriers_page | All barrier types with CBF formulations |
| @subpage extending_page | How to add custom tasks, limits, and barriers |
| @subpage tuning_guide | Parameter reference and tuning recipes |
| @subpage sim_to_real | ServoDriver, hardware setup, and real robot deployment |

---

## Building the documentation

```bash
# From the repository root:
doxygen Doxyfile

# Open in browser:
xdg-open docs/html/index.html   # Linux
open docs/html/index.html        # macOS
```

## Embedding videos in documentation

Torq documentation supports embedded videos for tutorials and demos.
To add a video to any documentation page, use Doxygen's `\htmlonly` block:

```
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/video.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
\endhtmlonly
```

For locally hosted videos, place `.mp4` files in `doc/videos/` and reference
them with a relative path. For GitHub-hosted videos, use a raw URL from
a dedicated assets branch or release.
