# Getting Started {#quickstart_page}

This tutorial walks through building your first Torq application: a robot
arm tracking a Cartesian target in simulation. By the end you will understand
the core control loop and the role of tasks, limits, and tuning parameters.

<!-- Video placeholder: record the SO101 arm tracking a jog target
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/quickstart_demo.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
\endhtmlonly
-->

---

## Prerequisites

- Torq built and installed (see `README.md` for build instructions).
- A robot model (URDF) and a MuJoCo scene (MJCF XML) in `workspace/models/`.

---

## Step 1: Configure the robot

All interaction with Torq goes through `torq::RobotSystem`. Start by
filling a `torq::RobotConfig`:

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"

int main() {
    torq::RobotConfig config;
    config.scene_path         = "workspace/models/SO101/scene.xml";
    config.robot_model_path   = "workspace/models/SO101/so101_new_calib.urdf";
    config.end_effector_frame = "Fixed_Jaw";
    config.locked_joints      = {"Jaw"};

    torq::RobotSystem robot;
    if (!robot.initialize(config)) {
        return 1;  // Model failed to load
    }
```

| Field | Purpose |
|-------|---------|
| `scene_path` | MuJoCo XML scene for simulation and GUI rendering. |
| `robot_model_path` | URDF (or MJCF) for building the Pinocchio kinematic model. |
| `end_effector_frame` | Frame name used by the built-in `FrameTask`. |
| `locked_joints` | Joints excluded from the IK solve (e.g. gripper). |

---

## Step 2: Create the GUI

The built-in GUI provides a 3D viewport, joint control panel, Cartesian
jog controls, and an IK tuning panel:

```cpp
    torq::Gui gui;
    gui.initialize(&robot);
```

---

## Step 3: Run the control loop

The main loop calls `robot.update()` and `gui.render()` for visualisation
(as often as you like; the examples use one update per GUI frame):

```cpp
    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
    return 0;
}
```

Each `robot.update()` call:

1. Steps the hardware (simulation physics or real servo read).
2. Builds a kinematic `Configuration` from the current joint state.
3. Each **task** computes its error and Jacobian.
4. Each **limit** computes its inequality constraint rows.
5. Each **barrier** computes its CBF constraint rows.
6. The QP solver (OSQP) finds the optimal \f$\Delta q\f$.
7. The result is integrated on the manifold: \f$q_{\text{new}} = q \oplus \Delta q\f$.

---

## Step 4: Drive the end-effector

Use the GUI's **Cartesian Jog** panel to move the end-effector interactively.
Programmatically, set a target pose:

```cpp
    // Set a Cartesian target for the built-in FrameTask
    pinocchio::SE3 target = pinocchio::SE3::Identity();
    target.translation() = Eigen::Vector3d(0.15, 0.0, 0.25);
    robot.setTaskSpaceTarget(target);
```

The built-in `FrameTask` will drive the end-effector toward this pose. The
QP also includes a `PostureTask` (regularisation) and a `DampingTask`
(velocity smoothing) by default.

---

## What happens under the hood

At each tick the IK solver minimises:

\f[
\min_{\Delta q} \;\tfrac{1}{2}\,\Delta q^\top
  \underbrace{\left[\sum_i H_i + \lambda I\right]}_{P}\,\Delta q
  + \underbrace{\left[\sum_i c_i\right]^\top}_{c^\top}\,\Delta q
\quad\text{s.t.}\quad G\,\Delta q \le h
\f]

where:
- \f$H_i, c_i\f$ come from the **FrameTask**, **PostureTask**, and **DampingTask**
  (plus any user-added tasks and barriers).
- \f$G, h\f$ come from **VelocityLimit** and **ConfigurationLimit**
  (plus any user-added limits and barriers).

See @ref qp_formulation for the full derivation.

---

## Step 5: Tune the parameters

The built-in ImGui **IK Tuning** panel lets you adjust all parameters at
runtime. Here are the most important ones:

| Parameter | What it does | Start here |
|-----------|-------------|------------|
| Frame position cost | How strongly the end-effector tracks position | 1.0 |
| Frame orientation cost | How strongly orientation is tracked | 1.0 |
| Frame gain (\f$\alpha\f$) | Convergence speed (1.0 = instant, 0.5 = smooth) | 0.5 – 1.0 |
| Posture cost | How strongly joints stay near the home pose | 1e-3 |
| Damping cost | How much joint velocity is penalised | 1e-4 |
| Config limit gain (\f$\gamma\f$) | How early the robot decelerates near joint limits | 0.5 |

See @ref tuning_guide for a complete reference.

---

## Complete example

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"

int main() {
    // 1. Configure
    torq::RobotConfig config;
    config.scene_path         = "workspace/models/SO101/scene.xml";
    config.robot_model_path   = "workspace/models/SO101/so101_new_calib.urdf";
    config.end_effector_frame = "Fixed_Jaw";
    config.locked_joints      = {"Jaw"};

    // 2. Initialise
    torq::RobotSystem robot;
    robot.initialize(config);

    torq::Gui gui;
    gui.initialize(&robot);

    // 3. Optionally set a target pose
    pinocchio::SE3 target = pinocchio::SE3::Identity();
    target.translation() = Eigen::Vector3d(0.15, 0.0, 0.25);
    robot.setTaskSpaceTarget(target);

    // 4. Run
    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
    return 0;
}
```

---

## Next steps

- @ref tutorials_page — More detailed tutorials with video walkthroughs.
- @ref tasks_page — All available task types.
- @ref limits_page — All available limit types.
- @ref barriers_page — Barrier functions for safety.
- @ref extending_page — How to add your own tasks, limits, and barriers.
