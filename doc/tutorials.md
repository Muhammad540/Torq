# Tutorials {#tutorials_page}

Hands-on tutorials demonstrating common Torq workflows. Each tutorial
includes a complete code example and a video demonstration.

> @b Recording videos: @b To record a tutorial video, run the example with
> screen capture software, then embed the `.mp4` in this page using the
> `\htmlonly` video tag (see @ref mainpage for the template).

---

## Tutorial 1: End-effector pose tracking

Drive the end-effector to a target SE(3) pose using the built-in
`FrameTask`. Demonstrates the basic control loop, gain tuning, and
the effect of position vs orientation cost weighting.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_pose_tracking.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

### Key concepts

- `RobotSystem::setTaskSpaceTarget()` sets the FrameTask target.
- Position-only tracking: set `setFrameTaskOrientationCost(0.0)`.
- The gain \f$\alpha\f$ controls convergence: after \f$n\f$ ticks
  the error is approximately \f$\|e_n\| \approx \|e_0\|(1-\alpha)^n\f$.

### Code

```cpp
#include "torq/RobotSystem.hpp"
#include "torq/Gui.hpp"
#include <cmath>

int main() {
    torq::RobotConfig config;
    config.scene_path         = "workspace/models/SO101/scene.xml";
    config.robot_model_path   = "workspace/models/SO101/so101_new_calib.urdf";
    config.end_effector_frame = "Fixed_Jaw";
    config.locked_joints      = {"Jaw"};

    torq::RobotSystem robot;
    robot.initialize(config);

    torq::Gui gui;
    gui.initialize(&robot);

    // Smooth convergence: reach target in ~2 seconds at 200 Hz
    double alpha = 1.0 - std::pow(0.01, 1.0 / 400.0);
    robot.setFrameTaskGain(alpha);

    // Position-only tracking (orientation free)
    robot.setFrameTaskPositionCost(1.0);
    robot.setFrameTaskOrientationCost(0.0);

    pinocchio::SE3 target = pinocchio::SE3::Identity();
    target.translation() = Eigen::Vector3d(0.15, 0.05, 0.20);
    robot.setTaskSpaceTarget(target);

    while (gui.windowIsOpen()) {
        robot.update();
        gui.render();
    }
}
```

---

## Tutorial 2: Adding regularisation (PostureTask + DampingTask)

Without regularisation, the IK solver can produce unstable behaviour near
singularities or nullspace drift in redundant robots. This tutorial
demonstrates why regularisation matters and how to tune it.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_regularisation.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

### What goes wrong without regularisation

| Problem | Cause | Symptom |
|---------|-------|---------|
| **Singularity instability** | Jacobian becomes ill-conditioned near full extension | Large, erratic joint velocities |
| **Nullspace drift** | Redundant DOFs are unconstrained | Elbow wanders, non-reproducible poses |

### Built-in regularisation

Torq includes `PostureTask` and `DampingTask` by default. Their costs
control the trade-off:

| Task | Default cost | Effect |
|------|-------------|--------|
| PostureTask | \f$10^{-3}\f$ | Biases joints toward a reference posture |
| DampingTask | \f$10^{-4}\f$ | Penalises all joint motion (minimum-norm velocity) |

### Tuning

```cpp
// Increase posture cost to resist nullspace drift more aggressively
robot.setPostureTaskCost(1e-2);

// Increase damping cost for smoother motion
robot.setDampingTaskCost(1e-3);

// Set the reference posture to the current joint angles
robot.setHomePosition(robot.getJointPositions());
```

---

## Tutorial 3: Working with limits

Limits are hard constraints — the solver never violates them. This tutorial
shows how to use velocity limits, configuration limits, and acceleration
limits together.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_limits.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

### Built-in limits

The `VelocityLimit` and `ConfigurationLimit` are included automatically.
They read bounds from the Pinocchio model (URDF).

### Adding an acceleration limit

```cpp
#include "torq/Limits.hpp"

// Create acceleration limit: 10 rad/s² per joint
Eigen::VectorXd a_max = Eigen::VectorXd::Constant(robot.nv(), 10.0);
auto accel_limit = std::make_unique<torq::AccelerationLimit>(
    robot.model(), a_max);

robot.addLimit(std::move(accel_limit));
```

> @b Important: @b `AccelerationLimit` requires calling `setLastIntegration()`
> each tick with the previous velocity. When added via `RobotSystem::addLimit()`,
> the controller handles this automatically.

### Effect of configuration limit gain

The gain \f$\gamma\f$ controls how early the robot decelerates:

| \f$\gamma\f$ | Behaviour |
|---------------|-----------|
| 1.0 | Only stops at the exact joint limit |
| 0.5 | Decelerates when halfway to the limit (default) |
| 0.1 | Very conservative — slows down far from limits |

```cpp
robot.setConfigLimitGain(0.3);  // Conservative for real hardware
```

---

## Tutorial 4: Barrier functions for safety

Barriers enforce safety constraints using Control Barrier Functions (CBFs).
This tutorial demonstrates workspace bounds and collision avoidance.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_barriers.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

### Workspace bounds with PositionBarrier

Keep the end-effector within a box:

```cpp
#include "torq/Barriers.hpp"

// Constrain Z axis: keep end-effector above the table (z > 0.05m)
Eigen::VectorXd z_min(1);
z_min << 0.05;

auto barrier = std::make_unique<torq::PositionBarrier>(
    "Fixed_Jaw",          // frame to monitor
    std::vector<int>{2},  // Z axis only
    z_min,                // lower bound
    std::nullopt,         // no upper bound
    1.0,                  // barrier gain
    0.5                   // safe displacement gain
);

robot.addBarrier(std::move(barrier));
```

### Collision avoidance with BodySphericalBarrier

Maintain minimum distance between two frames:

```cpp
auto barrier = std::make_unique<torq::BodySphericalBarrier>(
    "forearm_link",    // frame 1
    "base_link",       // frame 2
    0.05,              // minimum distance (5 cm)
    1.0,               // barrier gain
    3.0                // safe displacement gain
);

robot.addBarrier(std::move(barrier));
```

### Self-collision avoidance

Monitor the closest collision pairs from the geometry model:

```cpp
auto barrier = std::make_unique<torq::SelfCollisionBarrier>(
    5,     // monitor 5 closest pairs
    1.0,   // gain
    1.0,   // safe displacement gain
    0.02   // minimum distance (2 cm)
);

robot.addBarrier(std::move(barrier));
```

> @b Note: @b `SelfCollisionBarrier` requires a collision model loaded in
> the `Configuration`. Ensure your URDF includes collision geometries.

---

## Tutorial 5: Custom tasks for complex robots

For robots beyond single arms (humanoids, quadrupeds, dual-arm setups),
you need custom tasks. This tutorial shows how to use `ComTask`,
`RelativeFrameTask`, and `JointCouplingTask`.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_custom_tasks.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

### Center of mass regulation

```cpp
#include "torq/Tasks.hpp"

auto com_task = std::make_unique<torq::ComTask>(1.0);
com_task->setTarget(Eigen::Vector3d(0.0, 0.0, 0.5));

robot.addTask(std::move(com_task));
```

### Relative frame tracking (foot relative to pelvis)

```cpp
auto foot_task = std::make_unique<torq::RelativeFrameTask>(
    "left_foot",   // frame to control
    "pelvis",      // reference frame
    1.0,           // position cost
    1.0,           // orientation cost
    0.01           // LM damping
);

pinocchio::SE3 foot_target;  // desired foot pose in pelvis frame
foot_task->setTarget(foot_target);

robot.addTask(std::move(foot_task));
```

### Joint coupling (symmetric knees)

```cpp
auto coupling = std::make_unique<torq::JointCouplingTask>(
    std::vector<std::string>{"left_knee", "right_knee"},
    std::vector<double>{1.0, -1.0},  // same angle, opposite sign
    10.0,                             // high cost to enforce coupling
    robot.configuration()
);

robot.addTask(std::move(coupling));
```

---

## Tutorial 6: Sim-to-real deployment

Run the same control code on a real robot by changing only the hardware
driver configuration. See @ref sim_to_real for a complete guide.

<!-- Video placeholder
\htmlonly
<video width="600" controls style="display: block; margin: 1em auto;">
  <source src="https://your-host.com/path/to/tutorial_sim_to_real.mp4" type="video/mp4">
</video>
\endhtmlonly
-->

```cpp
// Simulation
config.driver_type = "mujoco";

// Real robot — only these lines change
config.driver_type       = "serial_servo";
config.driver_connection = "/dev/ttyUSB0";
config.active_control    = false;  // start in passive mode
```

---

## Recording and attaching tutorial videos

To create a tutorial video:

1. **Record**: Run the example application with screen capture software
   (e.g. OBS Studio, SimpleScreenRecorder, or `ffmpeg` screen capture).

2. **Encode**: Convert to H.264 MP4 for broad browser compatibility:
   ```bash
   ffmpeg -i raw_recording.mkv -c:v libx264 -crf 23 -preset medium \
          -c:a aac -movflags +faststart tutorial_name.mp4
   ```

3. **Host**: Upload to a GitHub release, an assets branch, or a CDN.

4. **Embed**: Use the `\htmlonly` block in the documentation page:
   ```
   \htmlonly
   <video width="600" controls style="display: block; margin: 1em auto;">
     <source src="URL_TO_VIDEO" type="video/mp4">
   </video>
   \endhtmlonly
   ```

5. **Rebuild**: Run `doxygen Doxyfile` to regenerate the documentation.
