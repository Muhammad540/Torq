# Architecture {#architecture}

## System diagram

Layered view of the Torq stack (top to bottom): inputs (scene, robot, config) → models (Pinocchio, MuJoCo) → RobotSystem → subsystems (HAL, kinematics, controller, GUI) → output (commands, rendered frame).

\dotfile system_diagram.dot

## Class hierarchy

### Core

- @b torq::RobotSystem @b — owns `HardwareInterface`, `KinematicsEngine`, `Controller`.
  Single entry point for all robot operations.
- @b torq::Controller @b — manages control modes (`IDLE`, `JOINT_SPACE`, `TASK_SPACE`),
  owns the IK solver and all task/limit instances, holds the `IKConfig` struct.
- @b torq::InverseKinematics @b — assembles the QP from tasks and limits, delegates to OSQP.

### Kinematics

- @b torq::KinematicsEngine @b — loads URDF or MJCF, builds full and reduced Pinocchio models.
- @b torq::Configuration @b — immutable snapshot of the robot state at a given \f$q\f$.
  Provides frame transforms, Jacobians, and manifold-aware integration.

### Tasks (QP objective)

- @b torq::Task @b (abstract) — defines `computeError()`, `computeJacobian()`, `computeQPObjective()`.
  - @b torq::FrameTask @b — regulate end-effector SE(3) pose.
  - @b torq::PostureTask @b — regulate joint angles toward a reference.
  - @b torq::DampingTask @b — penalize all joint velocities (viscous damping).

### Limits (QP inequality constraints)

- @b torq::Limit @b (abstract) — defines `computeQPInequalities()`.
  - @b torq::VelocityLimit @b — \f$ -\Delta t \, v_{\max} \le \Delta q \le \Delta t \, v_{\max} \f$.
  - @b torq::ConfigurationLimit @b — keep \f$q + \Delta q\f$ within \f$[q_{\min}, q_{\max}]\f$.

### Hardware abstraction

- @b torq::HardwareInterface @b (abstract) — `connect`, `step`, `getJointPositions`, `setJointPositions`, …
  - @b torq::MujocoDriver @b — RAII simulation backend wrapping `mjModel` / `mjData`.

### GUI

- @b torq::Gui @b — ImGui + GLFW + OpenGL3 docking interface. Panels for joint control,
  Cartesian jog, IK parameter tuning, and a 3D viewport rendering MuJoCo scenes.

## Ownership diagram

The following diagram shows ownership relations: RobotSystem owns the main subsystems; Controller owns the IK solver and all task/limit instances.

\dotfile ownership_diagram.dot

## Control loop frequencies

Typical frequencies (configurable where noted):

- **Task planner / user input** (e.g. GUI jog, programmatic setTaskSpaceTarget): ~60 Hz or event-driven. Target and jog step are set here.
- **RobotSystem::update()** and **Controller::update()** (IK layer): 200–1000 Hz. Configure via RobotConfig::control_frequency_hz or setControlFrequencyHz(); the application must call update() at this rate (e.g. using controlPeriodSec() for rate limiting).
- **Physics step** (MujocoDriver::step()): same as the control loop (one step per update()). Physics timestep is set in the MJCF/XML (getTimestep()); frequency = 1 / timestep.
- **Joint controller**: In simulation, MuJoCo’s position actuators act as the joint layer (typically 1 kHz+ internally). On real hardware, the joint controller runs at 1–4 kHz.

Max Cartesian tracking error for jog safety is configurable via RobotConfig::max_tracking_error or setMaxTrackingError() (default 0.05 m).

## Data flow per tick

1. The application calls `RobotSystem::update()` at the configured control frequency.
2. `MujocoDriver::step()` advances the physics by one timestep (applying the previous tick’s commands).
3. In TASK_SPACE, the Controller uses its **internal kinematic state** (not the physics state) to build a `Configuration`.
4. Each @b Task @b computes its error and Jacobian from that `Configuration`.
5. Each @b Limit @b computes its inequality rows from the `Configuration`.
6. `InverseKinematics::solve()` assembles the QP and calls OSQP.
7. The resulting \f$\Delta q\f$ is integrated on the manifold; the new \f$q\f$ is stored as the internal kinematic state and written to the hardware interface.

## Adding a new robot

1. Place model files (URDF/MJCF + meshes) under `workspace/models/`.
2. Create `workspace/<name>/src/main.cpp` — fill a `torq::RobotConfig` and run the loop.
3. Add `workspace/<name>/CMakeLists.txt` with `target_link_libraries(<name> PRIVATE torq)`.
4. Add `add_subdirectory(workspace/<name>)` to the root `CMakeLists.txt`.
