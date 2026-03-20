# Architecture {#architecture}

## System diagram

Layered view of the Torq stack (top to bottom): inputs (scene, robot, config) → models (Pinocchio, MuJoCo) → RobotSystem → subsystems (HAL, kinematics, controller, GUI) → output (commands, rendered frame).

\dotfile system_diagram.dot

## Public API and ownership

Library users interact only with @b torq::RobotSystem @b. The Controller and
InverseKinematics classes are internal implementation details and are not
exposed in the public API. All manipulation (setTaskSpaceTarget, addTask,
addLimit, addBarrier, IK tuning, update) goes through RobotSystem.

### Ownership hierarchy

| Owner | Owns | Notes |
|-------|------|--------|
| **RobotSystem** | `HardwareInterface`, `KinematicsEngine`, `Controller` | Sole owner of main components. |
| **RobotSystem** | User-added tasks, limits, barriers | Objects passed to `addTask()` / `addLimit()` / `addBarrier()` are owned by RobotSystem and destroyed on `remove*()` or when RobotSystem is destroyed (passed as unique ptrs). |
| **Controller** | `InverseKinematics` solver | Created and used internally. |
| **Controller** | Built-in tasks: `FrameTask`, `PostureTask`, `DampingTask` | Created lazily on first `setTaskSpaceTarget()`. |
| **Controller** | Built-in limits: `VelocityLimit`, `ConfigurationLimit`, optional `FloatingBaseVelocityLimit`, `AccelerationLimit` | Same lifecycle as built-in tasks. |
| **Controller** | Does @e not own user tasks/limits/barriers | These are passed in each tick by RobotSystem and owned by RobotSystem. |

This separation avoids memory leaks and use-after-free: user-added objects
have a single owner (RobotSystem), and the Controller never stores raw
pointers to them across ticks.

## Class hierarchy

### Core

- @b torq::RobotSystem @b : Single entry point for all robot operations. Owns
  `HardwareInterface`, `KinematicsEngine`, `Controller`, and any user-added
  tasks/limits/barriers. Exposes manipulation, IK tuning, and update().
- @b torq::Controller @b : Internal. Manages control modes (`IDLE`, `JOINT_SPACE`, `TASK_SPACE`),
  owns the IK solver and @e built-in task/limit instances, holds `IKConfig`.
  Invoked only by RobotSystem; receives user task/limit/barrier pointers each tick.
- @b torq::InverseKinematics @b : Internal. Assembles the QP from tasks, limits, and barriers; delegates to OSQP.

### Kinematics

- @b torq::KinematicsEngine @b : loads URDF or MJCF, builds full and reduced Pinocchio models.
- @b torq::Configuration @b : immutable snapshot of the robot state at a given \f$q\f$.
  Provides frame transforms, Jacobians, and manifold-aware integration.

### Tasks (QP objective)

- @b torq::Task @b (abstract) : defines `computeError()`, `computeJacobian()`, `computeQPObjective()`.
  - @b torq::FrameTask @b : regulate end-effector SE(3) pose (built-in).
  - @b torq::PostureTask @b : regulate joint angles toward a reference (built-in).
  - @b torq::DampingTask @b : penalize joint velocities (built-in).
  - @b torq::ComTask @b, @b torq::RelativeFrameTask @b, @b torq::LinearHolonomicTask @b, @b torq::JointCouplingTask @b, @b torq::JointVelocityTask @b, @b torq::LowAccelerationTask @b : Use the RobotSystem::addTask() api for custom IK where you wish to add any of these Tasks (e.g. humanoids, quadrupeds).

### Limits (QP inequality constraints)

- @b torq::Limit @b (abstract) : defines `computeQPInequalities()`.
  - @b torq::VelocityLimit @b, @b torq::ConfigurationLimit @b : built-in.
  - @b torq::FloatingBaseVelocityLimit @b, @b torq::AccelerationLimit @b : Add via RobotSystem::addLimit().

### Barriers (CBF inequality + optional objective)

- @b torq::Barrier @b (abstract) : defines `computeBarrier()`, `computeJacobian()`, `computeQPInequalities()`, `computeQPObjective()`.
  - @b torq::PositionBarrier @b, @b torq::BodySphericalBarrier @b, @b torq::SelfCollisionBarrier @b : add via RobotSystem::addBarrier(); owned by RobotSystem.

### Hardware abstraction

- @b torq::HardwareInterface @b (abstract) : `connect`, `step`, `getJointPositions`, `setJointPositions`, …
  - @b torq::MujocoDriver @b : Simulation backend wrapping `mjModel` / `mjData`.
  - @b torq::ServoDriver @b : Interacts with the servos attached to the robot.

### GUI

- @b torq::Gui @b : ImGui + GLFW + OpenGL3 docking interface. Panels for joint control,
  Cartesian jog, IK parameter tuning, and a 3D viewport rendering MuJoCo scenes.

## Ownership diagram

The following diagram shows ownership relations: RobotSystem owns the main
subsystems and any user-added tasks/limits/barriers; Controller owns only the
IK solver and built-in task/limit instances (no direct exposure to library users).

\dotfile ownership_diagram.dot

## Control loop frequencies

Typical frequencies:

- **Task planner / user input** (e.g. GUI jog, programmatic setTaskSpaceTarget): ~60 Hz or event-driven. Target and jog step are set here.
- **RobotSystem::update()** and **Controller::update()** (IK layer): 200–1000 Hz. The user application must call update() at this rate.
- **Hardware step** : Same as the control loop (one step per update()). Physics timestep is set in the MJCF/XML.
- **Joint controller**: [update]

## Data flow per tick

1. The application calls `RobotSystem::update()` at the configured control frequency.
2. `HardwareDriver::step()` advances the physics by one timestep (applying the previous tick’s commands).
3. In TASK_SPACE, the Controller uses its **internal kinematic state** (not the physics state) to build a `Configuration`.
4. Each @b Task @b computes its error and Jacobian from that `Configuration`.
5. Each @b Limit @b computes its inequality rows from the `Configuration`.
6. Each @b Barrier @b computes its inequality rows from the `Configuration`.
7. `InverseKinematics::solve()` assembles the QP and calls OSQP.
8. The resulting \f$\Delta q\f$ is integrated on the manifold [update add explanation about manifold]; the new \f$q\f$ is stored as the internal kinematic state and written to the hardware interface.

## Adding a new robot

1. Place model files (URDF/MJCF + meshes) under `workspace/models/`.
2. Create `workspace/<name>/src/main.cpp` — fill a `torq::RobotConfig` and run the loop.
3. Add `workspace/<name>/CMakeLists.txt` with `target_link_libraries(<name> PRIVATE torq)`.
4. Add `add_subdirectory(workspace/<name>)` to the root `CMakeLists.txt`.
