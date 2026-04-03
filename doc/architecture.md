# Architecture {#architecture}

## System diagram

Layered view of the Torq stack (top to bottom): inputs (scene, robot, config)
→ models (Pinocchio, MuJoCo) → RobotSystem → subsystems (HAL, kinematics,
controller, GUI) → output (commands, rendered frame).

\dotfile system_diagram.dot

## Public API and ownership

Library users interact only with @b torq::RobotSystem. The Controller and
InverseKinematics classes are internal implementation details and are not
exposed in the public API. All manipulation (setTaskSpaceTarget, addTask,
addLimit, addBarrier, IK tuning, update) goes through RobotSystem.

### Ownership hierarchy

| Owner | Owns | Notes |
|-------|------|--------|
| **RobotSystem** | `HardwareInterface`, `KinematicsEngine`, `Controller` | Sole owner of main components. |
| **RobotSystem** | User-added tasks, limits, barriers | Objects passed to `addTask()` / `addLimit()` / `addBarrier()` are owned by RobotSystem and destroyed on `remove*()` or when RobotSystem is destroyed (passed as `unique_ptr`). |
| **Controller** | `InverseKinematics` solver | Created and used internally. |
| **Controller** | Built-in tasks: `FrameTask`, `PostureTask`, `DampingTask` | Created lazily on first `setTaskSpaceTarget()`. |
| **Controller** | Built-in limits: `VelocityLimit`, `ConfigurationLimit`, optional `FloatingBaseVelocityLimit`, `AccelerationLimit` | Same lifecycle as built-in tasks. |
| **Controller** | Does @e not own user tasks/limits/barriers | These are passed in each tick by RobotSystem and owned by RobotSystem. |

This separation avoids memory leaks and use-after-free: user-added objects
have a single owner (RobotSystem), and the Controller never stores raw
pointers to them across ticks.

## Class hierarchy

### Core

- @b torq::RobotSystem : Single entry point for all robot operations. Owns
  `HardwareInterface`, `KinematicsEngine`, `Controller`, and any user-added
  tasks/limits/barriers. Exposes manipulation, IK tuning, and `update()`.
- @b torq::Controller : Internal. Manages control modes (`IDLE`, `JOINT_SPACE`, `TASK_SPACE`),
  owns the IK solver and built-in task/limit instances, holds `IKConfig`.
  Invoked only by RobotSystem; receives user task/limit/barrier pointers each tick.
- @b torq::InverseKinematics : Internal. Assembles the QP from tasks, limits, and barriers; delegates to OSQP.

### Kinematics

- @b torq::KinematicsEngine : Loads URDF or MJCF, builds full and reduced Pinocchio models.
- @b torq::Configuration : Immutable snapshot of the robot state at a given \f$q\f$.
  Provides frame transforms, Jacobians, CoM computation, collision queries,
  and manifold-aware integration. See @ref conventions for manifold operations.

### Tasks (QP objective) — see @ref tasks_page

- @b torq::Task (abstract) : defines `computeError()`, `computeJacobian()`, `computeQPObjective()`.
  - @b torq::FrameTask : Regulate end-effector SE(3) pose (built-in).
  - @b torq::PostureTask : Regulate joint angles toward a reference (built-in).
  - @b torq::DampingTask : Penalise joint velocities (built-in).
  - @b torq::ComTask : Centre-of-mass regulation.
  - @b torq::RelativeFrameTask : Pose of one frame relative to another.
  - @b torq::LinearHolonomicTask : General linear constraint \f$A(q_0 \ominus q) = b\f$.
  - @b torq::JointCouplingTask : Couple joint angles via ratios (extends `LinearHolonomicTask`).
  - @b torq::JointVelocityTask : Track reference joint velocities.
  - @b torq::LowAccelerationTask : Minimise joint accelerations.

  Use `RobotSystem::addTask()` for any user tasks (e.g. humanoids, quadrupeds).

### Limits (QP inequality constraints) — see @ref limits_page

- @b torq::Limit (abstract) : defines `computeQPInequalities()`.
  - @b torq::VelocityLimit, @b torq::ConfigurationLimit : Built-in.
  - @b torq::FloatingBaseVelocityLimit : For floating-base robots. Add via `RobotSystem::addLimit()`.
  - @b torq::AccelerationLimit : Acceleration + braking-distance bounds. Add via `RobotSystem::addLimit()`.

### Barriers (CBF inequality + optional objective) — see @ref barriers_page

- @b torq::Barrier (abstract) : defines `computeBarrier()`, `computeJacobian()`, `computeQPInequalities()`, `computeQPObjective()`.
  - @b torq::PositionBarrier : Cartesian position bounds on a frame.
  - @b torq::BodySphericalBarrier : Minimum distance between two frames.
  - @b torq::SelfCollisionBarrier : Self-collision avoidance via geometry model.

  All barriers are added via `RobotSystem::addBarrier()` and owned by RobotSystem.

### Hardware abstraction

- @b torq::HardwareInterface (abstract) : `connect`, `step`, `getJointPositions`, `setJointPositions`, …
  - @b torq::MujocoDriver : Simulation backend wrapping `mjModel` / `mjData`.
  - @b torq::ServoDriver : Serial servo communication (ST3215/STS/SMS) for real robots.

### GUI

- @b torq::Gui : ImGui + GLFW + OpenGL3 docking interface. Panels for joint control,
  Cartesian jog, IK parameter tuning, and a 3D viewport rendering MuJoCo scenes.

## Ownership diagram

The following diagram shows ownership relations: RobotSystem owns the main
subsystems and any user-added tasks/limits/barriers; Controller owns only the
IK solver and built-in task/limit instances (no direct exposure to library users).

\dotfile ownership_diagram.dot

## Control loop frequencies

Typical frequencies:

- **Task planner / user input** (e.g. GUI jog, programmatic `setTaskSpaceTarget`): ~60 Hz or event-driven.
- **RobotSystem::update()** and **Controller::update()** (IK layer): 200–1000 Hz. The user application must call `update()` at this rate.
- **Hardware step**: Same as the control loop (one step per `update()`). Physics timestep is set in the MJCF/XML.
- **Joint controller (real hardware)**: Internal to the servo (e.g. ST3215 runs a PID loop internally at ~1 kHz). Torq sends position setpoints at the control rate; the servo tracks them.

## Data flow per tick

1. The application calls `RobotSystem::update()` at the configured control frequency.
2. `HardwareDriver::step()` advances the physics by one timestep (applying the previous tick's commands).
3. In TASK_SPACE, the Controller uses its **internal kinematic state** (not the physics state) to build a `Configuration`.
4. Each @b Task computes its error \f$e_i(q)\f$ and Jacobian \f$J_i(q)\f$ from that `Configuration`.
5. Each @b Limit computes its inequality rows \f$(G_j, h_j)\f$ from the `Configuration` and \f$\Delta t\f$.
6. Each @b Barrier computes its inequality rows and optional objective terms from the `Configuration`.
7. `InverseKinematics::solve()` assembles the QP (tasks + barriers → objective; limits + barriers → constraints) and calls OSQP.
8. The resulting \f$\Delta q\f$ is integrated on the manifold using `pinocchio::integrate`:
   \f$q_{\text{new}} = q \oplus \Delta q\f$. This correctly handles quaternion joints (floating base) by using exponential integration, while reducing to simple addition for revolute/prismatic joints. See @ref conventions for details.
9. The new \f$q\f$ is stored as the internal kinematic state and written to the hardware interface as the next commanded position.

## Adding a new robot

1. Place model files (URDF/MJCF + meshes) under `workspace/models/`.
2. Create `workspace/<name>/src/main.cpp` — fill a `torq::RobotConfig` and run the loop.
3. Add `workspace/<name>/CMakeLists.txt` with `target_link_libraries(<name> PRIVATE torq)`.
4. Add `add_subdirectory(workspace/<name>)` to the root `CMakeLists.txt`.

See @ref quickstart_page for a step-by-step walkthrough.
