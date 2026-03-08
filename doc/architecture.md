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

## Data flow per tick

1. `RobotSystem::update()` reads joint state from `HardwareInterface`.
2. The controller builds a `Configuration` from the current \f$q\f$.
3. Each @b Task @b computes its error and Jacobian from the `Configuration`.
4. Each @b Limit @b computes its inequality rows from the `Configuration`.
5. `InverseKinematics::solve()` assembles the QP and calls OSQP.
6. The resulting \f$\Delta q\f$ is integrated on the manifold, and the new \f$q\f$
   is written back to the hardware interface.

## Adding a new robot

1. Place model files (URDF/MJCF + meshes) under `workspace/models/`.
2. Create `workspace/<name>/src/main.cpp` — fill a `torq::RobotConfig` and run the loop.
3. Add `workspace/<name>/CMakeLists.txt` with `target_link_libraries(<name> PRIVATE torq)`.
4. Add `add_subdirectory(workspace/<name>)` to the root `CMakeLists.txt`.
