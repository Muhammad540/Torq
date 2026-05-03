# Architecture {#architecture}

Torq is structured around a single class, `torq::RobotSystem`, that
owns every other component. The split between the **public API** and the **internal control stack** is deliberate: users never instantiate the `Controller` or `InverseKinematics` classes directly.

## System diagram

\dotfile system_diagram.dot

The data flow is one directional per tick:

1. `RobotSystem::update()` calls `HardwareInterface::step()` (advance physics
   or read data from the hardware).
2. The current joint state is wrapped in a `Configuration` (Pinocchio FK).
3. Each active **Task** contributes \f$(H_i, c_i)\f$ to the QP objective;
   each **Limit** and **Barrier** contributes inequality rows
   \f$(G_j, h_j)\f$.
4. `InverseKinematics::solve()` calls OSQP and returns \f$\Delta q\f$.
5. \f$q \leftarrow q \oplus \Delta q\f$ (manifold integration) and the new
   target is sent back to the hardware.

Each update call advances the system by exactly one timestep (`HardwareInterface::getTimestep()`).

---

## Class hierarchy

\dotfile ownership_diagram.dot

| Layer | Class | Role |
|-------|-------|------|
| Orchestrator | @ref torq::RobotSystem | Single entry point. Owns everything below. |
| Control | @ref torq::Controller | Holds built in tasks/limits, runs the IK loop. |
| Solver | @ref torq::InverseKinematics | Builds the QP and calls OSQP. |
| Kinematics | @ref torq::KinematicsEngine, @ref torq::Configuration | Pinocchio model and per tick FK snapshot. |
| Hardware | @ref torq::HardwareInterface (abstract) | Driver agnostic state and command API. |
| Driver | @ref torq::MujocoDriver, @ref torq::ServoDriver | Simulation and serial bus implementations. |
| GUI | @ref torq::Gui | ImGui + MuJoCo viewport, joint and IK tuning panels. |

### Tasks, Limits, Barriers

All three follow the same pattern: an abstract base + a few concrete
implementations. Users compose them by passing `unique_ptr` into
`RobotSystem::addTask` / `addLimit` / `addBarrier`.

| Base | Built-in concrete classes | User extends? |
|------|---------------------------|---------------|
| @ref torq::Task | `FrameTask`, `PostureTask`, `DampingTask` | Yes — see @ref extending_page |
| @ref torq::Limit | `VelocityLimit`, `ConfigurationLimit` | Yes |
| @ref torq::Barrier | `PositionBarrier`, `BodySphericalBarrier` | Yes |

`FrameTask`, `PostureTask`, `DampingTask`, `VelocityLimit` and
`ConfigurationLimit` are created automatically by the `Controller` the first
time you call `setTaskSpaceTarget`. Anything you add via `addTask` etc. is
appended on top each tick.

---

## Simulation integration

Simulation is implemented through `MujocoDriver`, an implementation of
`HardwareInterface` that wraps `mjModel` / `mjData`. Three modes are
supported:

- @b Pure @b simulation
- @b Real @b hardware
- @b Real @b hardware @b + @b sim @b visualization

---

## Adding a new robot

Three working examples are provided that can be used as a template for a new robot:

| Folder | Robot | Notes |
|--------|-------|-------|
| `workspace/panda/`     | Franka Emika Panda 7-DOF arm | Pure simulation |
| `workspace/arm_ur5e/`  | Universal Robots UR5e        | Pure simulation |
| `workspace/so101/`     | SO-101 6-DOF arm             | Sim or real (ST3215 servos) |

To bring up a new robot:

1. Add the model files (URDF/MJCF + meshes) into `workspace/models/<name>/`.
2. Copy a workspace folder
   `workspace/<name>/` and update the `src/main.cpp` file:
   - `scene_path`         → MuJoCo scene XML (for sim and visualization)
   - `robot_model_path`   → URDF or MJCF for kinematics
   - `end_effector_frame` → frame name in your URDF that you want to control
   - `locked_joints`      → joints to freeze (e.g. fingers)
3. Add `add_subdirectory(workspace/<name>)` to the root `CMakeLists.txt`.
4. Build and run the example.

For real hardware, set `driver_type = "serial_servo"` and provide a calibration file. See @ref sim_to_real.
