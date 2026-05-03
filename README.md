# Torq

A lightweight C++ library for robot control via differential inverse
kinematics. Inspired by [Pink](https://github.com/stephane-caron/pink) and
[Mink](https://github.com/kevinzakka/mink), built on top of
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (kinematics),
[MuJoCo](https://mujoco.org/) (simulation), and [OSQP](https://osqp.org/)
(QP solver).

At every control tick Torq formulates and solves a Quadratic Program that
finds joint velocities tracking a Cartesian (or joint-space) target while
respecting joint, velocity, and Cartesian safety constraints.

The same control code drives a MuJoCo simulation or a real robot —
swap the hardware driver via one config field.

## Dependencies

- Linux
- CMake 3.22+
- **Pinocchio**, **Eigen3**, **OSQP**, **OsqpEigen** (system packages)
- **OpenGL** + **GLEW**
- **MuJoCo**, **Dear ImGui**, **ImPlot** — fetched by CMake

Install Pinocchio via the [official instructions](https://stack-of-tasks.github.io/pinocchio/).

## Build

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
```

## Examples

Three reference robots ship in `workspace/`:

```bash
cd build/bin
./panda     # Franka Emika Panda (sim)
./arm_ur5e  # Universal Robots UR5e (sim)
./so101     # SO-101 6-DOF arm (sim or real ST3215 servos)
```

To bring up your own robot, copy the closest `workspace/<robot>/`
directory and point `RobotConfig` at your URDF/MJCF.

## Project layout

```text
include/torq/         # Public headers
src/
├── core/             # RobotSystem (facade)
├── control/          # Controller, InverseKinematics
├── kinematics/       # Pinocchio model and configuration
├── tasks/            # FrameTask, PostureTask, DampingTask
├── limits/           # VelocityLimit, ConfigurationLimit
├── barriers/         # PositionBarrier, BodySphericalBarrier
├── hardware/         # MujocoDriver, ServoDriver, SCServo protocol
└── gui/              # ImGui + MuJoCo viewport
workspace/
├── models/           # URDF / MJCF assets
├── panda/
├── arm_ur5e/
└── so101/
```

## Documentation

```bash
doxygen Doxyfile
xdg-open docs/html/index.html
```

The generated docs cover the QP formulation, the class hierarchy and
ownership model, every built-in task / limit / barrier, the IK tuning
parameters, and how to run on real hardware.

## Roadmap

- Full Pinocchio-based collision avoidance (self-collision and environment)
- Trajectory planning (RRT and spline-based)
- Batched simulation rollouts for imitation and reinforcement learning

## Acknowledgements

Torq draws on ideas from:

- [Pink](https://github.com/stephane-caron/pink)
- [Mink](https://github.com/kevinzakka/mink.git)
