# Torq

A lightweight C++ library for robot control via differential inverse
kinematics. Inspired by [Pink](https://github.com/stephane-caron/pink) and
[Mink](https://github.com/kevinzakka/mink), built on top of
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (kinematics),
[MuJoCo](https://mujoco.org/) (simulation), and [OSQP](https://osqp.org/)
(QP solver).

At every control tick Torq formulates and solves a Quadratic Program that
finds joint velocities tracking a Cartesian (or joint space) target while
respecting joint, velocity, and Cartesian safety constraints.

The same control code drives a MuJoCo simulation or a real robot by swapping the hardware driver.

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

The following examples are provided:

- `workspace/panda/`: Franka Emika Panda 7 DOF arm (simulation only)
- `workspace/arm_ur5e/`: Universal Robots UR5e (simulation only)
- `workspace/kuka/`: KUKA LBR iiwa 14 (simulation only)
- `workspace/so101/`: SO-101 6 DOF arm (simulation or real ST3215 servos)

To run an example, build from the repo root then run the binary from `build/bin/`:

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
./bin/panda    # or arm_ur5e, kuka, so101
```

## Documentation

For detailed documentation, please refer to the [documentation](https://torq.readthedocs.io/en/latest/).


## Acknowledgements

Torq draws on ideas from:

- [Pink](https://github.com/stephane-caron/pink)
- [Mink](https://github.com/kevinzakka/mink.git)
