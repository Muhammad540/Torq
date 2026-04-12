# Torq

C++ library for robot manipulation: **MuJoCo** for simulation, **Pinocchio** for kinematics, and **QPbased inverse kinematics** (tasks, limits, barriers). A HAL based approach is used to switch between sim and real.

## Dependencies

- Linux
- CMake 3.22+
- **Pinocchio**, **Eigen3**, **OSQP**, **OsqpEigen** (system packages)
- **OpenGL** + **GLEW**
- **MuJoCo**, **Dear ImGui**, **ImPlot** — fetched by CMake when you configure the project

Install Pinocchio using the [official instructions](https://stack-of-tasks.github.io/pinocchio/).

## Build

From the repository root:

```bash
mkdir -p build && cd build
cmake ..
cmake --build . -j
```
## Example

```bash
cd build/bin
./so101          # SO-101 arm (sim or real, depending on config)
./panda          # Panda example
./arm_ur5e
```

## Project layout

```text
Torq/
├── include/torq/         # headers
├── src/
│   ├── core/             # RobotSystem
│   ├── hardware/         # MujocoDriver, ServoDriver, scservo protocol
│   ├── kinematics/       # Pinocchio model / configuration
│   ├── control/          # Controller, InverseKinematics (OSQP)
│   ├── tasks/            # IK tasks
│   ├── limits/           # Joint / velocity limits
│   ├── barriers/         # QP barriers
│   └── gui/              # ImGui + MuJoCo viewport
├── workspace/
│   ├── models/           # URDF / MJCF assets
│   ├── so101/            # SO-101 + calibration
│   ├── panda/
│   └── arm_ur5e/
├── CMakeLists.txt
└── Doxyfile
```

## Overview

- **`libtorq`** — shared library: `RobotSystem` (orchestrator), `Controller` + IK, kinematics, GUI, and drivers.
- **`workspace/*`** — executables that link `libtorq`

## Documentation

API docs are built with Doxygen:

```bash
doxygen Doxyfile
# Open docs/html/index.html
```

## Acknowledgements

Torq draws on ideas from:
- [Pink](https://github.com/stephane-caron/pink)
- [Mink](https://github.com/kevinzakka/mink.git)
