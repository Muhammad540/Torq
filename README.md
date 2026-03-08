# Torq

High-performance C++ framework for robot manipulation that combines **MuJoCo** (physics + contacts) and **Pinocchio** (rigid body dynamics + kinematics) behind a concise API. Designed for QP-based inverse kinematics with configurable tasks, limits, and barriers, plus a **HAL** for sim-to-real switching.

## Dependencies

- Linux (Ubuntu recommended)
- CMake ≥ 3.22
- Pinocchio (system install)
- Eigen3
- MuJoCo (fetched by CMake)
- OSQP + OsqpEigen (system install)

### Install Pinocchio (Ubuntu)

Follow the official instructions: https://stack-of-tasks.github.io/pinocchio/

## Build

```bash
git clone <this-repo> && cd torq
mkdir -p build && cd build
cmake ..
cmake --build . -j
```

### CMake Options

| Option | Description | Default |
|--------|-------------|---------|
| `ENABLE_TRACKING_POINTS` | Visualize user-defined tracking points | `OFF` |

## Run (SO-101 example)

```bash
cd build/bin
./so101
```

## Project Layout

```text
torq/
├── include/torq/          # Public API headers
├── src/
│   ├── core/              # RobotSystem
│   ├── physics/           # MujocoDriver
│   ├── kinematics/        # PinocchioModel, Configuration
│   ├── control/           # Controller, InverseKinematics
│   ├── tasks/             # Task implementations
│   ├── limits/            # Limit implementations
│   └── gui/               # ImGui interface
├── workspace/
│   ├── models/            # Robot model files (URDF, MJCF)
│   ├── so101/             # SO-101 arm example
│   └── panda/             # Franka Panda example
└── Doxyfile               # Documentation config
```

## Architecture

- **libtorq** (engine): shared library providing physics, kinematics, QP-based IK, and GUI
  - `RobotSystem` — top-level API, owns all subsystems
  - `Controller` — control modes (idle / joint / task-space), IK parameter management
  - `InverseKinematics` — builds and solves the QP via OSQP
  - `Task` hierarchy — `FrameTask`, `PostureTask`, `DampingTask`
  - `Limit` hierarchy — `VelocityLimit`, `ConfigurationLimit`
  - `MujocoDriver` — RAII simulation backend
  - `KinematicsEngine` / `Configuration` — Pinocchio wrapper
  - `Gui` — ImGui docking interface with IK tuning panel

- **workspace/** (apps): robot-specific executables that link against `libtorq`

## Sim vs Real (HAL)

Control code targets `HardwareInterface`; the driver decides where commands go.

- `MujocoDriver` — simulation backend (included)
- Hardware drivers — implement the same interface for real robots

## Documentation

API reference and topic guides are generated with [Doxygen](https://www.doxygen.nl/). To build:

```bash
cd /path/to/torq
doxygen Doxyfile
# Open docs/html/index.html in your browser
```

## Acknowledgements

Torq is a direct port of [Pink](https://github.com/stephane-caron/pink), which uses Pinocchio under the hood. 