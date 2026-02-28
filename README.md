# OpenManip

High performance C++ framework for robot manipulation that combines **MuJoCo** (physics + contacts) and **Pinocchio** (rigid body dynamics + kinematics) behind a concise and easy to use API. Designed for motion planning and control (FK/IK/RRT) with a **HAL** to switch simulation vs real hardware.

## Dependencies

- Linux (Ubuntu recommended)
- CMake
- Pinocchio (system install required)
- Eigen3 (via Pinocchio on most setups)
- MuJoCo (fetched by CMake)
- osqp (system install required)
- osqpEigen (system install required)

### Install Pinocchio (Ubuntu)

Follow the official Pinocchio installation instructions for your distro/toolchain:
- https://stack-of-tasks.github.io/pinocchio/

## Build

```bash
git clone <this-repo>
cd OpenManip
mkdir -p build && cd build
cmake ..
cmake --build . -j
```

### Useful CMake Options

| Option | Description | Default |
|------|-------------|---------|
| `ENABLE_TRACKING_POINTS` | Visualize user defined tracking points (pos/size/color) | `OFF` |

Example:

```bash
cmake -DENABLE_TRACKING_POINTS=ON ..
cmake --build . -j
```

## Run (SO-101 example)

```bash
cd build/bin
./so101_robot
```

## Project Layout

```text
OpenManip/
├── include/                 # public API headers
├── src/                     
│   ├── core/                
│   ├── physics/             
│   ├── kinematics/          
│   └── control/
│
└── workspace/
    ├── models/               
    ├── so101/               
    └── <your_robot>/        # add your robot here
```

## Architecture

- **libopenmanip** (engine): reusable core library
  - `RobotSystem`: loads config, owns physics + kinematics + control backends
  - `MujocoDriver`: RAII wrapper around MuJoCo model/data, stepping, state IO
  - `Visualizer`: passive viewer (render + input)
  - `Pinocchio`: Handles FK/IK
  - `Controller`
  
- **workspace/** (apps): robot specific code that links against the library
  - example: `workspace/so101/src/main.cpp`

## Sim vs Real (HAL)

Planning/control code targets a driver interface; the driver decides where commands go.

- `MujocoDriver`: simulation backend
- `DynamixelDriver`: hardware backend

## Documentation

Docs are built with [MkDocs](https://www.mkdocs.org/) and live in the `docs/` folder. To serve them locally:

```bash
uv sync
uv run mkdocs serve
```

Then open [http://127.0.0.1:8000](http://127.0.0.1:8000).