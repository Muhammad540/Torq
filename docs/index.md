# OpenManip

High-performance C++ framework for robot manipulation that combines **MuJoCo** (physics + contacts) and **Pinocchio** (rigid-body dynamics + kinematics) behind a concise, easy-to-use API.

Designed for motion planning and control (FK/IK) with a **Hardware Abstraction Layer (HAL)** to seamlessly switch between simulation and real hardware.

## Features

- **Physics simulation** via MuJoCo with contact dynamics
- **Kinematics** (FK/IK) via Pinocchio
- **QP-based IK solver** with task objectives and joint limit constraints (OSQP)
- **Real-time GUI** with joint sliders, Cartesian jogging, and gripper control (ImGui)
- **HAL** to swap sim and real hardware drivers with zero code changes
- **SO-101** robot arm example included out of the box

## Quick Start

```bash
# Install dependencies (macOS)
brew install cmake eigen glew glfw osqp pinocchio pkg-config

# Build OsqpEigen (not in brew)
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. && cmake --build . -j
cmake --install . --prefix /opt/homebrew

# Build OpenManip
git clone <this-repo> && cd OpenManip
mkdir -p build && cd build
cmake .. && cmake --build . -j

# Run the SO-101 example
./bin/so101
```

## Project Layout

```
OpenManip/
├── include/openmanip/     # Public API headers
├── src/
│   ├── core/              # RobotSystem orchestrator
│   ├── physics/           # MujocoDriver (HAL impl)
│   ├── kinematics/        # PinocchioModel, Configuration
│   ├── control/           # Controller, IK solver
│   ├── tasks/             # Task definitions (FrameTask, PostureTask, …)
│   ├── limits/            # QP constraints (velocity, configuration)
│   └── gui/               # ImGui + MuJoCo visualizer
└── workspace/
    ├── models/SO101/      # Robot URDF + MuJoCo XML + meshes
    └── so101/             # SO-101 example application
```

## How It Works

A typical application is under 20 lines of code:

```cpp
#include "openmanip/RobotSystem.hpp"
#include "openmanip/Gui.hpp"

int main() {
    openmanip::RobotSystem robot;
    robot.initialize("scene.xml", "robot.urdf");

    openmanip::Gui gui;
    gui.initialize(&robot, "My Robot");

    while (gui.windowIsOpen()) {
        robot.update();   // step physics + run controller
        gui.render();     // draw viewport + control panels
    }
}
```

`RobotSystem` owns the physics driver, kinematics engine, and controller. Call `update()` each tick and everything stays in sync — IK solving, joint commands, and simulation stepping all happen under the hood.
