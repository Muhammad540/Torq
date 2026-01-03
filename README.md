# OpenManip: 
**A C++ Framework for Robot Manipulation & Planning**

OpenManip uses the physics simulation **MuJoCo** and rigid body dynamics algorithms from **Pinocchio**. It provides an environment for developing motion planning algorithms (RRT, IK) for robot manipulators.

## Installation

This project targets **Linux (Ubuntu)**. While MuJoCo and YAML-CPP are downloaded automatically by CMake, **Pinocchio** must be installed on your system.

## Setup

```bash
git clone <this-repo>
cd OpenManip
mkdir build && cd build
cmake ..
make -j4
```

## Execute

```bash
cd bin
./so101_robot
```

## Structure

```text
OpenManip/
├── models/        
├── src/           
│   ├── core/      
│   ├── physics/   
│   ├── kinematics/
│   └── planning/  
├── include/       
└── workspace/
```