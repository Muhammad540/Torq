# Adding Your Robot

This guide walks through adding a new robot to OpenManip.

## 1. Prepare Model Files

You need two model files:

- **MuJoCo XML** (`scene.xml`) — world + robot for physics simulation
- **URDF** (`robot.urdf`) — robot description for Pinocchio kinematics

Place them under `workspace/models/<your_robot>/`:

```
workspace/models/your_robot/
├── scene.xml          # MuJoCo scene (includes robot XML)
├── your_robot.xml     # MuJoCo robot model
├── your_robot.urdf    # URDF for Pinocchio
└── assets/            # STL/OBJ mesh files (if any)
```

### MuJoCo XML Tips

The `scene.xml` typically includes the robot model, a ground plane, and lighting:

```xml
<mujoco>
  <include file="your_robot.xml"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1" rgba="0.9 0.9 0.9 1"/>
    <light pos="0 0 3" dir="0 0 -1"/>
  </worldbody>
</mujoco>
```

Use position actuators for each joint so that `setJointPositions()` maps directly to `ctrl`:

```xml
<actuator>
  <position name="joint1" joint="joint1" ctrlrange="-3.14 3.14"/>
</actuator>
```

### URDF Tips

!!! tip
    The XML declaration `<?xml version="1.0"?>` must be the **first line** of your URDF — no comments or blank lines before it. Pinocchio's parser requires this.

Make sure the URDF joint names and order match the MuJoCo model. The URDF should include:

- All revolute/prismatic joints with `<limit>` tags (effort, velocity, lower, upper)
- A fixed frame at the end-effector for IK targeting (e.g. `gripper_frame_link`)

### Gripper Handling

If your robot has a gripper joint named `"gripper"` in the URDF, `KinematicsEngine` will automatically lock it when building the reduced model for IK. This means:

- The gripper DOF is excluded from the IK solver
- `toggleGripper()` controls it separately via MuJoCo actuator commands
- You can set which actuator controls the gripper with `setGripperActuator(idx)` (defaults to the last actuator)

## 2. Create a Workspace App

```
workspace/
└── your_robot/
    ├── CMakeLists.txt
    └── src/
        └── main.cpp
```

**CMakeLists.txt:**

```cmake
add_executable(your_robot src/main.cpp)
target_link_libraries(your_robot PRIVATE openmanip)
target_compile_definitions(your_robot PRIVATE PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
```

**main.cpp:**

```cpp
#include "openmanip/RobotSystem.hpp"
#include "openmanip/Gui.hpp"
#include <filesystem>

int main() {
    std::filesystem::path root(PROJECT_ROOT);
    std::string xml  = (root / "workspace/models/your_robot/scene.xml").string();
    std::string urdf = (root / "workspace/models/your_robot/your_robot.urdf").string();

    openmanip::RobotSystem robot;
    if (!robot.initialize(xml, urdf)) return 1;

    openmanip::Gui gui;
    gui.initialize(&robot, "Your Robot");

    while (gui.windowIsOpen()) {
        for (int i = 0; i < 10; ++i)
            robot.update();
        gui.render();
    }
    return 0;
}
```

The `for (int i = 0; i < 10; ++i)` loop runs multiple physics steps per render frame. Adjust this ratio based on your MuJoCo timestep and desired frame rate.

## 3. Register in CMake

Add to the bottom of the root `CMakeLists.txt`:

```cmake
add_subdirectory(workspace/your_robot)
```

## 4. Build and Run

```bash
cd build
cmake ..
cmake --build . -j
./bin/your_robot
```

## 5. Verify

Once running, check the GUI:

- **Joint sliders** should appear for each actuator and move the robot
- **Cartesian jogging** should work when you enter the correct end-effector frame name
- The console should show:

```
[KinematicsEngine] Full model loaded: N config dims, N tangent dims
[KinematicsEngine] Reduced model (gripper locked): M config dims, M tangent dims
[Controller] Initialized
```

## Checklist

- [ ] `scene.xml` loads in MuJoCo without errors
- [ ] URDF starts with `<?xml version="1.0"?>` on line 1
- [ ] Joint names and order match between URDF and MuJoCo XML
- [ ] URDF joints have `<limit>` tags with position and velocity bounds
- [ ] A fixed end-effector frame exists for IK targeting
- [ ] Position actuators are defined in the MuJoCo XML for each joint
- [ ] Gripper joint (if any) is named `"gripper"` in the URDF
- [ ] Workspace CMakeLists.txt is registered in the root CMakeLists.txt
