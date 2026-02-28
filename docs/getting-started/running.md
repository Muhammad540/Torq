# Running the Example

The SO-101 example launches a MuJoCo simulation of the [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) robot with a real-time ImGui control panel.

```bash
./build/bin/so101
```

## What Happens at Startup

1. `RobotSystem` loads the MuJoCo scene (`workspace/models/SO101/scene.xml`) and connects the physics driver.
2. The URDF (`workspace/models/SO101/so101_new_calib.urdf`) is loaded into Pinocchio. A full 6-DOF model and a reduced 5-DOF model (gripper locked) are built.
3. The GUI window opens with a 3D viewport and control panels.
4. The main loop runs: 10 physics steps per frame, then renders.

## GUI Layout

The GUI is split into three areas:

| Panel | Location | Purpose |
|---|---|---|
| **Viewport** | Center | MuJoCo 3D rendering of the scene |
| **Joint Control** | Left | Per-joint sliders, Home buttons |
| **Cartesian Control** | Right | End-effector jogging, gripper toggle |

### Viewport Controls

- **Rotate** — left-click drag
- **Pan** — right-click drag
- **Zoom** — scroll wheel

The camera is a free-orbit camera centered on the robot. MuJoCo rendering runs in an offscreen framebuffer and is composited into ImGui.

### Joint Control Panel

Each actuator gets a slider ranging from its MuJoCo `ctrlrange` limits. Dragging a slider immediately sends joint-space commands to the controller.

- **Set Home** — saves the current joint positions as the home configuration
- **Move to Home** — commands the robot back to the saved home position via joint-space control

### Cartesian Control Panel

- **Frame name** — the Pinocchio frame to jog (defaults to `gripper_frame_link`)
- **Linear / Angular step** — controls how far each jog button moves (meters / radians)
- **±X, ±Y, ±Z** — translate the end-effector along world axes
- **±Roll, ±Pitch, ±Yaw** — rotate the end-effector about world axes
- **Toggle Gripper** — opens or closes the gripper actuator

Each jog press computes a new target pose and runs the QP-based IK solver to find the corresponding joint motion.

## Multiple Physics Steps Per Frame

The example runs `robot.update()` 10 times per render frame:

```cpp
while (gui.windowIsOpen()) {
    for (int i = 0; i < 10; ++i)
        robot.update();
    gui.render();
}
```

This ensures the physics simulation (which typically uses a small timestep like 2ms) advances enough between GUI frames (which run at display refresh rate). Each `update()` call steps MuJoCo and runs the controller.
