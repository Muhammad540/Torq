# Simulation to real {#sim_to_real}

The same control loop drives a MuJoCo simulation or a real robot. Only one
field changes: `RobotConfig::driver_type`.

| Driver | `driver_type` | What `connect()` expects |
|--------|---------------|--------------------------|
| @ref torq::MujocoDriver "MujocoDriver" | `"mujoco"`       | Path to the MJCF scene (`scene_path`). |
| @ref torq::ServoDriver "ServoDriver"   | `"serial_servo"` | Path to a calibration file (`robot_calib_file`). |

Both implement @ref torq::HardwareInterface so every other component
(kinematics, IK, GUI, your tasks) works unchanged.

---

## ServoDriver: ST/STS/SMS bus servos

Currently supports the SCServo half duplex protocol used by Waveshare /
Feetech ST/STS/SMS series (e.g. ST3215). The driver opens a POSIX serial
port, runs `SyncRead` / `SyncWrite` cycles, and converts encoder ticks ↔
radians per joint via the calibration file.

```
ServoDriver           // HardwareInterface, radian conversion
└── SMS_STS_Port      // ST/STS/SMS protocol, memory map, sync read/write
    └── SCSPort       // Transport (POSIX serial today)
```

---

## Calibration file (currently used for the SO101 robot)
NOTE: this section is specific to the SO101 robot.
The calibration file uses a small key=value header with one row per joint
in joint order.

```text
port=/dev/ttyACM0
baud_rate=1000000
# joint_index servo_id joint_name direction raw_center raw_min raw_max
0 1 shoulder_pan   1 2048  718 3412
1 2 shoulder_lift  1 2048  848 3263
2 3 elbow_flex     1 2048  852 3065
3 4 wrist_flex     1 2048  854 3184
4 5 wrist_roll     1 2048    0 4095
5 6 gripper        1 2048 1786 3240
```

| Header key | Default | Notes |
|------------|---------|-------|
| `port`           | required | Serial device path |
| `baud_rate`      | 1000000 | |
| `control_period` | 0.002 s | Returned by `getTimestep()` |
| `default_speed`  | 2000   | SyncWrite speed register |

Joint rows are positional: each `raw_center` defines the encoder reading
that corresponds to the URDF zero. For the gripper the calibration uses
`raw_min` / `raw_max` as anchors (asymmetric jaw); other joints anchor
about `raw_center`.

---

## Real robot example

Same code as a simulation example, with three lines changed:

```cpp
torq::RobotConfig cfg;
cfg.scene_path         = "workspace/models/SO101/scene.xml";   // optional, for GUI
cfg.robot_model_path   = "workspace/models/SO101/so101_new_calib.urdf";
cfg.end_effector_frame = "gripper_frame_link";
cfg.locked_joints      = {"gripper"};

cfg.driver_type        = "serial_servo";
cfg.robot_calib_file   = "workspace/so101/calibration/calibration.txt";
cfg.active_control     = false;   // start passive, read only

torq::RobotSystem robot;
robot.initialize(cfg);

torq::Gui gui;
gui.initialize(&robot, "SO-101 (real)");

while (gui.windowIsOpen()) {
    robot.update();
    gui.render();
}
```

If `scene_path` is provided alongside the real driver, Torq loads a display only MuJoCo model. The GUI mirrors the real joint positions into it so you see the robot in 3D. No physics step is taken on the mirror.

---

## Active vs passive control

- @b Passive (`active_control = false`): `update()` reads encoders and
  mirrors them to the GUI but sends no commands. Good for verifying calibration.
- @b Active (`active_control = true`): IK runs and position targets are
  written to the hardware.

---