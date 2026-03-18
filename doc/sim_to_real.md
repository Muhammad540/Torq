# Sim-to-real with Torq {#sim_to_real}

Torq is designed so that the same control code (IK, tasks, limits, barriers) runs in simulation and on real hardware. You switch only the **HardwareInterface** implementation.

## Architecture

- **Simulation:** `MujocoDriver` implements `HardwareInterface`; `connect(config_path)` loads an MJCF scene; `getJointPositions()` / `setJointPositions()` use MuJoCo state and `ctrl`.
- **Real robot:** `ServoDriver` implements `HardwareInterface`; `connect(config_path)` opens the serial port (or parses a config file) and talks to ST/STS/SMS servos (e.g. ST3215) via the SCServo protocol; same get/set API.

`RobotSystem` owns a single `HardwareInterface*`. You choose the driver via **RobotConfig**:

- `driver_type = "mujoco"`: use `MujocoDriver`; `connect(scene_path)`.
- `driver_type = "serial_servo"`: use `ServoDriver`; `connect(driver_connection)` where `driver_connection` is the serial port (e.g. `/dev/ttyUSB0`) or a path to a driver config file (e.g. `workspace/so101/so101.conf`).

The same URDF, limits, barriers, and IK parameters can be used in both cases. Tune limits and barriers **stricter** for the real robot so the commanded motion is safe.

## Real robot: ServoDriver (ST3215 / ST/STS/SMS)

The **ServoDriver** supports ST/STS/SMS series servos (e.g. Waveshare/Feetech ST3215) using the SCServo protocol: 1 Mbps serial, 12-bit position 0–4095, 360° absolute. Used for the SO101 arm with 6× ST3215 servos. The driver uses a bottom-up stack: **SCSPort** (transport, POSIX serial; extensible to ESP32) → **SMS_STS_Port** (protocol, memory map, SyncWrite/Read) → **ServoDriver** (config, radian conversion, HardwareInterface). See `ServoDriver.hpp` and `include/torq/scservo/` for details.

### Connection

- **Option A — port only:** Set `driver_connection` to the serial port (e.g. `/dev/ttyUSB0` on Linux, `COM3` on Windows). ServoDriver uses ST3215 defaults: baud 1000000, control_period 0.002, servo IDs 1–6.
- **Option B — config file:** Use a small key=value config file and set `driver_connection` to its path. File extension must be `.json`, `.yaml`, or `.conf` to be parsed; otherwise the string is treated as the port path. Supported keys (one per line, `key=value`; lines without `=` are ignored):

  | Key              | Type   | Effect                                | Default    |
  |------------------|--------|----------------------------------------|------------|
  | port             | string | Serial device path                     | (required in file) |
  | baud_rate        | int    | Baud rate                              | 1000000    |
  | protocol         | string | Protocol label                         | "st3215"   |
  | control_period   | double | Timestep [s] for step() / getTimestep()| 0.002      |
  | default_speed    | int    | Speed used in SYNC WRITE               | 2000       |
  | servo_ids        | string | Space-separated IDs in joint order     | 1 2 3 4 5 6 |

  Other `SerialServoConfig` fields (position_min, position_max, position_scale, position_zero, default_time) are not read from the file; they stay at ST3215 defaults. Use `ServoDriver::config()` after connect to inspect the resolved config.

Example `so101.conf`:

```text
port=/dev/ttyUSB0
baud_rate=1000000
servo_ids=1 2 3 4 5 6
```

### SO101 real robot example

Use the same URDF and end-effector frame as in sim; only the driver and connection change:

```cpp
torq::RobotConfig config;
config.robot_model_path = "path/to/so101_new_calib.urdf";
config.end_effector_frame = "gripper_frame_link";
config.locked_joints = {"gripper"};

config.driver_type = "serial_servo";
config.driver_connection = "/dev/ttyUSB0";  // or path to so101.conf
config.active_control = false;  // passive: read only, mirror to GUI

torq::RobotSystem robot;
if (!robot.initialize(config)) {
    // handle error (e.g. port not found, servos not responding)
    return 1;
}

while (running) {
    robot.update();
    rate_limiter.sleep();  // e.g. 500 Hz
}
```

### GUI with real robot (display mirror)

When using `driver_type = "serial_servo"`, if you also provide `scene_path` pointing to the MuJoCo scene XML, `RobotSystem` loads a **display-only** MuJoCo model alongside the real hardware driver. Each `update()` call copies the real robot's joint positions into this model via `overrideJointPositions()` (no physics step — purely kinematic). The GUI then renders from this display model, so the 3D viewport shows the real robot's actual pose.

```cpp
torq::RobotConfig config;
config.robot_model_path   = "path/to/so101_new_calib.urdf";
config.scene_path         = "path/to/SO101/scene.xml";   // enables GUI display mirror
config.end_effector_frame = "gripper_frame_link";
config.locked_joints      = {"gripper"};

config.driver_type        = "serial_servo";
config.driver_connection  = "/dev/ttyUSB0";  // or path to so101.conf
config.active_control     = false;

torq::RobotSystem robot;
robot.initialize(config);

torq::Gui gui;
gui.initialize(&robot, "SO101 Real");   // getPhysics() returns the display model

while (gui.windowIsOpen()) {
    robot.update();   // reads real servos, optionally runs IK, mirrors state to display model
    gui.render();
}
```

If `scene_path` is omitted, `getPhysics()` returns null and the GUI 3D view is unavailable; you can still run headless or with a minimal UI.

### Passive mode (display only, move by hand)

You do **not** need to enable active control immediately. Use **passive mode** to move the robot by hand and see its pose in the GUI; when satisfied, enable active control to send commands.

- **At startup:** set `config.active_control = false` (default is false). Set to `true` to command the robot from the first update.
- **At runtime:** call `robot.setActiveControl(false)` to stop sending commands, or `robot.setActiveControl(true)` to enable. The GUI shows an "Active control" checkbox in the Joint Control panel when connected to a real robot; uncheck to enter passive mode, check to enable control.

In passive mode, `update()` still calls `hardware_->step()` (ServoDriver reads positions from the bus), then mirrors `hardware_->getJointPositions()` into the display model. No position or velocity commands are sent because `controller_->update()` is not called.

### Calibration and joint/velocity limits

You do **not** need to calibrate the real robot before using Torq for basic operation. Recommended approach:

- **Joint limits (position):** Use the **same URDF** as in simulation. The Pinocchio model and **ConfigurationLimit** take joint min/max from the URDF, so the IK will not command positions outside that range. The driver converts encoder raw (0–4095) to radians via `position_zero` and `position_scale` in `SerialServoConfig`; these use ST3215 defaults (centre 2047.5, scale 2π/4096). They are not currently read from the .conf file; for a custom mapping you would set them in code or extend the config parser.
- **Velocity limits:** Set in the **VelocityLimit** (or in the model) to safe values for the ST3215 (see the servo datasheet for max speed). This keeps the IK from commanding unrealistic velocities. No separate "calibration" step is required; use conservative limits at first and tune if needed.

So: start with the same URDF and limits as in sim; optionally tighten velocity from the ST3215 manual; adjust position_zero/position_scale only if you notice a constant offset (via code or a future config key).

### ST/STS/SMS protocol (SCSPort + SMS_STS_Port)

ServoDriver uses the in-tree **SCSPort** and **SMS_STS_Port** layers (see `include/torq/scservo/SCSPort.hpp`, `SMS_STS_Port.hpp`). SCSPort implements the SCS virtual read/write interface over a POSIX serial fd (Linux/macOS); the same interface can be implemented on ESP32 with an UART driver. SMS_STS_Port provides the ST/STS/SMS memory map and instructions: goal position, present position, torque enable, SyncWritePosEx, ReadPos, Ping, etc. Packet layout matches the SCServo/Waveshare manual: header `0xFF 0xFF`, instructions READ, WRITE, SYNC WRITE, SYNC READ; register addresses (e.g. goal at ACC 41, present position at 56–57) as in `SMS_STS_Port.hpp`. No Arduino dependency; the stack runs on the host and can be extended to embedded by swapping the transport in SCSPort.

## Limits and barriers for real hardware

To avoid dangerous velocities and torques on the real robot:

1. **VelocityLimit** — Ensure the model or the limit configuration uses safe max velocities (rad/s) for each joint. The ST3215 has limited speed; keep commanded velocity within spec.
2. **ConfigurationLimit** — Use the same joint limits as in the URDF; the IK will not command outside `[q_min, q_max]`. Tune `config_limit_gain` (e.g. 0.3–0.5) so the robot slows down before hitting limits.
3. **AccelerationLimit** (optional) — Add via `RobotSystem::addLimit()` with a vector of max accelerations per joint. Call `setLastIntegration(v_prev, dt)` each tick (the Controller does this when an AccelerationLimit is present). This smooths motion and avoids torque spikes.
4. **Barriers** — Use **PositionBarrier** (e.g. workspace bounds, CoM height) or **BodySphericalBarrier** (minimum distance between links) to keep the robot away from unsafe regions. Add via `RobotSystem::addBarrier()`.

The servos (e.g. ST3215) usually have internal position (and sometimes velocity) control. Torq sends **position setpoints** at the control rate; the servo's internal controller tracks them. So the "output" from Torq is desired joint positions; the HAL turns that into serial commands. Limiting the IK output (velocity, acceleration, configuration) keeps those setpoints safe.

## Checklist for real robot

- [ ] Same URDF (and calibration) as in sim.
- [ ] Correct serial port and baud rate; servo IDs in joint order (e.g. via `driver_connection` or .conf).
- [ ] Velocity limits set (model or limit config) to safe values.
- [ ] Configuration limits and optional acceleration limits enabled.
- [ ] Optional barriers for workspace or self-collision.
- [ ] Control loop running at a fixed rate (e.g. 500 Hz); use the same `control_frequency_hz` as in sim if possible.
- [ ] Emergency stop or disable path independent of Torq (hardware or watchdog).
