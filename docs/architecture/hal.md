# Hardware Abstraction Layer (HAL)

The HAL lets you write control and planning code once and run it in simulation or on real hardware by swapping the driver implementation.

## Interface

All drivers implement `HardwareInterface`:

```cpp
class HardwareInterface {
public:
    virtual bool connect(const std::string& config_path) = 0;
    virtual void disconnect() = 0;

    virtual Eigen::VectorXd getJointPositions() const = 0;
    virtual Eigen::VectorXd getJointVelocities() const = 0;

    virtual void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) = 0;
    virtual void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) = 0;

    virtual void step() = 0;
    virtual double getTimestep() const = 0;
    virtual int numJoints() const = 0;
};
```

### Method Summary

| Method | Purpose |
|---|---|
| `connect` | Load a model or open a connection. For MuJoCo, this loads the MJCF file. |
| `disconnect` | Release resources / close the connection. |
| `getJointPositions` | Read current joint positions as an Eigen vector. |
| `getJointVelocities` | Read current joint velocities as an Eigen vector. |
| `setJointPositions` | Write commanded positions (maps to actuator control inputs). |
| `setJointVelocities` | Write commanded velocities (integrated into control inputs). |
| `step` | Advance one timestep (simulation step or hardware read/write cycle). |
| `getTimestep` | Return the control timestep in seconds. |
| `numJoints` | Number of actuated joints. |

## MujocoDriver

`MujocoDriver` is the simulation backend shipped with OpenManip. It wraps a MuJoCo `mjModel` / `mjData` pair with RAII smart pointers.

### Loading

```cpp
MujocoDriver driver;
driver.connect("workspace/models/SO101/scene.xml");
```

`connect()` calls `mj_loadXML()` internally and allocates `mjData`.

### State Access

- `getJointPositions()` returns `Eigen::Map` over `mjData::qpos`
- `getJointVelocities()` returns `Eigen::Map` over `mjData::qvel`

### Command Mapping

- `setJointPositions(q)` writes to `mjData::ctrl` — this works because the SO-101 model uses position actuators (`<general dyntype="none" gaintype="fixed">`).
- `setJointVelocities(qd)` adds `qd * dt` to `ctrl` for velocity-mode commanding.
- `overrideJointPositions(q)` directly sets `qpos` and `ctrl`, zeros `qvel`, and calls `mj_forward()` — useful for teleporting the robot to a configuration.

### Stepping

`step()` calls `mj_step()` which integrates the physics forward by `mjModel::opt.timestep` seconds.

## Available Drivers

| Driver | Backend | Status |
|---|---|---|
| `MujocoDriver` | MuJoCo physics simulation | Shipped |
| `DynamixelDriver` | Real Dynamixel servos (STS3215) | Planned |

## Adding a Custom Driver

To add your own hardware backend:

1. Create a class that inherits from `HardwareInterface`.
2. Implement all pure virtual methods.
3. Swap it into `RobotSystem` (currently `RobotSystem` constructs a `MujocoDriver` in its constructor — you would modify this to accept a driver or use dependency injection).

Example skeleton:

```cpp
#include "openmanip/HardwareInterface.hpp"

class MyDriver : public openmanip::HardwareInterface {
public:
    bool connect(const std::string& config_path) override {
        // Open serial port, load config, etc.
        return true;
    }

    void disconnect() override { /* cleanup */ }

    Eigen::VectorXd getJointPositions() const override {
        // Read from hardware
    }

    Eigen::VectorXd getJointVelocities() const override {
        // Read from hardware
    }

    void setJointPositions(const Eigen::Ref<const Eigen::VectorXd>& q) override {
        // Send to hardware
    }

    void setJointVelocities(const Eigen::Ref<const Eigen::VectorXd>& qd) override {
        // Send to hardware
    }

    void step() override {
        // Read sensors, send commands, etc.
    }

    double getTimestep() const override { return 0.002; }
    int numJoints() const override { return 6; }
};
```
