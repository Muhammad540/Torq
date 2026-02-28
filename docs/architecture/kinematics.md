# Kinematics Engine

Kinematics is handled by [Pinocchio](https://github.com/stack-of-tasks/pinocchio), a fast rigid-body dynamics library. OpenManip wraps it in two classes: `KinematicsEngine` (model loading) and `Configuration` (per-query state).

## KinematicsEngine

`KinematicsEngine` loads a URDF and builds two Pinocchio models:

- **Full model** — all joints including the gripper (6 DOF for SO-101)
- **Reduced model** — gripper joint locked via `pinocchio::buildReducedModel` (5 DOF for SO-101)

The reduced model is used for IK so the solver only plans the arm joints.

```
[KinematicsEngine] Full model loaded: 6 config dims, 6 tangent dims
[KinematicsEngine] Reduced model (gripper locked): 5 config dims, 5 tangent dims
```

### API

```cpp
KinematicsEngine kin;
kin.initialize("/path/to/robot.urdf");

const pinocchio::Model& model  = kin.model();       // reduced model (for IK)
const pinocchio::Model& full   = kin.fullModel();    // full model (for FK)

Configuration config = kin.makeConfiguration(q);     // build a Configuration from reduced q

Eigen::VectorXd q_reduced = kin.fullToReducedQ(q_full);  // strip gripper DOF
kin.printFrames();   // dump all frame names to console
```

### Gripper Detection

During `initialize()`, if a joint named `"gripper"` exists in the URDF, it is automatically locked in the reduced model. If no such joint exists, the full and reduced models are identical.

## Configuration

A `Configuration` is a snapshot of the robot state at a particular `q`. It wraps the Pinocchio model and data, runs forward kinematics on construction, and provides queries.

### Creating a Configuration

```cpp
Configuration config = kinematics.makeConfiguration(q);
```

Forward kinematics is computed immediately so all frame poses and Jacobians are available.

### Forward Kinematics

```cpp
pinocchio::SE3 T = config.getTransformFrameToWorld("gripper_frame_link");
// T.translation() → Eigen::Vector3d position
// T.rotation()    → Eigen::Matrix3d orientation
```

Returns the SE3 pose of any named frame in the URDF.

You can also get the relative transform between two frames:

```cpp
pinocchio::SE3 T_ab = config.getTransform("frame_a", "frame_b");
```

### Jacobians

```cpp
Eigen::MatrixXd J = config.getFrameJacobian("gripper_frame_link");
```

Returns the 6×nv frame Jacobian in the local frame. This is used by the IK solver to map joint velocities to end-effector twist.

### Integration

After IK produces a velocity `dq`, integrate it into a new configuration:

```cpp
Eigen::VectorXd q_new = config.integrate(velocity, dt);
// or in-place:
config.integrateInplace(velocity, dt);
```

Uses `pinocchio::integrate()` which properly handles joint types on the configuration manifold.

### Limit Checking

```cpp
config.checklimits(/*tolerance=*/1e-3, /*safety_break=*/false);
if (config.geterror() != ErrorCode::None) {
    // joint out of bounds
    config.clearError();
}
```

## Frame Names

Frames are defined in the URDF as links and fixed frames. For the SO-101:

| Frame | Description |
|---|---|
| `base_link` | Fixed base |
| `shoulder_link` | After shoulder pan joint |
| `upper_arm_link` | After shoulder lift joint |
| `forearm_link` | After elbow flex joint |
| `wrist_link` | After wrist flex joint |
| `gripper_link` | After wrist roll joint |
| `gripper_frame_link` | Fixed frame at the gripper tip (end-effector) |

Use `kin.printFrames()` to list all available frames for your robot.
