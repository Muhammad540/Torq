# Extending Torq {#extending_page}

This page explains how to add custom tasks, limits, and barriers to Torq.
The library is designed so that all three follow the same pattern:
subclass an abstract base, implement a few pure virtual methods, and
register the object with `RobotSystem`.

---

## Adding a custom Task

### Step 1: Subclass `torq::Task`

Implement two pure virtual methods:

| Method | Returns | Purpose |
|--------|---------|---------|
| `computeError(config)` | \f$e \in \mathbb{R}^k\f$ | Error vector — zero at the target. |
| `computeJacobian(config)` | \f$J \in \mathbb{R}^{k \times n_v}\f$ | Maps \f$\Delta q\f$ to task-space changes. |

The base class `computeQPObjective()` automatically converts these into
the QP contribution \f$(H, c)\f$ using the @ref task_qp_generic "generic formulation".

### Step 2: Choose constructor parameters

Pass the cost (scalar or vector), gain, and LM damping to the `Task`
base constructor:

```cpp
class MyCustomTask : public torq::Task {
public:
    MyCustomTask(double cost, double gain = 1.0, double lm_damping = 0.0)
        : Task(cost, gain, lm_damping) {}

    Eigen::VectorXd computeError(const torq::Configuration& config) const override {
        // Return k-dimensional error vector
        Eigen::VectorXd e(3);
        // ... compute error based on config ...
        return e;
    }

    Eigen::MatrixXd computeJacobian(const torq::Configuration& config) const override {
        // Return k x nv Jacobian matrix
        Eigen::MatrixXd J(3, config.nv());
        // ... compute Jacobian based on config ...
        return J;
    }
};
```

> @b Key rule: The error must be zero when the task is satisfied. The
> system drives \f$e \to 0\f$ via \f$J\,\Delta q = -\alpha\,e\f$.

### Step 3: Register with RobotSystem

```cpp
auto task = std::make_unique<MyCustomTask>(1.0);
// Configure the task (set targets, etc.)
robot.addTask(std::move(task));
```

The task is now included in every IK solve. `RobotSystem` owns the task
and will destroy it on `removeTask()` or when `RobotSystem` is destroyed.

### Step 4: Update targets in the control loop (if needed)

If your task has a dynamic target, update it before each `robot.update()`:

```cpp
while (running) {
    my_task_ptr->setTarget(new_target);  // keep a raw pointer for updates
    robot.update();
}
```

### Example: Elbow height task

A task that keeps the elbow frame above a minimum height:

```cpp
class ElbowHeightTask : public torq::Task {
public:
    ElbowHeightTask(const std::string& elbow_frame, double target_z, double cost)
        : Task(cost), frame_(elbow_frame), target_z_(target_z) {}

    Eigen::VectorXd computeError(const torq::Configuration& config) const override {
        double current_z = config.getTransformFrameToWorld(frame_).translation()(2);
        Eigen::VectorXd e(1);
        e(0) = current_z - target_z_;
        return e;
    }

    Eigen::MatrixXd computeJacobian(const torq::Configuration& config) const override {
        Eigen::MatrixXd J_local = config.getFrameJacobian(frame_).topRows<3>();
        Eigen::Matrix3d R = config.getTransformFrameToWorld(frame_).rotation();
        Eigen::MatrixXd J_world = R * J_local;
        return J_world.row(2);  // Z component only
    }

private:
    std::string frame_;
    double target_z_;
};
```

---

## Adding a custom Limit

### Step 1: Subclass `torq::Limit`

Implement one pure virtual method:

| Method | Returns | Purpose |
|--------|---------|---------|
| `computeQPInequalities(config, dt)` | `optional<pair<G, h>>` | Constraint rows \f$G\,\Delta q \le h\f$. |

Return `std::nullopt` if there are no active constraints.

### Step 2: Implement the constraint

```cpp
class MyCustomLimit : public torq::Limit {
public:
    MyCustomLimit(/* parameters */) { /* ... */ }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    computeQPInequalities(const torq::Configuration& config, double dt) const override {
        int nv = config.nv();

        // Build G (m x nv) and h (m x 1) where m is the number of constraint rows
        Eigen::MatrixXd G(2, nv);
        Eigen::VectorXd h(2);

        // Example: constrain joint 0 displacement to [-0.1, 0.1]
        G.setZero();
        G(0, 0) = 1.0;   // +dq_0 <= 0.1
        G(1, 0) = -1.0;  // -dq_0 <= 0.1
        h(0) = 0.1;
        h(1) = 0.1;

        return std::make_pair(G, h);
    }
};
```

### Step 3: Register

```cpp
auto limit = std::make_unique<MyCustomLimit>(/* params */);
robot.addLimit(std::move(limit));
```

### Guidelines

- Each row of \f$G\f$ is a linear constraint on \f$\Delta q\f$.
- Return `std::nullopt` when the limit is inactive to avoid unnecessary
  rows in the QP.
- Remember that \f$\Delta q = v \cdot \Delta t\f$, so if your constraint
  is on velocity, multiply the bound by \f$\Delta t\f$.

---

## Adding a custom Barrier

### Step 1: Subclass `torq::Barrier`

Implement two pure virtual methods:

| Method | Returns | Purpose |
|--------|---------|---------|
| `computeBarrier(config)` | \f$h(q) \in \mathbb{R}^d\f$ | Barrier function value. \f$\ge 0\f$ in the safe region. |
| `computeJacobian(config)` | \f$\partial h / \partial q \in \mathbb{R}^{d \times n_v}\f$ | Barrier Jacobian. |

The base class automatically converts these into:
- QP inequality rows via `computeQPInequalities()`.
- QP objective terms via `computeQPObjective()` (if `safe_displacement_gain > 0`).

### Step 2: Choose constructor parameters

```cpp
class MyCustomBarrier : public torq::Barrier {
public:
    MyCustomBarrier(double gain, double safe_displacement_gain)
        : Barrier(1 /* dimension */, gain, safe_displacement_gain) {}

    Eigen::VectorXd computeBarrier(const torq::Configuration& config) const override {
        Eigen::VectorXd h(1);
        // h(q) >= 0 in the safe region
        // Example: keep some quantity positive
        h(0) = /* compute barrier value */;
        return h;
    }

    Eigen::MatrixXd computeJacobian(const torq::Configuration& config) const override {
        Eigen::MatrixXd J(1, config.nv());
        // Jacobian of the barrier function
        J = /* compute dh/dq */;
        return J;
    }
};
```

### Optional: Override `computeSafeDisplacement()`

The default safe displacement is zero. Override to provide a displacement
that moves the robot toward a known-safe configuration:

```cpp
Eigen::VectorXd computeSafeDisplacement(
    const torq::Configuration& config) const override {
    // Return a displacement that pushes the robot to safety
    return /* safe delta_q */;
}
```

### Optional: Set a custom gain function

Replace the default linear gain \f$g(x) = x\f$ with a custom function:

```cpp
barrier->setGainFunction([](double h) {
    return h / (1.0 + std::abs(h));  // Saturating gain
});
```

### Step 3: Register

```cpp
auto barrier = std::make_unique<MyCustomBarrier>(1.0, 0.5);
robot.addBarrier(std::move(barrier));
```

### Example: Orientation barrier

Keep the end-effector's Z-axis aligned with the world Z-axis (prevent
flipping):

```cpp
class UprightBarrier : public torq::Barrier {
public:
    UprightBarrier(const std::string& frame, double gain, double min_cos = 0.5)
        : Barrier(1, gain, 0.0), frame_(frame), min_cos_(min_cos) {}

    Eigen::VectorXd computeBarrier(const torq::Configuration& config) const override {
        Eigen::Matrix3d R = config.getTransformFrameToWorld(frame_).rotation();
        double cos_angle = R(2, 2);  // Z-axis dot product with world Z
        Eigen::VectorXd h(1);
        h(0) = cos_angle - min_cos_;
        return h;
    }

    Eigen::MatrixXd computeJacobian(const torq::Configuration& config) const override {
        // Approximate: use angular Jacobian's Z-row
        Eigen::MatrixXd J_local = config.getFrameJacobian(frame_).bottomRows<3>();
        Eigen::Matrix3d R = config.getTransformFrameToWorld(frame_).rotation();
        Eigen::MatrixXd J_world = R * J_local;
        // dcos/dq ≈ -sin(theta) * angular_velocity_z / dq
        // Simplified: use the Z-row of the angular Jacobian
        return -J_world.row(2);  // Negative because barrier increases with alignment
    }

private:
    std::string frame_;
    double min_cos_;
};
```

---

## Registration and ownership

| Method | Accepts | Ownership |
|--------|---------|-----------|
| `RobotSystem::addTask(unique_ptr<Task>)` | Any `Task` subclass | RobotSystem owns |
| `RobotSystem::addLimit(unique_ptr<Limit>)` | Any `Limit` subclass | RobotSystem owns |
| `RobotSystem::addBarrier(unique_ptr<Barrier>)` | Any `Barrier` subclass | RobotSystem owns |

Objects are passed as `std::unique_ptr` and destroyed when removed or when
`RobotSystem` is destroyed. The `Controller` receives raw pointers each
tick but never stores them across ticks — no use-after-free risk.

---

## Testing your extension

1. **Dimension check**: Ensure `computeError()` returns the expected
   dimension and `computeJacobian()` returns `(k, nv)`.
2. **Gradient check**: Verify the Jacobian numerically by perturbing \f$q\f$
   and checking that \f$J\,\delta q \approx e(q + \delta q) - e(q)\f$.
3. **Inert behaviour**: If your task/barrier has an optional target, verify
   it returns zero contribution when inactive.
4. **Build and run**: Add the task/limit/barrier, run the control loop, and
   verify correct behaviour in the GUI.
