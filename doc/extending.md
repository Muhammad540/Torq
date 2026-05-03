# Extending Torq {#extending_page}

Tasks, limits and barriers all have similar structure:

1. Subclass the abstract base.
2. Implement a small number of pure virtual methods.
3. Pass a `unique_ptr` to `RobotSystem`.

`RobotSystem` owns the object until `removeTask` / `removeLimit` /
`removeBarrier` is called or the `RobotSystem` is destroyed.

---

## Custom Task

Implement `computeError` and `computeJacobian`; the base class assembles
\f$H, c\f$ for you.

```cpp
class ElbowHeightTask : public torq::Task {
public:
    ElbowHeightTask(std::string frame, double z_target, double cost)
        : Task(cost), frame_(std::move(frame)), z_target_(z_target) {}

    Eigen::VectorXd computeError(const torq::Configuration& c) const override {
        Eigen::VectorXd e(1);
        e(0) = c.getTransformFrameToWorld(frame_).translation().z() - z_target_;
        return e;
    }

    Eigen::MatrixXd computeJacobian(const torq::Configuration& c) const override {
        Eigen::Matrix3d R = c.getTransformFrameToWorld(frame_).rotation();
        Eigen::MatrixXd J = R * c.getFrameJacobian(frame_).topRows<3>();
        return J.row(2);   // z component only
    }

private:
    std::string frame_;
    double z_target_;
};

robot.addTask(std::make_unique<ElbowHeightTask>("elbow", 0.3, 1.0));
```

The error must be **zero at the target**; the system drives \f$e \to 0\f$.

---

## Custom Limit

Return inequality rows \f$G\,\Delta q \le h\f$, or `std::nullopt` when the
limit is inactive.

```cpp
class CapJointZeroVel : public torq::Limit {
public:
    explicit CapJointZeroVel(int joint_idx, double v_max)
        : idx_(joint_idx), v_max_(v_max) {}

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    computeQPInequalities(const torq::Configuration& c, double dt) const override {
        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2, c.nv());
        Eigen::VectorXd h(2);
        G(0, idx_) =  1.0;
        G(1, idx_) = -1.0;
        h.setConstant(dt * v_max_);
        return std::make_pair(G, h);
    }

private:
    int idx_;
    double v_max_;
};
```

---

## Custom Barrier

Implement `computeBarrier` (must satisfy \f$h(q) \ge 0\f$ in the safe set)
and `computeJacobian` (\f$\partial h / \partial q\f$). The base class turns
them into a CBF inequality and an optional safe displacement objective.

```cpp
class UprightBarrier : public torq::Barrier {
public:
    UprightBarrier(std::string frame, double min_cos, double gain)
        : Barrier(1, gain), frame_(std::move(frame)), min_cos_(min_cos) {}

protected:
    void computeBarrier(const torq::Configuration& c,
                        Eigen::Ref<Eigen::VectorXd> out_h) const override {
        out_h(0) = c.getTransformFrameToWorld(frame_).rotation()(2, 2) - min_cos_;
    }

    void computeJacobian(const torq::Configuration& c,
                         Eigen::Ref<Eigen::MatrixXd> out_J) const override {
        Eigen::Matrix3d R = c.getTransformFrameToWorld(frame_).rotation();
        Eigen::MatrixXd J = R * c.getFrameJacobian(frame_).bottomRows<3>();
        out_J = -J.row(2);
    }

private:
    std::string frame_;
    double min_cos_;
};
```

To bias the robot toward a known safe configuration, override
`computeSafeDisplacement` (default zero) and pass a non zero
`safe_displacement_gain`.

To shape the CBF response (e.g. saturate near boundaries) replace the gain
function:

```cpp
barrier->setGainFunction([](double h) { return h / (1.0 + std::abs(h)); });
```

---