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
limit is inactive. Build `G` with `nv` columns matching the reduced model.
`VelocityLimit::computeQPInequalities` requires `dt > 0`.

For joint position boxes on scalar coordinates, use direct margins
\f$q^\text{max} - q\f$ rather than `pinocchio::difference` (see
@ref configurationlimit).

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

Implement `computeBarrier` (must satisfy \f$h(q) \ge 0\f$ in the safe set) and
`computeJacobian` with rows \f$\partial h / \partial q\f$. In `updateQP` the base
class sets \f$G = -\partial h/\partial q\f$ and
\f$h_\text{rhs} = \alpha \odot h(q)\f$ (discrete CBF with QP variable
\f$\Delta q\f$). `updateQP` throws if `dt \le 0`.

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
        out_J.row(0) = J.row(2);   // ∂h/∂q for h = R_zz - min_cos
    }

private:
    std::string frame_;
    double min_cos_;
};
```

To bias motion inside the safe set, override `computeSafeDisplacement`
(default returns zero) and set `safe_displacement_gain` \f$\kappa > 0\f$.

Tune enforcement with `gain` at construction (scalar or `Eigen::VectorXd` per
row). There is no runtime gain function hook.

---
