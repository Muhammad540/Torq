# Limits {#limits_page}

Limits define @b inequality constraints @b in the QP. They ensure the solved
\f$\Delta q\f$ respects the physical bounds of the robot. Each limit
produces rows \f$(G, h)\f$ such that \f$G\,\Delta q \le h\f$. See
@ref qp_formulation for how constraints compose and @ref conventions for
notation.

Limits are @b hard constraints — the QP solver will never return a
\f$\Delta q\f$ that violates them. If no feasible solution exists, the
solver reports infeasibility.

Torq provides **4 limit types**. Two are built into every `RobotSystem`
(VelocityLimit, ConfigurationLimit); the others are added via
`RobotSystem::addLimit()`.

---

## Limit overview

| Limit | Rows | Purpose | Built-in? |
|-------|------|---------|-----------|
| @ref velocitylimit "VelocityLimit" | \f$2k\f$ | Joint velocity bounds | Yes |
| @ref configurationlimit "ConfigurationLimit" | \f$2k\f$ | Joint position bounds with smooth deceleration | Yes |
| @ref floatingbasevelocitylimit "FloatingBaseVelocityLimit" | \f$2 \times 6\f$ | Base twist limits for mobile/legged robots | No |
| @ref accelerationlimit "AccelerationLimit" | \f$2k\f$ | Joint acceleration bounds with braking distance | No |

---

## Common interface

All limits inherit from `torq::Limit` and implement a single method:

```cpp
virtual std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>
    computeQPInequalities(const Configuration& config, double dt) const = 0;
```

Returns `std::nullopt` if there are no active constraints (e.g. no finite
bounds in the model), in which case the limit contributes zero rows.

---

## VelocityLimit {#velocitylimit}

@b Purpose: Bound joint displacements so that the implied velocity does not
exceed hardware limits.

### Formulation

Given the maximum velocity vector \f$\dot{q}_{\max} \in \mathbb{R}^{n_v}\f$
(from `model.velocityLimit`) and the integration timestep \f$\Delta t\f$:

\f[
-\Delta t\;\dot{q}_{\max} \;\le\; \Delta q \;\le\; \Delta t\;\dot{q}_{\max}
\f]

Written in standard inequality form \f$G\,\Delta q \le h\f$ this becomes
\f$2k\f$ rows (upper + lower bound for each bounded DOF):

\f[
\begin{bmatrix} +S \\ -S \end{bmatrix} \Delta q
\;\le\;
\begin{bmatrix} \Delta t\;\dot{q}_{\max}^{(k)} \\
                \Delta t\;\dot{q}_{\max}^{(k)} \end{bmatrix}
\f]

where \f$S \in \mathbb{R}^{k \times n_v}\f$ is a selection matrix that picks
out the \f$k\f$ joints with finite velocity bounds. Joints without a finite
`velocityLimit` in the URDF/MJCF are excluded.

### How it works in code

1. On construction, `VelocityLimit` scans the Pinocchio model for joints
   with finite velocity bounds and builds a projection matrix \f$S\f$.
2. Each tick, `computeQPInequalities()` scales the bounds by \f$\Delta t\f$
   and returns the stacked \f$(G, h)\f$.

### Parameters

None — limits are read directly from the Pinocchio model. To change
velocity limits, modify the URDF or the model before constructing the limit.

### Key property

Velocity limits scale correctly with the timestep \f$\Delta t\f$. A larger
timestep allows larger per-step displacements. The actual velocity bound
\f$\dot{q}_{\max}\f$ is independent of the control frequency.

---

## ConfigurationLimit {#configurationlimit}

@b Purpose: Keep \f$q + \Delta q\f$ within the joint position bounds
\f$[q_{\min}, q_{\max}]\f$, using a proportional gain to steer away from
limits before they are hit.

### Formulation

For each bounded joint \f$j\f$ with current position \f$q_j\f$ and limits
\f$q_j^{\min}, q_j^{\max}\f$:

\f[
\Delta q_j \;\le\; \gamma\,(q_j^{\max} \ominus q_j)
\f]
\f[
-\Delta q_j \;\le\; \gamma\,(q_j \ominus q_j^{\min})
\f]

where \f$\gamma \in (0, 1]\f$ is the @b configuration limit gain
(`config_limit_gain`) and \f$\ominus\f$ is the manifold-aware difference.
For revolute joints, \f$q_j^{\max} \ominus q_j = q_j^{\max} - q_j\f$.

In matrix form for all \f$k\f$ bounded joints:

\f[
\begin{bmatrix} +S \\ -S \end{bmatrix} \Delta q
\;\le\;
\begin{bmatrix}
  \gamma\,(q^{\max}_{(k)} \ominus q_{(k)}) \\
  \gamma\,(q_{(k)} \ominus q^{\min}_{(k)})
\end{bmatrix}
\f]

### How the gain works

The gain \f$\gamma\f$ controls how much of the remaining range the solver
is allowed to use in a single tick:

| \f$\gamma\f$ | Behaviour |
|---------------|-----------|
| 1.0 | Allow the full remaining range — only prevents actual violation. |
| 0.5 (default) | Permit half the remaining range per tick — smooth deceleration near limits. |
| 0.1 | Very conservative — robot slows down far from limits. |

Lower gain → earlier and smoother deceleration as the robot approaches a
limit. This prevents hard stops and reduces stress on hardware.

### How it works in code

1. On construction, scans for joints with finite position bounds.
2. Each tick, computes `pinocchio::difference(model, q, q_max)` and
   `pinocchio::difference(model, q, q_min)` to get manifold-aware slack.
3. Scales the slack by \f$\gamma\f$ and returns the stacked constraints.

### Parameters

| Parameter | Symbol | Default | Setter |
|-----------|--------|---------|--------|
| `config_limit_gain` | \f$\gamma\f$ | 0.5 | `setConfigLimitGain(double)` |

---

## FloatingBaseVelocityLimit {#floatingbasevelocitylimit}

@b Purpose: Cap the linear and angular velocity of the floating base for
mobile or legged robots.

### Formulation

Given maximum twist \f$v_{\max} \in \mathbb{R}^6\f$ (linear + angular):

\f[
-\Delta t\,v_{\max} \;\le\; J_{\text{root}}\,\Delta q \;\le\; \Delta t\,v_{\max}
\f]

In standard form:

\f[
\begin{bmatrix} +J_{\text{root}} \\ -J_{\text{root}} \end{bmatrix} \Delta q
\;\le\;
\begin{bmatrix} \Delta t\,v_{\max} \\ \Delta t\,v_{\max} \end{bmatrix}
\f]

The Jacobian \f$J_{\text{root}}\f$ is the frame Jacobian of the base with
all columns outside the root joint zeroed. This ensures only the
floating-base DOFs are constrained.

### Requirements

- The model must have a `root_joint` (floating base).
- A base frame name can be provided, or it is auto-detected.

### How it works in code

1. On construction, identifies the root joint and its velocity indices.
2. Each tick, computes the frame Jacobian, zeros non-root columns, and
   builds the box constraint.
3. Only rows with finite bounds are included.

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `base_frame` | string | Base frame name (empty = auto-detect). |
| `max_linear_velocity` | Vector3d or double | Linear velocity bound [m/s]. |
| `max_angular_velocity` | Vector3d or double | Angular velocity bound [rad/s]. |

### Example

```cpp
auto fb_limit = std::make_unique<torq::FloatingBaseVelocityLimit>(
    robot.model(),
    "",     // auto-detect base frame
    0.5,    // max linear velocity [m/s]
    1.0     // max angular velocity [rad/s]
);
robot.addLimit(std::move(fb_limit));
```

---

## AccelerationLimit {#accelerationlimit}

@b Purpose: Limit joint accelerations to smooth motion and prevent torque
spikes. Combines a finite-difference acceleration bound with a
braking-distance constraint.

### Formulation

Two constraint types are enforced per joint, and the tighter one wins:

**1. Finite-difference acceleration bound:**

\f[
\Delta q_{\text{prev}} - a_{\max}\,\Delta t^2 \;\le\; \Delta q
\;\le\; \Delta q_{\text{prev}} + a_{\max}\,\Delta t^2
\f]

This limits how much the displacement can change from the previous tick,
bounding the finite-difference acceleration.

**2. Braking-distance bound** (Flacco 2015, Del Prete 2018):

\f[
|\dot{q}| \;\le\; \sqrt{2\,a_{\max}\,d_{\text{limit}}}
\f]

where \f$d_{\text{limit}}\f$ is the distance to the nearest joint position
limit. This ensures the robot can always decelerate to a stop before
hitting the limit, given the maximum deceleration \f$a_{\max}\f$.

In displacement terms:

\f[
\Delta q_{\max}^{\text{brake}} = \Delta t \cdot \sqrt{2\,a_{\max}\,\max(d_{\text{limit}}, 0)}
\f]

The effective bound is:

\f[
h = \min\left(a_{\max}\,\Delta t^2 + \Delta q_{\text{prev}},\;
    \Delta t\,\sqrt{2\,a_{\max}\,d_{\text{limit}}}\right)
\f]

### Requirements

`setLastIntegration(v_prev, dt)` must be called each tick with the
velocity from the previous IK solve. When added via
`RobotSystem::addLimit()`, the controller handles this automatically.

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `acceleration_limit` | VectorXd | Per-joint max acceleration (dimension \f$n_v\f$). |

### Example

```cpp
Eigen::VectorXd a_max = Eigen::VectorXd::Constant(robot.nv(), 10.0);
auto accel_limit = std::make_unique<torq::AccelerationLimit>(
    robot.model(), a_max);
robot.addLimit(std::move(accel_limit));
```

---

## How limits stack

All limit types produce rows in the same inequality system:

\f[
G = \begin{bmatrix} G_{\text{vel}} \\ G_{\text{cfg}} \\ G_{\text{fb}} \\
    G_{\text{accel}} \end{bmatrix}, \quad
h = \begin{bmatrix} h_{\text{vel}} \\ h_{\text{cfg}} \\ h_{\text{fb}} \\
    h_{\text{accel}} \end{bmatrix}
\f]

A limit may return `std::nullopt` if it has no active constraints, in
which case it contributes zero rows. Barrier constraint rows are also
stacked into the same system (see @ref barriers_page).

The OSQP solver handles the full stacked system simultaneously.

---

## Planned: equality constraints

Some tasks may in the future act as hard equality constraints rather than
soft objectives:

\f[
A_{\text{eq}}\,\Delta q = b_{\text{eq}}
\f]

The `QPProblem` struct is designed to be extended with equality rows.
