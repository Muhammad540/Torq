# Limits {#limits_page}

Limits define @b inequality constraints @b in the QP. They ensure the solved
\f$\Delta q\f$ respects the physical bounds of the robot. Each limit
produces rows \f$(G, h)\f$ such that \f$G\,\Delta q \le h\f$. See
@ref qp_formulation for how constraints compose and @ref conventions for
notation.

Limits are @b hard constraints — the QP solver will never return a
\f$\Delta q\f$ that violates them. If no feasible solution exists, the
solver reports infeasibility.

Torq provides **two built-in limit types** in every `RobotSystem`
(`VelocityLimit`, `ConfigurationLimit`). You can add **custom** limits by
subclassing `torq::Limit` and passing them to `RobotSystem::addLimit()`.

---

## Limit overview

| Limit | Rows | Purpose | Built-in? |
|-------|------|---------|-----------|
| @ref velocitylimit "VelocityLimit" | \f$2k\f$ | Joint velocity bounds | Yes |
| @ref configurationlimit "ConfigurationLimit" | \f$2k\f$ | Joint position bounds with smooth deceleration | Yes |
| Custom `Limit` subclasses | varies | User-defined inequality constraints | No (`addLimit`) |

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

## How limits stack

All limit types produce rows in the same inequality system:

\f[
G = \begin{bmatrix} G_{\text{vel}} \\ G_{\text{cfg}} \\ G_{\text{user}}
    \end{bmatrix}, \quad
h = \begin{bmatrix} h_{\text{vel}} \\ h_{\text{cfg}} \\ h_{\text{user}}
    \end{bmatrix}
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
