# Barriers {#barriers_page}

Barriers enforce @b safety constraints @b using Control Barrier Functions
(CBFs). A barrier function \f$h(q) \ge 0\f$ defines the safe region of
configuration space. The CBF condition guarantees that the robot stays
within this safe region at all times.

Barriers contribute to the QP in **two ways**:

1. @b Inequality rows @b \f$(G, h)\f$ — hard constraints that prevent the
   robot from leaving the safe set.
2. @b Objective terms @b \f$(H, c)\f$ — optional safe-displacement
   regularisation that actively pushes the robot away from the boundary.

See @ref qp_formulation for how barriers compose with tasks and limits,
and @ref conventions for notation.

Torq provides **3 barrier types**, all added via
`RobotSystem::addBarrier()`.

---

## Barrier overview

| Barrier | Dimension | Purpose |
|---------|-----------|---------|
| @ref positionbarrier "PositionBarrier" | \f$\le 6\f$ | Workspace bounds on a frame's Cartesian position |
| @ref bodysphericalbarrier "BodySphericalBarrier" | 1 | Minimum distance between two frames |
| @ref selfcollisionbarrier "SelfCollisionBarrier" | \f$N\f$ | Self-collision avoidance via geometry model |

---

## CBF theory {#cbf_theory}

### The CBF condition

For a barrier function \f$h : \mathcal{Q} \to \mathbb{R}\f$ where
\f$h(q) \ge 0\f$ defines the safe region, the CBF condition is:

\f[
\frac{\partial h}{\partial q}\,\dot{q} + \alpha\,h(q) \ge 0
\f]

This guarantees forward invariance of the safe set: if the robot starts
in \f$h(q) \ge 0\f$, it stays there. The gain \f$\alpha > 0\f$ controls
how aggressively the barrier is enforced — higher values allow the robot
to approach the boundary more closely before being pushed back.

### QP inequality form

In displacement form (\f$\Delta q = \dot{q} \cdot \Delta t\f$), the CBF
condition becomes:

\f[
\frac{\partial h}{\partial q}\,\frac{\Delta q}{\Delta t} + \alpha\,h(q) \ge 0
\f]

Rearranging into the standard \f$G\,\Delta q \le h\f$ form:

\f[
-\frac{1}{\Delta t}\frac{\partial h}{\partial q}\,\Delta q \;\le\;
\alpha \cdot g\!\left(h(q)\right)
\f]

where \f$g(\cdot)\f$ is a @b gain function @b applied element-wise. By
default, \f$g(x) = x\f$ (linear gain). Some barriers use non-linear gain
functions for smoother behaviour — e.g. `BodySphericalBarrier` uses
\f$g(x) = x / (1 + |x|)\f$.

**Implementation** (`Barriers.cpp`):
```cpp
Eigen::MatrixXd G = -jac / dt;
for (int i = 0; i < dim_; ++i)
    h(i) = gain_(i) * gain_function_(barrier(i));
```

### Safe-displacement objective

In addition to the hard constraint, barriers optionally add a
safe-displacement cost that pushes the robot away from the boundary:

\f[
H_{\text{barrier}} = \frac{\kappa}{\|J_h\|_F^2}\,I_{n_v}, \qquad
c_{\text{barrier}} = -\frac{\kappa}{\|J_h\|_F^2}\,\Delta q_{\text{safe}}
\f]

where \f$\kappa\f$ is the @b safe displacement gain @b and
\f$\Delta q_{\text{safe}}\f$ is a known-safe displacement (default: zero,
overridable by subclasses).

When \f$\kappa = 0\f$ only the hard constraint is active. When
\f$\kappa > 0\f$ the solver also actively moves the robot away from
the unsafe boundary. The normalisation by \f$\|J_h\|_F^2\f$ ensures the
regularisation magnitude is invariant to the barrier Jacobian's scale.

**Implementation** (`Barriers.cpp`):
```cpp
double weight = safe_displacement_gain_ / jac_sq_norm;
H.diagonal().array() += weight;
c -= weight * safe_dq;
```

---

## Common parameters {#barrier_common_params}

| Parameter | Symbol | Effect |
|-----------|--------|--------|
| `gain` | \f$\alpha\f$ | How aggressively the barrier is enforced. Higher = stricter. Can be scalar or per-dimension vector. |
| `safe_displacement_gain` | \f$\kappa\f$ | Strength of the "push away" objective. 0 = constraint only, >0 = active avoidance. |
| Custom gain function | \f$g(\cdot)\f$ | Non-linear mapping applied to \f$h(q)\f$ before the gain. Set via `setGainFunction()`. |

---

## PositionBarrier {#positionbarrier}

@b Purpose: Constrain the world-frame position of a named frame to lie
within a box \f$[p_{\min}, p_{\max}]\f$ along selected Cartesian axes.

### Barrier function

For selected axes \f$i \in \{x, y, z\}\f$:

\f[
h(q) = \begin{bmatrix}
  p^{(i)} - p_{\min}^{(i)} & \text{(if } p_{\min} \text{ given)} \\
  p_{\max}^{(i)} - p^{(i)} & \text{(if } p_{\max} \text{ given)}
\end{bmatrix}
\f]

Each component is non-negative when the frame position is within bounds.
The dimension equals the number of active bounds (lower + upper per axis).

### Jacobian

\f[
J = \begin{bmatrix}
  +S_{\text{axes}} \cdot R \cdot J_{\text{frame}}^{\text{lin}} & \text{(lower)} \\
  -S_{\text{axes}} \cdot R \cdot J_{\text{frame}}^{\text{lin}} & \text{(upper)}
\end{bmatrix}
\f]

where \f$R\f$ is the frame's rotation (to convert from local to world
frame), \f$J_{\text{frame}}^{\text{lin}}\f$ is the linear (top 3 rows)
part of the local frame Jacobian, and \f$S_{\text{axes}}\f$ selects the
active Cartesian axes.

### Use cases

- **Table avoidance**: Keep end-effector above a surface (\f$z > z_{\min}\f$).
- **Workspace bounds**: Restrict the robot to a work area.
- **CoM height**: Prevent a humanoid from squatting too low.

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame` | string | Frame to monitor. |
| `indices` | vector<int> | Cartesian axes (0=x, 1=y, 2=z). Default: all. |
| `p_min` | VectorXd (optional) | Minimum position bound (per selected axis). |
| `p_max` | VectorXd (optional) | Maximum position bound (per selected axis). |
| `gain` | double | Barrier gain \f$\alpha\f$. |
| `safe_displacement_gain` | double | Safe displacement cost \f$\kappa\f$. |

At least one of `p_min` or `p_max` must be provided.

### Example

```cpp
// Keep end-effector above z = 0.05m and below z = 0.40m
Eigen::VectorXd z_min(1), z_max(1);
z_min << 0.05;
z_max << 0.40;

auto barrier = std::make_unique<torq::PositionBarrier>(
    "end_effector",
    std::vector<int>{2},    // Z axis
    z_min, z_max,
    1.0,    // gain
    0.5     // safe displacement gain
);
robot.addBarrier(std::move(barrier));
```

---

## BodySphericalBarrier {#bodysphericalbarrier}

@b Purpose: Maintain a minimum distance between two frames using a
spherical (point-to-point) approximation. Useful for collision avoidance
between specific body pairs.

### Barrier function (1-D)

\f[
h(q) = \|p_1(q) - p_2(q)\|^2 - d_{\min}^2
\f]

The barrier is non-negative when the distance between the frames exceeds
\f$d_{\min}\f$. Using the squared distance avoids a square root in the
hot path and keeps the Jacobian smooth.

### Jacobian (1 x \f$n_v\f$)

\f[
J = 2(p_1 - p_2)^\top (J_1^{\text{world}} - J_2^{\text{world}})
\f]

where \f$J_i^{\text{world}} = R_i \cdot J_i^{\text{local,lin}}\f$ are the
world-frame linear Jacobians of each frame.

### Gain function

`BodySphericalBarrier` uses a non-linear gain function for smooth approach
behaviour:

\f[
g(h) = \frac{h}{1 + |h|}
\f]

This saturates the gain for large barrier values, preventing the constraint
from being overly conservative when the frames are far apart, while still
being strict near the boundary.

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame1`, `frame2` | string | Two frame names to monitor. |
| `d_min` | double | Minimum distance [m]. |
| `gain` | double | Barrier gain \f$\alpha\f$. |
| `safe_displacement_gain` | double | \f$\kappa\f$ (default 3.0). |

### Example

```cpp
auto barrier = std::make_unique<torq::BodySphericalBarrier>(
    "forearm_link", "base_link",
    0.08,   // 8 cm minimum distance
    1.0,    // gain
    3.0     // safe displacement gain
);
robot.addBarrier(std::move(barrier));
```

---

## SelfCollisionBarrier {#selfcollisionbarrier}

@b Purpose: Monitor multiple collision pairs from the Pinocchio geometry
model and keep their distances above a minimum threshold. Provides
whole-body self-collision avoidance.

### Barrier function (\f$N\f$-D)

For the \f$N\f$ closest collision pairs (sorted by distance each tick):

\f[
h_i(q) = d(p^1_i, p^2_i) - d_{\min}, \quad i = 1, \ldots, N
\f]

where \f$d(\cdot, \cdot)\f$ is the minimum distance between collision
geometries, computed by Pinocchio's collision detection.

### Jacobian (\f$N \times n_v\f$)

For each monitored pair \f$i\f$ with nearest points \f$w_1, w_2\f$ on
bodies attached to joints \f$j_1, j_2\f$:

\f[
J_i = \hat{n}^\top J_{j_1}^{\text{lin}}
    + (\hat{r}_1 \times \hat{n})^\top J_{j_1}^{\text{ang}}
    - \hat{n}^\top J_{j_2}^{\text{lin}}
    - (\hat{r}_2 \times \hat{n})^\top J_{j_2}^{\text{ang}}
\f]

where:
- \f$\hat{n} = (w_1 - w_2) / \|w_1 - w_2\|\f$ is the contact normal.
- \f$\hat{r}_k = w_k - o_{j_k}\f$ is the vector from joint \f$k\f$'s
  origin to the nearest point.
- Joint Jacobians use `LOCAL_WORLD_ALIGNED` reference frame.

### Requirements

- The `Configuration` must have a collision model loaded (URDF with
  collision geometries).
- Call `pinocchio::computeDistances()` before the barrier evaluates (this
  is handled by `Configuration` when collision is enabled).

### Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `n_collision_pairs` | int | Number of closest pairs to monitor per tick. |
| `gain` | double | Barrier gain \f$\alpha\f$. |
| `safe_displacement_gain` | double | \f$\kappa\f$ (default 1.0). |
| `d_min` | double | Minimum distance [m] (default 0.02). |

### Example

```cpp
auto barrier = std::make_unique<torq::SelfCollisionBarrier>(
    5,      // monitor 5 closest pairs
    1.0,    // gain
    1.0,    // safe displacement gain
    0.02    // 2 cm minimum distance
);
robot.addBarrier(std::move(barrier));
```

---

## How barriers fit into the QP

Barriers contribute to both sides of the QP:

### 1. Inequality constraints (hard safety)

Barrier inequality rows are stacked alongside limit rows:

\f[
G = \begin{bmatrix} G_{\text{limits}} \\ G_{\text{barriers}} \end{bmatrix}, \quad
h = \begin{bmatrix} h_{\text{limits}} \\ h_{\text{barriers}} \end{bmatrix}
\f]

### 2. Objective terms (active avoidance)

Barrier objective contributions are summed with task contributions:

\f[
P = \lambda I + \sum_{\text{tasks}} H_i + \sum_{\text{barriers}} H_j, \quad
c = \sum_{\text{tasks}} c_i + \sum_{\text{barriers}} c_j
\f]

The overall QP structure remains unchanged — barriers are additive to
both the objective and constraints.

---

## Tuning barriers

### Gain too low (\f$\alpha \ll 1\f$)

The barrier allows the robot to get very close to the boundary before
reacting. Risk of constraint violation if the solver cannot find a
feasible solution in time.

### Gain too high (\f$\alpha \gg 1\f$)

The barrier becomes very conservative, preventing the robot from
approaching the boundary even when it is safe to do so. Can reduce the
effective workspace.

### Safe displacement gain (\f$\kappa\f$)

- \f$\kappa = 0\f$: Constraint only. The barrier prevents violation but
  does not actively move the robot away from the boundary.
- \f$\kappa > 0\f$: Active avoidance. The robot is pushed away from
  the boundary by the safe-displacement objective. Higher values
  produce stronger avoidance.

### Practical recommendations

| Barrier | Typical gain | Typical \f$\kappa\f$ | Notes |
|---------|-------------|---------------------|-------|
| PositionBarrier | 1.0 | 0.0 – 1.0 | Use moderate \f$\kappa\f$ for workspace bounds. |
| BodySphericalBarrier | 1.0 | 3.0 | Higher \f$\kappa\f$ because the non-linear gain function is already smooth. |
| SelfCollisionBarrier | 1.0 | 1.0 | Keep \f$d_{\min}\f$ conservative (2+ cm) for mesh approximation error. |
