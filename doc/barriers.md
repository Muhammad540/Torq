# Barriers {#barriers_page}

> @b Status: @b Barriers are @b planned but not yet implemented @b in Torq.  This
> page documents the intended design, based on
> [Pink](https://github.com/stephane-caron/pink)'s barrier system.

---

## Control Barrier Functions (CBFs)

A barrier enforces a safety constraint \f$h(q) \ge 0\f$ by adding inequality
rows to the QP that prevent the robot from entering unsafe regions.

### CBF inequality

For a barrier function \f$h : \mathcal{Q} \to \mathbb{R}\f$:

\f[
\nabla_q h(q)^\top \Delta q \;\ge\; -\gamma_b\; h(q)
\f]

or equivalently (in \f$G\,\Delta q \le h\f$ form):

\f[
-\nabla_q h(q)^\top \Delta q \;\le\; \gamma_b\; h(q)
\f]

where \f$\gamma_b > 0\f$ is the @b barrier gain @b .

### Safe displacement objective

In addition to the constraint, barriers add a secondary objective that pushes
the robot away from the boundary:

\f[
\min_{\Delta q}\; \tfrac{1}{2}\,\kappa \left\|\nabla_q h(q)^\top \Delta q
+ \gamma_b\; h(q)\right\|^2
\f]

where \f$\kappa\f$ is the @b safe displacement gain @b .  When \f$\kappa = 0\f$
only the hard constraint is active; when \f$\kappa > 0\f$ the solver also
actively moves the robot away from the unsafe boundary.

---

## Planned barrier types

### PositionBarrier

Bound the Cartesian position of a frame along selected axes.

\f[
p_{\min}^{(i)} \le p_{\text{frame}}^{(i)} \le p_{\max}^{(i)}, \quad
i \in \{\text{x}, \text{y}, \text{z}\}
\f]

| Parameter | Type | Description |
|-----------|------|-------------|
| `frame` | string | Frame to monitor |
| `indices` | list of int | Position axes (0=x, 1=y, 2=z) |
| `p_min` / `p_max` | vector | Minimum / maximum position bounds |
| `gain` | float | Barrier gain \f$\gamma_b\f$ |
| `safe_displacement_gain` | float | \f$\kappa\f$ |

### BodySphericalBarrier

Maintain minimum distance between two frames (collision avoidance between
specific body pairs).

\f[
h(q) = \|p_A(q) - p_B(q)\| - d_{\min}
\f]

| Parameter | Type | Description |
|-----------|------|-------------|
| `frames` | (string, string) | Two frame names |
| `d_min` | float | Minimum distance (m) |
| `gain` | float | Barrier gain \f$\gamma_b\f$ |
| `safe_displacement_gain` | float | \f$\kappa\f$ (default 3.0) |

### SelfCollisionBarrier

Monitor multiple collision pairs from the Pinocchio geometry model and keep
them above a minimum distance.

\f[
h_k(q) = d_k(q) - d_{\min}, \quad k = 1, \dots, n_{\text{pairs}}
\f]

| Parameter | Type | Description |
|-----------|------|-------------|
| `n_collision_pairs` | int | Number of closest pairs to monitor |
| `d_min` | float | Minimum distance (m), default 0.02 |
| `gain` | float | Barrier gain \f$\gamma_b\f$ |
| `safe_displacement_gain` | float | \f$\kappa\f$ (default 1.0) |

---

## How barriers fit into the QP

Once implemented, barriers will contribute:

1. @b Inequality rows @b to the constraint matrix \f$(G, h)\f$, alongside
   velocity and configuration limits.
2. @b Objective terms @b to \f$(P, c)\f$ via the safe-displacement cost
   (analogous to task contributions but with barrier-specific weights).

The overall QP structure remains unchanged — barriers are additive.

---

## Common barrier parameters

| Parameter | Symbol | Effect |
|-----------|--------|--------|
| `gain` | \f$\gamma_b\f$ | How aggressively the barrier is enforced. Higher = stricter. |
| `safe_displacement_gain` | \f$\kappa\f$ | Strength of the "push away" objective. 0 = constraint only. |
| `d_min` | \f$d_{\min}\f$ | Minimum allowable distance/clearance. |
