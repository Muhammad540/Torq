# Barriers {#barriers_page}

A **barrier** enforces a safety region with a Control Barrier Function
(CBF). Define \f$h(q) \ge 0\f$ on the safe set. It is a:

- a hard inequality row that prevents leaving the safe set, and
- (optionally) an objective term that pushes the robot away from the
  boundary.

The CBF condition

\f[
\frac{\partial h}{\partial q}\,\dot q + \alpha\, h(q) \;\ge\; 0
\f]

becomes (with \f$\Delta q = \dot q\, \Delta t\f$):

\f[
-\frac{1}{\Delta t}\,\frac{\partial h}{\partial q}\,\Delta q
\;\le\; \alpha \cdot g\!\bigl(h(q)\bigr)
\f]

where \f$g(\cdot)\f$ is a per barrier gain function (default identity).

| Parameter | Symbol | Effect |
|-----------|--------|--------|
| `gain`                   | \f$\alpha\f$ | How aggressively to enforce the constraint. |
| `safe_displacement_gain` | \f$\kappa\f$ | If > 0, also push actively away from the boundary. |

Add barriers via `RobotSystem::addBarrier(std::unique_ptr<Barrier>)`.

| Class | Dim | Purpose |
|-------|-----|---------|
| @ref torq::PositionBarrier "PositionBarrier"               | up to 6 | Box bounds on a frame's Cartesian position |
| @ref torq::BodySphericalBarrier "BodySphericalBarrier"     | 1       | Minimum distance between two frames |

---

## PositionBarrier {#positionbarrier}

Constrain the world position of a frame to a box \f$[p_\text{min}, p_\text{max}]\f$
along selected Cartesian axes:

\f[
h(q) = \begin{bmatrix}
  p^{(i)} - p_\text{min}^{(i)} \\
  p_\text{max}^{(i)} - p^{(i)}
\end{bmatrix}
\f]

Useful for defining the workspace of the robot.

| Parameter | Meaning |
|-----------|---------|
| `frame`    | Frame to monitor. |
| `indices`  | Cartesian axes (subset of \f$\{0,1,2\}\f$). |
| `p_min`, `p_max` | One or both bounds, per selected axis. |
| `gain`, `safe_displacement_gain` | CBF parameters. |

---

## BodySphericalBarrier {#bodysphericalbarrier}

Maintain a minimum distance between two frames using a point to point
(spherical) approximation:

\f[
h(q) = \|p_1(q) - p_2(q)\|^2 - d_\text{min}^2
\f]

A non linear gain \f$g(h) = h / (1 + |h|)\f$ smooths the response when the
frames are far apart so the barrier does not over react.

| Parameter | Meaning |
|-----------|---------|
| `frame1`, `frame2` | Two frames to keep apart. |
| `d_min`            | Minimum distance [m]. |
| `gain`, `safe_displacement_gain` | CBF parameters. |

---

See @ref extending_page for writing your own barrier.
