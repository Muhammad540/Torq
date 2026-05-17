# Barriers {#barriers_page}

A **barrier** enforces a safety region with a Control Barrier Function
(CBF). Define \f$h(q) \ge 0\f$ on the safe set. It is a:

- a hard inequality row that prevents leaving the safe set, and
- (optionally) an objective term that pushes the robot away from the
  boundary.

The continuous-time CBF condition

\f[
\frac{\partial h}{\partial q}\,\dot q + \alpha\, h(q) \;\ge\; 0
\f]

is discretised with forward Euler on the QP decision variable
\f$\Delta q\f$ (joint displacement over one tick):

\f[
h(q) + \frac{\partial h}{\partial q}\,\Delta q \;\ge\; 0
\quad\Leftrightarrow\quad
G\,\Delta q \le h_\text{rhs},
\qquad
G = -\frac{\partial h}{\partial q},
\qquad
h_\text{rhs} = \alpha \odot h(q)
\f]

where \f$\alpha\f$ is the per-row `gain` vector.

`Barrier::updateQP` requires `dt > 0`. The implementation sets
`G = -\texttt{computeJacobian}` (rows of `computeJacobian` are
\f$\partial h / \partial q\f$).

| Parameter | Symbol | Effect |
|-----------|--------|--------|
| `gain`                   | \f$\alpha\f$ | Per-row scale on \f$h(q)\f$ in \f$h_\text{rhs}\f$ (constant; no runtime gain function). |
| `safe_displacement_gain` | \f$\kappa\f$ | If > 0, adds \f$\tfrac{\kappa}{2}\|\Delta q - \Delta q_\text{safe}\|^2\f$ with \f$\Delta q_\text{safe}\f$ from `computeSafeDisplacement` (default zero). |

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

`PositionBarrier` dimension is computed from which of `p_min` / `p_max` are set
and how many axes are in `indices` (up to six rows for full XYZ min and max).

---

## BodySphericalBarrier {#bodysphericalbarrier}

Maintain a minimum distance between two frames using a point to point
(spherical) approximation:

\f[
h(q) = \|p_1(q) - p_2(q)\|^2 - d_\text{min}^2
\f]

| Parameter | Meaning |
|-----------|---------|
| `frame1`, `frame2` | Two frames to keep apart. |
| `d_min`            | Minimum distance [m]. |
| `gain`, `safe_displacement_gain` | CBF parameters. |

---

See @ref extending_page for writing your own barrier.
