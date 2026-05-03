# Tasks {#tasks_page}

A **task** is a soft objective in the QP. Each task contributes a Hessian
block \f$H_i\f$ and a gradient \f$c_i\f$ to the cost function. See
@ref qp_formulation for how they compose.

Every concrete task derives from `torq::Task` and provides:

\f[
J(q)\,\Delta q = -\alpha\, e(q)
\f]

The base class converts \f$(e, J)\f$ into a weighted least squares term:

\f[
\min_{\Delta q}\; \tfrac{1}{2}\,\bigl\| W\,(J\,\Delta q + \alpha\, e) \bigr\|^2
+ \tfrac{1}{2}\,\mu\,\|W(-\alpha e)\|^2\,\|\Delta q\|^2
\f]

| Symbol | Name | Notes |
|--------|------|-------|
| \f$w\f$ | cost / weight | Scalar or per component vector. Higher = higher priority. |
| \f$\alpha \in (0, 1]\f$ | gain | Convergence rate per tick (1.0 = dead beat). |
| \f$\mu \ge 0\f$ | LM damping | Levenberg Marquardt regularization, scales with squared error. |

The three built in tasks are constructed automatically the first time
`setTaskSpaceTarget` is called. You can add more with `RobotSystem::addTask`.

| Class | Dim | Purpose |
|-------|-----|---------|
| @ref torq::FrameTask "FrameTask"     | 6      | Track an SE(3) end effector pose |
| @ref torq::PostureTask "PostureTask" | \f$n_v\f$ | Bias joints toward a reference posture |
| @ref torq::DampingTask "DampingTask" | \f$n_v\f$ | Penalise joint motion (smoothing) |

---

## FrameTask {#frametask}

Drive a named frame to a desired pose \f$T_\text{des} \in SE(3)\f$.

\f[
e = \log_6\!\bigl(T_\text{cur}^{-1}\, T_\text{des}\bigr) \in \mathbb{R}^6,
\qquad
J = -J_{\log_6}\!\cdot {}^{\text{local}}J_\text{frame}(q)
\f]

The error is a 6D body frame twist and is zero at the target. The 6D
weight splits into translational and rotational components:
\f$w = [w_\text{pos}^{(3)},\, w_\text{ori}^{(3)}]\f$. When no target is set
the task is **inert** (contributes nothing).

| Parameter | Meaning |
|-----------|---------|
| `position_cost`   | Weight on translational error (set 0 for orientation only). |
| `orientation_cost`| Weight on rotational error (set 0 for position only). |
| `gain`            | \f$\alpha\f$, how fast to close the error. |
| `lm_damping`      | Activates near singular or unreachable targets. |

---

## PostureTask {#posturetask}

Regulate joints toward a reference configuration \f$q_\text{ref}\f$.

\f[
e = q_\text{ref} \ominus q_\text{cur} \in \mathbb{R}^{n_v},
\qquad
J = I_{n_v}
\f]

Manifold aware difference (`pinocchio::difference`) is used so floating base
quaternions are handled correctly. Floating base coordinates are excluded,
only actuated joints contribute. Acts as a low priority regularizer to keep
the arm in a sensible configuration; **inert** without a target.

| Parameter | Meaning |
|-----------|---------|
| `cost`       | Weight per joint. Typically 2-3 orders of magnitude below `FrameTask`. |
| `gain`       | \f$\alpha\f$, convergence rate toward the reference. |
| `lm_damping` | Optional LM regularization. |

---

## DampingTask {#dampingtask}

Penalise all joint velocities equally acts as viscous damping.

\f[
e = 0,
\qquad
J = I_{n_v}
\quad\Longrightarrow\quad
H = w^2 I_{n_v},\;\; c = 0
\f]

Always active once constructed. Equivalent to a small Tikhonov term: keeps
the solver well conditioned near singularities and selects the
minimum norm solution.

| Parameter | Meaning |
|-----------|---------|
| `cost` | Weight on \f$\|\Delta q\|^2\f$. Increase for smoother motion. |

---

See @ref tuning_guide for typical default values, and @ref extending_page
for writing your own task.
