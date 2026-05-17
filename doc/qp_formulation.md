# QP Formulation {#qp_formulation}

Torq solves a single Quadratic Program at every control tick. The decision
variable is the joint displacement \f$\Delta q \in \mathbb{R}^{n_v}\f$
(tangent space velocity \f$\times\,\Delta t\f$):

\f[
\boxed{
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q \\
\text{s.t.} \quad & G\,\Delta q \le h
\end{aligned}}
\f]

| Term | Comes from |
|------|-----------|
| \f$P\f$ (Hessian)  | \f$\lambda I + \sum_i H_i^\text{task} + \sum_j H_j^\text{barrier}\f$ |
| \f$c\f$ (gradient) | \f$\sum_i c_i^\text{task} + \sum_j c_j^\text{barrier}\f$ |
| \f$G, h\f$         | Stacked rows from every active limit and barrier inequality |

---

## Per task contribution

Each task has error \f$e\in\mathbb{R}^k\f$, Jacobian
\f$J\in\mathbb{R}^{k\times n_v}\f$, weight vector \f$w\in\mathbb{R}^k\f$,
gain \f$\alpha\in(0,1]\f$, and LM damping \f$\mu \ge 0\f$. With
\f$W = \operatorname{diag}(w)\f$, \f$\bar J = WJ\f$,
\f$\bar e = W(-\alpha e)\f$:

\f[
H_i = \bar J^\top \bar J + \mu\,\|\bar e\|^2\, I_{n_v},
\qquad
c_i = -\bar J^\top \bar e
\f]

The LM term is adaptive: it only becomes effective when the residual is large (e.g.
unreachable target), so well tracked tasks pay no regularization cost.

A small Tikhonov damping \f$\lambda I\f$ (default \f$10^{-12}\f$) is added
to keep \f$P\f$ positive definite when tasks under constrain the DOFs.

---

## Per barrier contribution

A barrier with value \f$h(q)\in\mathbb{R}^d\f$ and Jacobian
\f$\partial h/\partial q\f$ produces:

- @b Inequality @b rows (discrete CBF, one row per scalar barrier component):
  \f[
  G_j = -\frac{\partial h}{\partial q},
  \qquad
  h_j^\text{rhs} = \alpha \odot h(q)
  \f]
  The QP variable is \f$\Delta q\f$; this enforces \f$h(q) + (\partial h/\partial q)\,\Delta q \ge 0\f$.
- @b Optional @b objective @b terms (when `safe_displacement_gain` \f$\kappa > 0\f$):
  \f[
  H_j^\text{barrier} = \kappa\,I_{n_v},
  \qquad
  c_j^\text{barrier} = -\kappa\,\Delta q_\text{safe}
  \f]
  with \f$\Delta q_\text{safe}\f$ from `computeSafeDisplacement` (zero unless overridden).

@b Limit @b rows @b are @b appended @b directly @b to \f$G, h\f$ @b without @b any @b objective @b contribution.

---

## Solver

Torq uses [OSQP](https://osqp.org/) via OsqpEigen with warm start. Settings
include absolute/relative tolerance \f$10^{-5}\f$, up to 200 iterations, and
`polish` disabled. The solver is reinitialised only when \f$(n_v, m)\f$
change. The constraint matrix is passed in sparse form with a fixed sparsity
pattern each tick (structural zeros use a tiny filler value for OSQP updates).

When there are no inequality constraints, the solver falls back to a direct
LDLT solve on \f$P\f$.

### Post-solve safety

OSQP can return `NoError` while the primal grossly violates
\f$G\,\Delta q \le h\f$ (e.g. infeasible or ill-conditioned QPs). After each
solve, Torq checks the minimum constraint **slack**

\f[
\text{slack}_i = h_i - G_i\,\Delta q,
\qquad
\text{slack}_\min = \min_i \text{slack}_i
\f]

If \f$\text{slack}_\min < -10^{-4}\f$, the step is discarded and **zero
velocity** is returned for that tick.

Otherwise \f$\Delta q\f$ is **clamped** per joint so
\f$|\Delta q_i| \le \Delta t\,\dot q_{\max,i}\f$ wherever
`model.velocityLimit` is finite and positive, then

\f[
v = \Delta q / \Delta t
\f]

Clamping enforces velocity caps even if the QP primal overshoots; it does not
re-check full \f$G\,\Delta q \le h\f$ after clipping.

### Integration

The velocity is integrated on the configuration manifold:

\f[
q_\text{new} = q \oplus \Delta q
\f]

For revolute and prismatic joints \f$\oplus\f$ reduces to ordinary addition;
for a floating base, quaternions use the exponential map. See @ref conventions.

### Practical note

Start the robot in a **feasible** state when using barriers (e.g. end-effector
inside a `PositionBarrier` box before enabling a frame task that pulls toward
a target). If \f$h(q) < 0\f$ on any barrier row at the first task-space tick,
the stacked QP is often infeasible and many ticks may command zero motion until
the configuration becomes feasible.
