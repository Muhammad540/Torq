# QP Formulation {#qp_formulation}

[update qp formulation has missing barriers]
Torq solves differential inverse kinematics as a @b Quadratic Program (QP) @b at
every control tick.  This page derives the complete formulation explaining how tasks,
limits, and damping compose into a single optimisation problem.

## Overview

Given the current robot configuration \f$q\f$, the IK solver finds a
displacement \f$\Delta q \in \mathbb{R}^{n_v}\f$ (tangent-space velocity
\f$\times\,\Delta t\f$) that best satisfies all active tasks while respecting
physical limits and defined barriers.

\f[
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q \\
\text{s.t.} \quad & G\,\Delta q \le h
\end{aligned}
\f]

where \f$P\f$ (positive semi-definite) and \f$c\f$ accumulate task objectives,
and the rows of \f$(G, h)\f$ encode limit and barrier constraints. [update: check again]

---

## Per-task contribution

Every task \f$i\f$ defines:

| Symbol | Meaning | Shape |
|--------|---------|-------|
| \f$e_i(q)\f$ | Task error | \f$k_i \times 1\f$ |
| \f$J_i(q)\f$ | Task Jacobian | \f$k_i \times n_v\f$ |
| \f$w_i\f$ | Cost (weight) vector | \f$k_i \times 1\f$ |
| \f$\alpha_i\f$ | Gain \f$\in (0, 1]\f$ | scalar |
| \f$\mu_i\f$ | Levenberg–Marquardt damping | scalar |
[update: add a column briefly describing the contribution]
### Weight matrix

The diagonal weight matrix \f$W_i = \operatorname{diag}(w_i)\f$ scales each
component of the error independently.  For `FrameTask` the 6-D weight vector
is \f$[w_{\text{pos}}^{(3)},\; w_{\text{ori}}^{(3)}]\f$.

### Weighted quantities

\f[
\bar{J}_i = W_i \, J_i, \qquad
\bar{e}_i = W_i \left(-\alpha_i \, e_i\right)
\f]

The task wants \f$\bar{J}_i \, \Delta q \approx \bar{e}_i\f$, which in
least-squares form is:

\f[
\min_{\Delta q}\; \tfrac{1}{2}\left\|\bar{J}_i\,\Delta q - \bar{e}_i\right\|^2
\f]

### Levenberg–Marquardt damping

When the target is unreachable the error can be large and the solver may
produce huge velocities.  LM damping adds a configuration-space regularisation
proportional to the task's own residual:

\f[
\mu_i^{\text{eff}} = \mu_i \cdot \bar{e}_i^\top \bar{e}_i
\f]

### Per-task Hessian and gradient

\f[
H_i = \bar{J}_i^\top \bar{J}_i + \mu_i^{\text{eff}}\, I_{n_v}, \qquad
c_i = -\bar{J}_i^\top \bar{e}_i
\f]

---

## Composing multiple tasks

Because tasks live in the same \f$\Delta q\f$ space, the QP Hessian and
gradient are simply summed:

\f[
P = \sum_{i} H_i, \qquad c = \sum_{i} c_i
\f]

The relative importance of each task is encoded in its weight vector \f$w_i\f$.
Higher cost → the solver prioritises that task more aggressively.

---

## Tikhonov (solver) damping

A small global regularisation \f$\lambda\f$ (default \f$10^{-12}\f$) is added
to the diagonal of \f$P\f$ to guarantee positive definiteness even when tasks
do not fully constrain all DOFs:

\f[
P \leftarrow P + \lambda \, I_{n_v}
\f]

Unlike LM damping (per-task, error-dependent), Tikhonov damping is a constant
background term.

---

## Inequality constraints from limits

Each `Limit` produces rows \f$(G_j, h_j)\f$ such that
\f$G_j\,\Delta q \le h_j\f$.

All limit rows are stacked vertically:

\f[
G = \begin{bmatrix} G_1 \\ G_2 \\ \vdots \end{bmatrix}, \qquad
h = \begin{bmatrix} h_1 \\ h_2 \\ \vdots \end{bmatrix}
\f]

See @ref limits_page for the exact derivation of each limit type.

---

## The complete QP

\f[
\boxed{
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top
  \left[\sum_i H_i + \lambda I\right] \Delta q
  + \left[\sum_i c_i\right]^\top \Delta q \\[4pt]
\text{s.t.} \quad & G\,\Delta q \le h
\end{aligned}
}
\f]

---

## OSQP solver

Torq uses [OSQP](https://osqp.org/) via
[OsqpEigen](https://github.com/robotology/osqp-eigen).  OSQP expects the
standard form:

\f[
\min_x \;\tfrac{1}{2}\,x^\top P\,x + q^\top x
\quad\text{s.t.}\quad l \le A\,x \le u
\f]

The mapping from Torq's `QPProblem` is:

| Torq | OSQP |
|------|------|
| \f$P\f$ (Hessian) | \f$P\f$ (upper triangle, sparse) |
| \f$c\f$ | \f$q\f$ |
| \f$G\f$ | \f$A\f$ |
| \f$-\infty\f$ | \f$l\f$ |
| \f$h\f$ | \f$u\f$ |

The solver is warm-started across ticks: the sparsity pattern is set once and
only the numerical values are updated on subsequent solves.

---

## Manifold-aware integration

The solved \f$\Delta q\f$ is a tangent vector.  For joints with manifold
structure (e.g. floating-base quaternions) Torq uses Pinocchio's
`pinocchio::integrate` instead of simple addition:

\f[
q_{\text{new}} = q \oplus \Delta q
\f]

For revolute and prismatic joints \f$\oplus\f$ reduces to ordinary addition,
so there is no overhead for fixed-base arms.
