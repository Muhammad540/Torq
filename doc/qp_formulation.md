# QP Formulation {#qp_formulation}

Torq solves differential inverse kinematics as a @b Quadratic Program (QP)
at every control tick. This page derives the complete formulation, showing
how tasks, limits, and barriers compose into a single optimisation problem.

See @ref conventions for notation and @ref tasks_page, @ref limits_page,
@ref barriers_page for the per-component derivations.

---

## Overview

Given the current robot configuration \f$q\f$, the IK solver finds a
displacement \f$\Delta q \in \mathbb{R}^{n_v}\f$ (tangent-space velocity
\f$\times\,\Delta t\f$) that best satisfies all active tasks while respecting
physical limits and safety barriers:

\f[
\boxed{
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top P\,\Delta q + c^\top \Delta q \\
\text{s.t.} \quad & G\,\Delta q \le h
\end{aligned}
}
\f]

where:

| Component | Source | Role |
|-----------|--------|------|
| \f$P\f$ (positive semi-definite) | Tasks + barrier objectives + Tikhonov damping | Quadratic cost on \f$\Delta q\f$ |
| \f$c\f$ | Tasks + barrier objectives | Linear cost on \f$\Delta q\f$ |
| \f$G, h\f$ | Limits + barrier inequalities | Hard inequality constraints |

---

## Per-task contribution

Every task \f$i\f$ defines:

| Symbol | Meaning | Shape | Contribution |
|--------|---------|-------|--------------|
| \f$e_i(q)\f$ | Task error | \f$k_i \times 1\f$ | Drives the objective gradient |
| \f$J_i(q)\f$ | Task Jacobian | \f$k_i \times n_v\f$ | Maps \f$\Delta q\f$ to task-space changes |
| \f$w_i\f$ | Cost (weight) vector | \f$k_i \times 1\f$ | Scales error and Jacobian components |
| \f$\alpha_i\f$ | Gain \f$\in (0, 1]\f$ | scalar | Controls convergence speed |
| \f$\mu_i\f$ | LM damping | scalar | Regularises near singularities |

### Weight matrix

The diagonal weight matrix \f$W_i = \operatorname{diag}(w_i)\f$ scales each
component of the error independently. For `FrameTask` the 6-D weight vector
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

### Levenberg-Marquardt damping

When the target is unreachable the error can be large and the solver may
produce huge velocities. LM damping adds a configuration-space regularisation
proportional to the task's own residual:

\f[
\mu_i^{\text{eff}} = \mu_i \cdot \bar{e}_i^\top \bar{e}_i
\f]

This is adaptive: the damping is strong when the residual is large
(unreachable target) and vanishes when the error is small.

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
P_{\text{tasks}} = \sum_{i} H_i, \qquad c_{\text{tasks}} = \sum_{i} c_i
\f]

The relative importance of each task is encoded in its weight vector
\f$w_i\f$. Higher cost → the solver prioritises that task more aggressively.

For the default built-in tasks:

| Task | Typical cost | Effect |
|------|-------------|--------|
| FrameTask | 1.0 | Primary objective: track end-effector |
| PostureTask | \f$10^{-3}\f$ | Regularisation: stay near home |
| DampingTask | \f$10^{-4}\f$ | Smoothing: minimise motion |

---

## Tikhonov (solver) damping

A small global regularisation \f$\lambda\f$ (default \f$10^{-12}\f$) is added
to the diagonal of \f$P\f$ to guarantee positive definiteness even when tasks
do not fully constrain all DOFs:

\f[
P \leftarrow P + \lambda \, I_{n_v}
\f]

Unlike LM damping (per-task, error-dependent), Tikhonov damping is a constant
background term that ensures the QP is always solvable.

---

## Inequality constraints from limits

Each `Limit` produces rows \f$(G_j, h_j)\f$ such that
\f$G_j\,\Delta q \le h_j\f$.

All limit rows are stacked vertically:

\f[
G_{\text{limits}} = \begin{bmatrix} G_1 \\ G_2 \\ \vdots \end{bmatrix}, \qquad
h_{\text{limits}} = \begin{bmatrix} h_1 \\ h_2 \\ \vdots \end{bmatrix}
\f]

See @ref limits_page for the exact derivation of each limit type.

---

## Barrier contributions

Each `Barrier` contributes to @b both @b the objective and the constraints:

### Barrier inequality rows

The CBF condition for barrier \f$j\f$:

\f[
G_j = -\frac{1}{\Delta t}\,\frac{\partial h_j}{\partial q}, \qquad
h_j^{\text{rhs}} = \alpha_j \cdot g\!\left(h_j(q)\right)
\f]

where \f$g(\cdot)\f$ is the barrier's gain function (default: identity).

Barrier rows are stacked alongside limit rows:

\f[
G = \begin{bmatrix} G_{\text{limits}} \\ G_{\text{barriers}} \end{bmatrix}, \qquad
h = \begin{bmatrix} h_{\text{limits}} \\ h_{\text{barriers}} \end{bmatrix}
\f]

### Barrier objective terms

If the safe-displacement gain \f$\kappa_j > 0\f$:

\f[
H_j^{\text{barrier}} = \frac{\kappa_j}{\|J_j\|_F^2}\,I_{n_v}, \qquad
c_j^{\text{barrier}} = -\frac{\kappa_j}{\|J_j\|_F^2}\,\Delta q_{\text{safe},j}
\f]

These are summed into the QP objective alongside task contributions.

---

## The complete QP

Putting it all together:

\f[
\boxed{
\begin{aligned}
\min_{\Delta q} \quad & \tfrac{1}{2}\,\Delta q^\top
  \underbrace{\left[
    \lambda I
    + \sum_{i \in \text{tasks}} H_i
    + \sum_{j \in \text{barriers}} H_j^{\text{barrier}}
  \right]}_{P}\,\Delta q
  + \underbrace{\left[
    \sum_{i \in \text{tasks}} c_i
    + \sum_{j \in \text{barriers}} c_j^{\text{barrier}}
  \right]^\top}_{c^\top}\,\Delta q \\[8pt]
\text{s.t.} \quad &
  \underbrace{\begin{bmatrix}
    G_{\text{limits}} \\ G_{\text{barriers}}
  \end{bmatrix}}_{G}\,\Delta q
  \;\le\;
  \underbrace{\begin{bmatrix}
    h_{\text{limits}} \\ h_{\text{barriers}}
  \end{bmatrix}}_{h}
\end{aligned}
}
\f]

---

## OSQP solver

Torq uses [OSQP](https://osqp.org/) via
[OsqpEigen](https://github.com/robotology/osqp-eigen). OSQP expects the
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

The solver is warm-started across ticks: the sparsity pattern is set once
and only the numerical values are updated on subsequent solves. When there
are no inequality constraints (\f$m = 0\f$), Torq uses a direct LDLT
decomposition instead of OSQP for better performance.

---

## Manifold-aware integration

The solved \f$\Delta q\f$ is a tangent vector. For joints with manifold
structure (e.g. floating-base quaternions) Torq uses Pinocchio's
`pinocchio::integrate` instead of simple addition:

\f[
q_{\text{new}} = q \oplus \Delta q
\f]

For revolute and prismatic joints \f$\oplus\f$ reduces to ordinary addition,
so there is no overhead for fixed-base arms. For floating-base robots, the
quaternion component is updated using the exponential map, maintaining
the unit-norm constraint and avoiding gimbal lock.

See @ref conventions for a full explanation of manifold operations.

---

## Velocity output

The QP solves for a displacement \f$\Delta q\f$. The corresponding
generalised velocity is:

\f[
v = \frac{\Delta q}{\Delta t}
\f]

This velocity is returned to the caller and used by limits (which scale
their bounds by \f$\Delta t\f$) and by `Configuration::integrate()`.

---

## Summary of the assembly pipeline

1. **Tikhonov**: \f$P = \lambda I\f$, \f$c = 0\f$.
2. **Tasks**: For each task, compute \f$(H_i, c_i)\f$ → \f$P \mathrel{+}= H_i\f$, \f$c \mathrel{+}= c_i\f$.
3. **Barrier objectives**: For each barrier with \f$\kappa > 0\f$, compute \f$(H_j, c_j)\f$ → \f$P \mathrel{+}= H_j\f$, \f$c \mathrel{+}= c_j\f$.
4. **Limits**: For each limit, compute \f$(G_j, h_j)\f$ → stack into \f$(G, h)\f$.
5. **Barrier inequalities**: For each barrier, compute \f$(G_j, h_j)\f$ → stack into \f$(G, h)\f$.
6. **Solve**: If \f$m > 0\f$ use OSQP; else use direct LDLT.
7. **Integrate**: \f$q_{\text{new}} = q \oplus \Delta q\f$.
