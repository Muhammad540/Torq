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

- @b Inequality @b rows that enforce the CBF condition:
  \f[
  G_j = -\frac{1}{\Delta t}\,\frac{\partial h}{\partial q},
  \qquad
  h_j^\text{rhs} = \alpha \cdot g\!\bigl(h(q)\bigr)
  \f]
- @b Optional @b objective @b terms (when `safe_displacement_gain` > 0):
  \f[
  H_j^\text{barrier} = \frac{\kappa}{\|J_h\|_F^2}\,I_{n_v},
  \qquad
  c_j^\text{barrier} = -\frac{\kappa}{\|J_h\|_F^2}\,\Delta q_\text{safe}
  \f]

@b Limit @b rows @b are @b appended @b directly @b to \f$G, h\f$ @b without @b any @b objective @b contribution.

---

## Solver

Torq uses [OSQP](https://osqp.org/) via OsqpEigen with the warm start of the
previous solution. The solver is reinitialised only when the problem
dimensions \f$(n_v, m)\f$ change. When there are no inequality constraints
the solver falls back to a direct LDLT solve.

The output \f$\Delta q\f$ is integrated on the configuration manifold:

\f[
q_\text{new} = q \oplus \Delta q
\f]

For revolute and prismatic joints \f$\oplus\f$ reduces to ordinary addition, 
for floating base quaternions it uses the exponential map. See
@ref conventions.
