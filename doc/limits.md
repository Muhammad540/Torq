# Limits {#limits_page}

Limits define @b inequality constraints @b in the QP.  They ensure the solved
\f$\Delta q\f$ respects the physical bounds of the robot.  Each limit produces
rows \f$(G, h)\f$ such that \f$G\,\Delta q \le h\f$ — see
@ref qp_formulation for how constraints compose.

---

## VelocityLimit

@b Purpose: @b bound joint displacements so that the implied velocity does not
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
out the \f$k\f$ joints with finite velocity bounds.  Joints without a finite
`velocityLimit` in the URDF/MJCF are excluded.

### Parameters

None — limits are read directly from the Pinocchio model.

---

## ConfigurationLimit

@b Purpose: @b keep \f$q + \Delta q\f$ within the joint position bounds
\f$[q_{\min}, q_{\max}]\f$, using a proportional gain to steer away from
limits before they are hit.

### Formulation

For each bounded joint \f$j\f$ with current position \f$q_j\f$ and limits
\f$q_j^{\min}, q_j^{\max}\f$:

\f[
\Delta q_j \;\le\; \gamma\,(q_j^{\max} - q_j)
\f]
\f[
-\Delta q_j \;\le\; \gamma\,(q_j - q_j^{\min})
\f]

where \f$\gamma \in (0, 1]\f$ is the @b configuration limit gain @b
(`config_limit_gain`).

In matrix form for all \f$k\f$ bounded joints:

\f[
\begin{bmatrix} +S \\ -S \end{bmatrix} \Delta q
\;\le\;
\begin{bmatrix}
  \gamma\,(q^{\max}_{(k)} - q_{(k)}) \\
  \gamma\,(q_{(k)} - q^{\min}_{(k)})
\end{bmatrix}
\f]

### Effect of `config_limit_gain`

| \f$\gamma\f$ | Behaviour |
|---------------|-----------|
| 1.0 | Allow the full remaining range — only prevents actual violation. |
| 0.5 (default) | Permit half the remaining range per tick — smooth deceleration near limits. |
| 0.1 | Very conservative — robot slows down far from limits. |

Lower gain → earlier and smoother deceleration as the robot approaches a limit.

### Parameters

| Parameter | Symbol | Default | Setter |
|-----------|--------|---------|--------|
| `config_limit_gain` | \f$\gamma\f$ | 0.5 | `setConfigLimitGain(double)` |

---

## How limits stack

All limit types produce rows in the same inequality:

\f[
G = \begin{bmatrix} G_{\text{vel}} \\ G_{\text{cfg}} \end{bmatrix}, \quad
h = \begin{bmatrix} h_{\text{vel}} \\ h_{\text{cfg}} \end{bmatrix}
\f]

A limit may return `std::nullopt` if it has no active constraints (e.g. no
finite bounds in the model), in which case it contributes zero rows.

The OSQP solver handles the full stacked system simultaneously.

---

## Planned: equality constraints

Some tasks may in the future act as hard equality constraints rather than soft
objectives:

\f[
A_{\text{eq}}\,\Delta q = b_{\text{eq}}
\f]

This is not yet implemented in Torq but the `QPProblem` struct is designed to
be extended with equality rows.
