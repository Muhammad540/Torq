# Tasks {#tasks_page}

Tasks define the @b objective @b (cost) of the QP problem.  Each task contributes
a Hessian block \f$H_i\f$ and a gradient vector \f$c_i\f$ — see
@ref qp_formulation for how they compose.

This page derives the error and Jacobian for every concrete task type.

---

## Common parameters

Every task inherits from `torq::Task` and exposes:

| Parameter | Symbol | Range | Effect |
|-----------|--------|-------|--------|
| @b cost @b (weight) | \f$w\f$ | \f$(0, \infty)\f$ | Relative importance vs other tasks. Higher = prioritised. |
| @b gain @b | \f$\alpha\f$ | \f$(0, 1]\f$ | Convergence speed. 1.0 = dead-beat (full correction each tick). Lower values smooth the response. |
| @b lm_damping @b | \f$\mu\f$ | \f$[0, \infty)\f$ | Levenberg–Marquardt regularisation. Increase when target is unreachable to avoid velocity spikes. |

All three are adjustable at runtime via setters on the task and via
`RobotSystem`'s IK tuning API.

---

## FrameTask

@b Purpose: @b regulate the SE(3) pose of a named robot frame (typically the
end-effector) to a desired target \f$T_{\text{des}} \in SE(3)\f$.

### Error (6-D)

The 6-D error is computed using the @b logarithmic map @b of the relative
transform:

\f[
e = \log_6\!\left(T_{\text{cur}}^{-1}\; T_{\text{des}}\right)
  = \begin{bmatrix} e_{\text{pos}} \\ e_{\text{ori}} \end{bmatrix}
  \in \mathbb{R}^6
\f]

where \f$\log_6 : SE(3) \to \mathfrak{se}(3)\f$ maps the relative
transform into a twist in the **local** (body) frame.

This is a manifold-aware error: it correctly handles the topology of rotations
without gimbal lock or discontinuities.

### Jacobian (6×n_v)

The frame Jacobian in the @b LOCAL @b reference frame:

\f[
J = {}^{\text{local}}J_{\text{frame}}(q)
\f]

computed by `pinocchio::computeFrameJacobian` with `LOCAL` reference frame.
Using the local frame ensures consistency with the \f$\log_6\f$ error.

### Weight vector

The 6-D cost vector is \f$w = [w_{\text{pos}}^{(3)},\; w_{\text{ori}}^{(3)}]\f$
where the first three components weight translational error and the last three
weight rotational error.  This allows independent prioritisation of position vs
orientation tracking.

### QP contribution

\f[
W = \operatorname{diag}(w), \quad
\bar{J} = W J, \quad
\bar{e} = W(-\alpha\, e)
\f]
\f[
H_{\text{frame}} = \bar{J}^\top \bar{J}
  + \mu\,(\bar{e}^\top\bar{e})\,I_{n_v}, \quad
c_{\text{frame}} = -\bar{J}^\top \bar{e}
\f]

### Setters

- `setPositionCost(double)` / `setPositionCost(Eigen::Vector3d)`
- `setOrientationCost(double)` / `setOrientationCost(Eigen::Vector3d)`
- `setGain(double)`, `setLMDamping(double)`
- `setTarget(pinocchio::SE3)`, `setTargetFromConfiguration(Configuration)`

When no target is set the task is @b inert @b  — it contributes zero to the QP.

---

## PostureTask

@b Purpose: @b regulate all actuated joint angles toward a reference posture
\f$q_{\text{ref}}\f$.

### Error (n_v-D)

\f[
e = q_{\text{ref}} \ominus q_{\text{cur}}
\f]

where \f$\ominus\f$ is `pinocchio::difference` — the manifold-aware
subtraction that produces a tangent vector.  For revolute/prismatic joints this
reduces to \f$q_{\text{ref}} - q_{\text{cur}}\f$.

Floating-base root coordinates are excluded; only actuated joints contribute.

### Jacobian (n_v×n_v)

The posture Jacobian is the identity matrix (after excluding root DOFs):

\f[
J_{\text{posture}} = I_{n_v}
\f]

(More precisely, it is the identity on actuated DOFs and zero on root DOFs.)

### Weight

A single scalar cost \f$w_p\f$ is broadcast to all actuated joints:
\f$W = w_p\, I\f$.  Typical value: \f$10^{-3}\f$ — low priority
regularisation.

### QP contribution

\f[
H_{\text{posture}} = w_p^2\, I_{n_v}
  + \mu\,(\bar{e}^\top\bar{e})\,I_{n_v}, \quad
c_{\text{posture}} = -w_p^2\,\alpha\, e
\f]

When no target is set the task is @b inert @b .

---

## DampingTask

@b Purpose: @b penalise all joint velocities equally, acting as viscous damping
that brings the robot to rest.

### Error

\f[
e_{\text{damp}} = \mathbf{0} \in \mathbb{R}^{n_v}
\f]

The error is always zero — this task only contributes through its Jacobian.

### Jacobian

\f[
J_{\text{damp}} = I_{n_v}
\f]

### QP contribution

With scalar cost \f$w_d\f$ and zero error the LM term vanishes:

\f[
H_{\text{damp}} = w_d^2\, I_{n_v}, \qquad
c_{\text{damp}} = \mathbf{0}
\f]

This adds a quadratic penalty on \f$\|\Delta q\|^2\f$ — the solver will
minimise unnecessary motion.  Typical cost: \f$10^{-4}\f$.

No target needed; always active once constructed.

---

## How tasks compose

The QP Hessian and gradient are the element-wise sum of all task contributions:

\f[
P = H_{\text{frame}} + H_{\text{posture}} + H_{\text{damp}} + \lambda I, \quad
c = c_{\text{frame}} + c_{\text{posture}} + c_{\text{damp}}
\f]

Because `FrameTask` typically has cost \f$\gg\f$ `PostureTask` \f$\gg\f$
`DampingTask`, the hierarchy is:

1. Track the end-effector target (highest priority).
2. Stay near the reference posture (medium priority, regularisation).
3. Minimise overall motion (lowest priority, smoothing).

Adjusting costs shifts these priorities at runtime.
