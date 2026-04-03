# Conventions and Notation {#conventions}

This page defines the mathematical notation, coordinate frame conventions,
and manifold operations used throughout Torq and its documentation.
Understanding these conventions is essential for correctly interpreting
task errors, Jacobians, and constraint formulations.

---

## Notation table

| Symbol | Name | Description |
|--------|------|-------------|
| \f$q \in \mathcal{Q}\f$ | Configuration | Full robot configuration vector (dimension \f$n_q\f$). Includes joint angles and, for floating-base robots, base position + quaternion. |
| \f$n_q\f$ | Configuration dimension | Size of the configuration vector \f$q\f$. For floating-base robots, \f$n_q \ne n_v\f$ because quaternions use 4 components but have 3 DOFs. |
| \f$n_v\f$ | Velocity dimension | Dimension of the tangent space at \f$q\f$. This is the number of degrees of freedom. |
| \f$\Delta q \in \mathbb{R}^{n_v}\f$ | Configuration displacement | Tangent vector representing a small configuration change. This is the QP decision variable. |
| \f$v = \Delta q / \Delta t\f$ | Velocity | Generalised velocity in the tangent space. |
| \f$\Delta t\f$ | Timestep | Integration timestep between consecutive IK solves. |
| \f$e_i(q) \in \mathbb{R}^{k_i}\f$ | Task error | Error vector for task \f$i\f$. Dimension \f$k_i\f$ depends on the task type (e.g. 6 for FrameTask, 3 for ComTask). |
| \f$J_i(q) \in \mathbb{R}^{k_i \times n_v}\f$ | Task Jacobian | Maps joint displacements to task-space changes for task \f$i\f$. |
| \f$w_i\f$ | Cost (weight) | Per-component weight vector determining relative task importance. |
| \f$\alpha_i \in (0, 1]\f$ | Task gain | Convergence rate. 1.0 = dead-beat (correct full error per tick). |
| \f$\mu_i \ge 0\f$ | LM damping | Levenberg-Marquardt damping. Regularises the task when the target is unreachable. |
| \f$\lambda\f$ | Solver damping | Global Tikhonov regularisation on the QP Hessian diagonal. |
| \f$\gamma\f$ | Configuration limit gain | Controls deceleration near joint limits. \f$\gamma \in (0, 1]\f$. |
| \f$P\f$ | QP Hessian | Positive semi-definite matrix in the QP objective. |
| \f$c\f$ | QP gradient | Linear term in the QP objective. |
| \f$G, h\f$ | Inequality constraints | \f$G\,\Delta q \le h\f$ — rows from limits and barriers. |
| \f$h(q)\f$ | Barrier function | Scalar (or vector) function where \f$h(q) \ge 0\f$ defines the safe region. |
| \f$T \in SE(3)\f$ | Rigid-body transform | A 4×4 homogeneous transformation matrix (rotation + translation). |
| \f$\log_6(\cdot)\f$ | SE(3) logarithmic map | Maps a relative transform \f$T\f$ to a 6-D twist in the tangent space \f$\mathfrak{se}(3)\f$. |

---

## Coordinate frames

Torq uses the following coordinate frame conventions, inherited from
Pinocchio and MuJoCo:

### World frame

The **world frame** is the fixed, global reference frame. All absolute
positions and orientations are expressed relative to this frame. Gravity
acts along the negative Z axis by default.

### Body (local) frame

Each rigid body has a **local frame** attached to it. Frame transforms
\f$T_{\text{frame}}^{\text{world}}\f$ give the pose of a body frame in
world coordinates.

### Jacobian reference frames

Torq uses the **LOCAL** reference frame for frame Jacobians, consistent
with the logarithmic map error formulation:

- **LOCAL** (body frame): The Jacobian maps joint velocities to a twist
  expressed in the body's own frame. This is the natural choice when the
  error is computed via \f$\log_6(T_{\text{cur}}^{-1} T_{\text{des}})\f$
  because the logarithm produces a body-frame twist.

- **LOCAL_WORLD_ALIGNED**: Used internally for some barrier computations
  (e.g. `SelfCollisionBarrier`) where the geometric quantities are
  naturally expressed in world-aligned coordinates.

### SE(3) transform composition

Transforms compose by matrix multiplication. If \f$T_A^W\f$ is the pose
of frame A in world, and \f$T_B^A\f$ is the pose of frame B in frame A:

\f[
T_B^W = T_A^W \cdot T_B^A
\f]

- **Left-multiplication** applies a transform in the **world** frame.
- **Right-multiplication** applies a transform in the **local** frame.

---

## Manifold operations

Robot configuration spaces are not always Euclidean. Floating-base robots
use quaternions for orientation, which live on \f$S^3\f$. Torq handles
this using Pinocchio's manifold-aware operations.

### Difference (\f$\ominus\f$)

The manifold difference computes a tangent vector between two configurations:

\f[
\delta = q_2 \ominus q_1 = \text{pinocchio::difference}(\text{model}, q_1, q_2)
\f]

For revolute joints, this reduces to \f$q_2 - q_1\f$. For quaternion
joints, it computes the logarithmic map of the relative rotation. The
result is always a vector in \f$\mathbb{R}^{n_v}\f$.

### Integration (\f$\oplus\f$)

Manifold integration applies a tangent vector to a configuration:

\f[
q_{\text{new}} = q \oplus \Delta q = \text{pinocchio::integrate}(\text{model}, q, \Delta q)
\f]

For revolute/prismatic joints, \f$\oplus\f$ reduces to ordinary addition.
For floating-base quaternions, it uses proper quaternion exponential
integration, avoiding gimbal lock and maintaining unit-norm constraints.

### Why this matters

The QP solves for \f$\Delta q \in \mathbb{R}^{n_v}\f$ (tangent space).
After the solve, manifold integration maps this back to a valid
configuration:

\f[
q_{\text{new}} = q_{\text{current}} \oplus \Delta q
\f]

This is why \f$n_q\f$ (configuration dimension) can differ from \f$n_v\f$
(velocity/tangent dimension): a free-floating base has \f$n_q = 7\f$
(position + quaternion) but \f$n_v = 6\f$ (linear + angular velocity).

---

## Task convention

Every task implements the first-order dynamics:

\f[
J_i(q)\,\Delta q = -\alpha_i\,e_i(q)
\f]

The error \f$e_i(q)\f$ is defined so that it is **zero at the target**
and the system drives \f$e \to 0\f$. The gain \f$\alpha\f$ controls the
convergence rate: with \f$\alpha = 1\f$ the full error is corrected each
tick (dead-beat); with \f$\alpha < 1\f$ the error decays exponentially:

\f[
\|e_n\| \approx \|e_0\| \cdot (1 - \alpha)^n
\f]

---

## Limit convention

Limits produce inequality constraints of the form:

\f[
G\,\Delta q \le h
\f]

These are **hard constraints** — the QP solver will never return a
\f$\Delta q\f$ that violates them. If no feasible solution exists, the
solver reports infeasibility.

---

## Barrier convention

Barriers enforce safety via Control Barrier Functions. A barrier function
\f$h(q) \ge 0\f$ defines the safe region. The CBF condition:

\f[
\frac{\partial h}{\partial q} \dot{q} + \alpha\,h(q) \ge 0
\f]

guarantees forward invariance of the safe set. In the QP, this becomes
an inequality constraint (rows in \f$G, h\f$) and optionally a
safe-displacement objective term.

---

## Units

| Quantity | Unit |
|----------|------|
| Joint angles | radians |
| Positions | metres |
| Velocities | rad/s or m/s |
| Accelerations | rad/s\f$^2\f$ or m/s\f$^2\f$ |
| Timestep | seconds |
| Costs/weights | dimensionless (relative) |
| Gains | dimensionless, \f$\in (0, 1]\f$ |
| Distances | metres |
