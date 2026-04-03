# Tasks {#tasks_page}

Tasks define the @b objective @b (cost) of the QP. Each task contributes
a Hessian block \f$H_i\f$ and a gradient vector \f$c_i\f$ to the
quadratic program. See @ref qp_formulation for how tasks compose
and @ref conventions for notation.

Torq provides **8 concrete task types**. Three are built into every
`RobotSystem` (FrameTask, PostureTask, DampingTask); the others are added
via `RobotSystem::addTask()`.

---

## Task overview

| Task | Dimension | Purpose | Built-in? |
|------|-----------|---------|-----------|
| @ref frametask "FrameTask" | 6 | Track an SE(3) end-effector pose | Yes |
| @ref posturetask "PostureTask" | \f$n_v\f$ | Regularise toward a reference posture | Yes |
| @ref dampingtask "DampingTask" | \f$n_v\f$ | Penalise all joint velocities | Yes |
| @ref comtask "ComTask" | 3 | Regulate centre-of-mass position | No |
| @ref relativeframetask "RelativeFrameTask" | 6 | Track a frame's pose relative to another frame | No |
| @ref linearholonomictask "LinearHolonomicTask" | \f$p\f$ | Linear constraint \f$A(q_0 \ominus q) = b\f$ | No |
| @ref jointcouplingtask "JointCouplingTask" | 1 | Couple joint angles via ratios | No |
| @ref jointvelocitytask "JointVelocityTask" | \f$n_v\f$ | Track reference joint velocities | No |
| @ref lowaccelerationtask "LowAccelerationTask" | \f$n_v\f$ | Minimise joint accelerations | No |

---

## Common parameters {#task_common_params}

Every task inherits from `torq::Task` and exposes:

| Parameter | Symbol | Range | Effect |
|-----------|--------|-------|--------|
| @b cost @b (weight) | \f$w\f$ | \f$(0, \infty)\f$ | Relative importance. Higher = prioritised. Can be scalar (broadcast) or per-component vector. |
| @b gain @b | \f$\alpha\f$ | \f$(0, 1]\f$ | Convergence speed. 1.0 = dead-beat. Lower values smooth the response. |
| @b lm_damping @b | \f$\mu\f$ | \f$[0, \infty)\f$ | Levenberg-Marquardt regularisation. Increase when the target is unreachable to avoid velocity spikes. |

All three are adjustable at runtime via setters on the task and via
`RobotSystem`'s IK tuning API.

### Generic QP contribution {#task_qp_generic}

Given a task with error \f$e \in \mathbb{R}^k\f$, Jacobian
\f$J \in \mathbb{R}^{k \times n_v}\f$, weight vector
\f$w \in \mathbb{R}^k\f$, gain \f$\alpha\f$, and LM damping \f$\mu\f$:

**Step 1 — Weight the quantities:**

\f[
W = \operatorname{diag}(w), \quad
\bar{J} = W J, \quad
\bar{e} = W(-\alpha\, e)
\f]

**Step 2 — Compute effective LM damping:**

\f[
\mu^{\text{eff}} = \mu \cdot \|\bar{e}\|^2
\f]

The damping is proportional to the residual, so it only activates when the
error is non-trivial and the target may be unreachable.

**Step 3 — Form the Hessian and gradient:**

\f[
H = \bar{J}^\top \bar{J} + \mu^{\text{eff}}\, I_{n_v}, \qquad
c = -\bar{J}^\top \bar{e}
\f]

This corresponds to the weighted least-squares objective:

\f[
\min_{\Delta q}\; \tfrac{1}{2}\left\|\bar{J}\,\Delta q - \bar{e}\right\|^2
+ \tfrac{1}{2}\,\mu^{\text{eff}}\,\|\Delta q\|^2
\f]

This formulation is shared by all tasks via `Task::computeQPObjective()`.
Each concrete task only needs to implement `computeError()` and
`computeJacobian()`.

---

## FrameTask {#frametask}

@b Purpose: Regulate the SE(3) pose of a named robot frame (typically the
end-effector) to a desired target \f$T_{\text{des}} \in SE(3)\f$.

### Error (6-D)

The error is computed using the @b logarithmic map of the relative
transform:

\f[
e = \log_6\!\left(T_{\text{cur}}^{-1}\; T_{\text{des}}\right)
  = \begin{bmatrix} e_{\text{pos}} \\ e_{\text{ori}} \end{bmatrix}
  \in \mathbb{R}^6
\f]

where \f$\log_6 : SE(3) \to \mathfrak{se}(3)\f$ maps the relative
transform into a twist in the **body** frame. This is manifold-aware:
it correctly handles the topology of rotations without gimbal lock.

**Implementation** (`Tasks.cpp`):
```cpp
pinocchio::SE3 T_frame = config.getTransformFrameToWorld(frame_);
pinocchio::SE3 T_target_to_frame = T_frame.actInv(target_.value());
return pinocchio::log6(T_target_to_frame).toVector();
```

### Jacobian (6 x \f$n_v\f$)

The Jacobian accounts for the non-linearity of the SE(3) logarithm using
the left Jacobian of the log map:

\f[
J = -J_{\log_6}\!\left(T_{\text{frame} \to \text{target}}\right) \cdot
    {}^{\text{local}}J_{\text{frame}}(q)
\f]

where \f$J_{\log_6}\f$ is the derivative of \f$\log_6\f$ (from
`pinocchio::Jlog6`) and the frame Jacobian is computed in the LOCAL
reference frame for consistency with the body-frame error.

**Implementation** (`Tasks.cpp`):
```cpp
pinocchio::SE3 T_frame_to_target = target_.value().actInv(T_frame);
pinocchio::Jlog6(T_frame_to_target, Jlog);
Eigen::MatrixXd J_frame = config.getFrameJacobian(frame_);
return -Jlog * J_frame;
```

### Weight vector

The 6-D cost vector is \f$w = [w_{\text{pos}}^{(3)},\; w_{\text{ori}}^{(3)}]\f$.
The first three components weight translational error and the last three
weight rotational error. This allows independent prioritisation:

- Position-only tracking: set `setOrientationCost(0.0)`.
- Orientation-only tracking: set `setPositionCost(0.0)`.

### QP contribution

\f[
H_{\text{frame}} = \bar{J}^\top \bar{J}
  + \mu\,(\bar{e}^\top\bar{e})\,I_{n_v}, \quad
c_{\text{frame}} = -\bar{J}^\top \bar{e}
\f]

### Setters

- `setPositionCost(double)` / `setPositionCost(Eigen::Vector3d)` — per-axis position weight.
- `setOrientationCost(double)` / `setOrientationCost(Eigen::Vector3d)` — per-axis orientation weight.
- `setGain(double)` — convergence gain \f$\alpha\f$.
- `setLMDamping(double)` — LM regularisation \f$\mu\f$.
- `setTarget(pinocchio::SE3)` — set the desired pose.
- `setTargetFromConfiguration(Configuration)` — capture current pose as target.

When no target is set the task is @b inert — it contributes zero to the QP.

---

## PostureTask {#posturetask}

@b Purpose: Regularise all actuated joint angles toward a reference posture
\f$q_{\text{ref}}\f$. Prevents nullspace drift and provides a well-defined
arm configuration when the primary task leaves redundant DOFs
unconstrained.

### Error (\f$n_v\f$-D)

\f[
e = q_{\text{ref}} \ominus q_{\text{cur}}
  = \text{pinocchio::difference}(\text{model}, q_{\text{ref}}, q_{\text{cur}})
\f]

For revolute/prismatic joints this reduces to \f$q_{\text{ref}} - q_{\text{cur}}\f$.
For quaternion joints it uses the manifold-aware logarithmic map.

Floating-base root coordinates are **excluded**; only actuated joints
contribute.

### Jacobian (\f$n_v \times n_v\f$)

\f[
J_{\text{posture}} = I_{n_v}
\f]

(Identity on actuated DOFs, zero on root DOFs for floating-base robots.)

### QP contribution

With scalar cost \f$w_p\f$:

\f[
H_{\text{posture}} = w_p^2\, I_{n_v}
  + \mu\,(\bar{e}^\top\bar{e})\,I_{n_v}, \quad
c_{\text{posture}} = -w_p^2\,\alpha\, e
\f]

### Velocity interpretation

The posture task behaves as a joint-space proportional controller:

\f[
\dot{q}_{\text{des}} = \frac{\alpha}{\Delta t}(q_{\text{ref}} - q)
= k_p(q_{\text{ref}} - q)
\f]

This is always full-rank and provides robust regularisation when the
primary task Jacobian is ill-conditioned.

### Setters

- `setTarget(Eigen::VectorXd)` — set the reference posture (full \f$n_q\f$ vector).
- `setTargetFromConfiguration(Configuration)` — capture current \f$q\f$ as reference.

When no target is set the task is @b inert.

---

## DampingTask {#dampingtask}

@b Purpose: Penalise all joint velocities equally, acting as viscous
damping that brings the robot to rest when no other task demands motion.

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

With scalar cost \f$w_d\f$ and zero error, the LM term vanishes:

\f[
H_{\text{damp}} = w_d^2\, I_{n_v}, \qquad
c_{\text{damp}} = \mathbf{0}
\f]

This adds a quadratic penalty on \f$\|\Delta q\|^2\f$. The solver will
minimise unnecessary motion.

### Connection to Tikhonov regularisation

When a primary task Jacobian \f$J_h\f$ becomes rank-deficient near
a singularity, adding the damping task makes the combined Hessian:

\f[
H = J_h^\top J_h + w_d^2\, I
\f]

This is exactly Tikhonov regularisation — it bounds joint velocities near
singularities and selects the minimum-norm solution.

No target needed; always active once constructed.

---

## ComTask {#comtask}

@b Purpose: Regulate the centre-of-mass (CoM) position. Critical for
humanoid balance and quadruped stability.

### Error (3-D)

\f[
e = \text{com}_{\text{actual}}(q) - \text{com}_{\text{target}}
\f]

### Jacobian (3 x \f$n_v\f$)

\f[
J_{\text{com}} = \frac{\partial \text{com}(q)}{\partial q}
\f]

Computed by `pinocchio::jacobianCenterOfMass`.

### QP contribution

Standard @ref task_qp_generic "generic task QP" with \f$k = 3\f$.

### Setters

- `setTarget(Eigen::Vector3d)` — desired CoM position in world frame.
- `setTargetFromConfiguration(Configuration)` — capture current CoM.

When no target is set the task is @b inert.

### Example

```cpp
auto com_task = std::make_unique<torq::ComTask>(1.0);
com_task->setTarget(Eigen::Vector3d(0.0, 0.0, 0.5));
robot.addTask(std::move(com_task));
```

---

## RelativeFrameTask {#relativeframetask}

@b Purpose: Regulate the SE(3) pose of one frame @b relative to another
frame. Essential for specifying foot poses relative to the pelvis in
legged robots, or hand poses relative to the torso.

### Error (6-D)

\f[
e = \log_6\!\left(T_{\text{target} \to \text{root}}^{-1}\;
    T_{\text{frame} \to \text{root}}\right) \in \mathbb{R}^6
\f]

where \f$T_{\text{frame} \to \text{root}}\f$ is the pose of the
controlled frame in the reference frame's coordinates.

### Jacobian (6 x \f$n_v\f$)

\f[
J = J_{\log_6}(T_{ft}) \left(J_f - \text{Ad}_{T_{f \to r}}\, J_r\right)
\f]

where:
- \f$J_f\f$ is the frame Jacobian of the controlled frame.
- \f$J_r\f$ is the frame Jacobian of the root/reference frame.
- \f$\text{Ad}_{T_{f \to r}}\f$ is the adjoint representation that maps
  twists between frames.

This correctly accounts for the motion of both frames.

### QP contribution

Standard @ref task_qp_generic "generic task QP" with \f$k = 6\f$.
The 6-D cost vector splits into position and orientation components
just like `FrameTask`.

### Setters

- `setTarget(pinocchio::SE3)` — desired transform of frame in root's coordinates.
- `setTargetFromConfiguration(Configuration)` — capture current relative pose.
- `setPositionCost(double)` / `setOrientationCost(double)` — weight components.

When no target is set the task is @b inert.

### Example

```cpp
auto foot_task = std::make_unique<torq::RelativeFrameTask>(
    "left_foot", "pelvis", 1.0, 1.0, 0.01);

foot_task->setTargetFromConfiguration(robot.configuration());

robot.addTask(std::move(foot_task));
```

---

## LinearHolonomicTask {#linearholonomictask}

@b Purpose: Enforce a general linear relationship between configuration
variables: \f$A(q_0 \ominus q) = b\f$. This is the base class for
`JointCouplingTask` and can express arbitrary linear constraints on the
tangent space.

### Error (\f$p\f$-D)

\f[
e = A \cdot \text{difference}(q_0, q) - b
\f]

where \f$q_0\f$ is a reference configuration and
\f$\text{difference}(q_0, q)\f$ is the manifold-aware subtraction
(tangent vector from \f$q_0\f$ to \f$q\f$).

### Jacobian (\f$p \times n_v\f$)

\f[
J = A \cdot \frac{\partial}{\partial q}\text{difference}(q_0, q)
\f]

The partial derivative is computed by `pinocchio::dDifference` with
`ARG1`, giving the Jacobian of the difference operation with respect
to its second argument (the current configuration).

### QP contribution

Standard @ref task_qp_generic "generic task QP" with \f$k = p\f$.

### Example

```cpp
// Constraint: joints 2 and 4 should maintain a fixed angular difference
Eigen::MatrixXd A = Eigen::MatrixXd::Zero(1, robot.nv());
A(0, 2) = 1.0;
A(0, 4) = -1.0;

Eigen::VectorXd b = Eigen::VectorXd::Constant(1, 0.5);  // 0.5 rad difference
Eigen::VectorXd q0 = pinocchio::neutral(robot.model());

auto task = std::make_unique<torq::LinearHolonomicTask>(A, b, q0, 10.0);
robot.addTask(std::move(task));
```

---

## JointCouplingTask {#jointcouplingtask}

@b Purpose: Enforce a coupling constraint between revolute joints:
\f$\sum_{j \in J} r_j \, q_j = 0\f$. A special case of
`LinearHolonomicTask` with a single constraint row.

### Formulation

Given joint names \f$\{j_1, \ldots, j_m\}\f$ with ratios
\f$\{r_1, \ldots, r_m\}\f$, the constraint matrix is:

\f[
A = \begin{bmatrix} 0 & \cdots & r_1 & \cdots & r_m & \cdots & 0 \end{bmatrix}
\f]

where each \f$r_j\f$ is placed at the velocity index of the corresponding
joint. The reference configuration is the neutral configuration and
\f$b = 0\f$.

### Use cases

- **Symmetric knees**: Set \f$r_1 = 1, r_2 = -1\f$ to couple two knee
  joints to the same angle.
- **Gear ratio**: Set \f$r_1 = 1, r_2 = -N\f$ to enforce a gear ratio
  between two joints.
- **Mirror motion**: Couple left and right limbs for symmetric gaits.

### Example

```cpp
auto coupling = std::make_unique<torq::JointCouplingTask>(
    std::vector<std::string>{"left_knee", "right_knee"},
    std::vector<double>{1.0, -1.0},
    10.0,  // high cost for tight coupling
    robot.configuration()
);
robot.addTask(std::move(coupling));
```

---

## JointVelocityTask {#jointvelocitytask}

@b Purpose: Track reference joint velocities. Useful for velocity-controlled
locomotion layers that output desired joint velocities which the IK solver
should follow while respecting other constraints.

### Error (\f$n_v\f$-D)

\f[
e = v_{\text{ref}} \cdot \Delta t
\f]

The error is set to the target displacement \f$\Delta q = v \cdot dt\f$.
With gain \f$\alpha = 1\f$ (default), the task drives
\f$\Delta q \to v_{\text{ref}} \cdot dt\f$.

### Jacobian (\f$n_v \times n_v\f$)

\f[
J = I_{n_v}
\f]

(Identity on actuated joints, excludes floating-base root.)

### QP contribution

Standard @ref task_qp_generic "generic task QP" with \f$k = n_v\f$.
With the identity Jacobian and \f$e = v_{\text{ref}} \cdot dt\f$:

\f[
H = w^2 I_{n_v}, \quad c = -w^2 (v_{\text{ref}} \cdot dt)
\f]

### Setters

- `setTarget(Eigen::VectorXd v, double dt)` — set the reference velocity
  and timestep. Internally stores \f$\Delta q = v \cdot dt\f$.

When no target is set the task is @b inert.

### Example

```cpp
auto vel_task = std::make_unique<torq::JointVelocityTask>(0.5);

Eigen::VectorXd v_ref = Eigen::VectorXd::Zero(robot.nv());
v_ref(2) = 0.5;  // Joint 2 at 0.5 rad/s

vel_task->setTarget(v_ref, dt);
robot.addTask(std::move(vel_task));
```

---

## LowAccelerationTask {#lowaccelerationtask}

@b Purpose: Minimise joint accelerations by penalising changes in velocity
between consecutive ticks. Smooths motion and reduces jerk. Pair with
`DampingTask` for best results.

### Error (\f$n_v\f$-D)

\f[
e = -\Delta q_{\text{prev}}
\f]

By setting the error to the negative of the previous displacement, the
task drives \f$\Delta q \to \Delta q_{\text{prev}}\f$ (i.e. constant
velocity), which minimises acceleration.

### Jacobian (\f$n_v \times n_v\f$)

\f[
J = I_{n_v}
\f]

### QP contribution

With gain \f$\alpha = 1\f$ (default), \f$\bar{e} = W \Delta q_{\text{prev}}\f$:

\f[
H = w^2 I_{n_v}, \quad c = -w^2 \Delta q_{\text{prev}}
\f]

The solver balances maintaining the previous velocity (low acceleration)
against satisfying other tasks.

### Setters

- `setLastIntegration(Eigen::VectorXd v_prev, double dt)` — must be called
  each tick with the velocity from the previous IK solve.

### Example

```cpp
auto smooth_task = std::make_unique<torq::LowAccelerationTask>(0.1);
robot.addTask(std::move(smooth_task));

// In the control loop, after each solve:
smooth_task->setLastIntegration(velocity, dt);
```

---

## How tasks compose

The QP Hessian and gradient are the element-wise sum of all task
contributions (plus barrier objective terms):

\f[
P = \lambda I + \sum_{i \in \text{tasks}} H_i
  + \sum_{j \in \text{barriers}} H_j, \quad
c = \sum_{i \in \text{tasks}} c_i
  + \sum_{j \in \text{barriers}} c_j
\f]

where \f$\lambda\f$ is the Tikhonov solver damping.

Because the built-in tasks typically have cost
\f$w_{\text{frame}} \gg w_{\text{posture}} \gg w_{\text{damping}}\f$,
the hierarchy is:

1. **FrameTask** — track the end-effector target (highest priority).
2. **PostureTask** — stay near the reference posture (medium, regularisation).
3. **DampingTask** — minimise overall motion (lowest, smoothing).

User-added tasks participate in the same sum. Adjusting costs shifts
priorities at runtime.
