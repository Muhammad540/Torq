# Conventions {#conventions}

## Notation

| Symbol | Meaning |
|--------|---------|
| \f$q \in \mathcal{Q}\f$ | Configuration (size \f$n_q\f$). Includes joint angles + floating base position/quaternion. |
| \f$n_q\f$ | Configuration dimension. |
| \f$n_v\f$ | Tangent / velocity dimension (number of DOFs).|
| \f$\Delta q \in \mathbb{R}^{n_v}\f$ | Configuration displacement, the QP decision variable. |
| \f$v = \Delta q / \Delta t\f$ | Generalised velocity. |
| \f$e_i, J_i, w_i\f$ | Task error / Jacobian / weight. |
| \f$\alpha, \mu, \lambda, \gamma, \kappa\f$ | Task gain, LM damping, Tikhonov damping, configuration limit gain, safe displacement gain. |
| \f$T \in SE(3)\f$ | Rigid body transform (4×4 homogeneous matrix). |
| \f$\log_6\f$ | SE(3) logarithmic map: relative transform → 6-D twist. |
| \f$\oplus, \ominus\f$ | Manifold integration / difference (`pinocchio::integrate`, `pinocchio::difference`). |

## Frames

- @b World @b frame: fixed global reference; gravity along \f$-z\f$.
- @b Body @b frame: attached to each rigid body; what `getFramePose` returns.
- @b Frame @b Jacobians use the LOCAL reference frame

## Manifold integration

The QP works in the tangent space \f$\mathbb{R}^{n_v}\f$. After each solve:

\f[
q_\text{new} = q \oplus \Delta q
\f]

## Units

| Quantity | Unit |
|----------|------|
| Joint angles | rad |
| Positions    | m |
| Velocities   | rad/s, m/s |
| Timestep     | s |
| Costs / gains | dimensionless |
