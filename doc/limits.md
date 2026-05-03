# Limits {#limits_page}

A **limit** contributes inequality rows \f$G\,\Delta q \le h\f$ to the QP.
Limits are @b hard and the solver will never return a \f$\Delta q\f$ that
violates them. If your custom limit has nothing to enforce on a given tick,
return `std::nullopt` and it costs the QP nothing.

| Class | Built-in? | Purpose |
|-------|-----------|---------|
| @ref torq::VelocityLimit "VelocityLimit"           | Yes | Joint velocity bounds |
| @ref torq::ConfigurationLimit "ConfigurationLimit" | Yes | Joint position bounds with smooth deceleration |

---

## VelocityLimit {#velocitylimit}

Bound \f$\Delta q\f$ so the implied velocity respects the model's
`velocityLimit` vector:

\f[
-\Delta t\;\dot q_\text{max}\; \le \; \Delta q \; \le \; \Delta t\;\dot q_\text{max}
\f]

Joints whose `velocityLimit` is non finite in the URDF/MJCF are skipped.

To set them if the Pinocchio parser leaves them unbounded (specifically for MJCF), assign `RobotConfig::joint_velocity_limit_rad_s` before `RobotSystem::initialize()`; it fills every DoF of the reduced model's `velocityLimit`.

---

## ConfigurationLimit {#configurationlimit}

Keep \f$q + \Delta q\f$ inside \f$[q_\text{min}, q_\text{max}]\f$, with a
proportional gain \f$\gamma \in (0, 1]\f$ that decelerates smoothly as the
robot approaches a limit:

\f[
\Delta q_j \;\le\;  \gamma\,(q_j^\text{max} \ominus q_j),
\qquad
-\Delta q_j \;\le\;  \gamma\,(q_j \ominus q_j^\text{min})
\f]

| `config_limit_gain` \f$\gamma\f$ | Behaviour |
|----------------------------------|-----------|
| 1.0 | Use the full remaining range; only stops at the boundary. |
| 0.5 (default) | Allow half the remaining range per tick, smooth deceleration. |
| 0.1 | Conservative; slows down well before the limit. |

---

See @ref extending_page for writing your own `Limit` subclass.
