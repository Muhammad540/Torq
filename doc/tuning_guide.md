# Parameter Tuning Guide {#tuning_guide}

This page is a practical reference for tuning the IK parameters exposed by
`torq::IKConfig` and the runtime setters on `torq::RobotSystem`. See
@ref conventions for notation and @ref qp_formulation for how these
parameters affect the QP.

---

## Quick reference

### Built-in task parameters

| Parameter | Default | Typical range | Setter |
|-----------|---------|---------------|--------|
| Frame position cost | 1.0 | 0.1 – 10 | `setFrameTaskPositionCost()` |
| Frame orientation cost | 1.0 | 0.1 – 10 | `setFrameTaskOrientationCost()` |
| Frame gain (\f$\alpha\f$) | 1.0 | 0.1 – 1.0 | `setFrameTaskGain()` |
| Frame LM damping (\f$\mu\f$) | 0.0 | 0 – 1.0 | `setFrameTaskLMDamping()` |
| Posture cost | 1e-3 | 1e-4 – 1e-1 | `setPostureTaskCost()` |
| Posture gain | 1.0 | 0.1 – 1.0 | `setPostureTaskGain()` |
| Posture LM damping | 0.0 | 0 – 1.0 | `setPostureTaskLMDamping()` |
| Damping cost | 1e-4 | 1e-6 – 1e-2 | `setDampingTaskCost()` |

### Solver and limit parameters

| Parameter | Default | Typical range | Setter |
|-----------|---------|---------------|--------|
| Solver (Tikhonov) damping (\f$\lambda\f$) | 1e-12 | 1e-14 – 1e-6 | `setSolverDamping()` |
| Configuration limit gain (\f$\gamma\f$) | 0.5 | 0.1 – 1.0 | `setConfigLimitGain()` |

### Barrier parameters (set per-barrier on the object)

| Parameter | Typical range | Setter |
|-----------|---------------|--------|
| Barrier gain (\f$\alpha\f$) | 0.5 – 5.0 | Constructor or `Barrier::gain()` |
| Safe displacement gain (\f$\kappa\f$) | 0.0 – 5.0 | Constructor |
| Minimum distance (\f$d_{\min}\f$) | 0.01 – 0.10 m | Constructor |

---

## The IKConfig struct

All built-in parameters are stored in `torq::IKConfig` (defined in
`torq/Controller.hpp`) with sensible defaults. The struct is owned by the
`Controller`; `RobotSystem` provides pass-through setters.

```cpp
struct IKConfig {
    double frame_position_cost   = 1.0;
    double frame_orientation_cost = 1.0;
    double frame_gain            = 1.0;
    double frame_lm_damping      = 0.0;
    double posture_cost          = 1e-3;
    double posture_gain          = 1.0;
    double posture_lm_damping    = 0.0;
    double damping_cost          = 1e-4;
    double solver_damping        = 1e-12;
    double config_limit_gain     = 0.5;
};
```

---

## Parameter details

### Frame task costs

@b frame_position_cost and @b frame_orientation_cost control how
aggressively the solver tracks the Cartesian target.

- Increase position cost to prioritise reaching the exact position (even if
  orientation drifts).
- Increase orientation cost to prioritise correct end-effector alignment.
- Set one to 0 to disable that component entirely (e.g. position-only tracking).

### Frame gain (\f$\alpha\f$)

Controls convergence speed. After \f$n\f$ ticks the error is approximately
\f$\|e_n\| \approx \|e_0\|(1 - \alpha)^n\f$.

| Value | Behaviour |
|-------|-----------|
| 1.0 | Dead-beat: correct the full error in one tick. Fastest but may overshoot. |
| 0.5 | Correct half the error per tick. Smoother trajectory. |
| 0.1 | Very slow convergence. Useful for teleoperation or gentle motions. |

To reach 1% of the initial error in \f$n\f$ frames:
\f$\alpha = 1 - 0.01^{1/n}\f$.

### Frame LM damping (\f$\mu\f$)

Levenberg-Marquardt regularisation for the frame task:

- @b 0.0 — no damping, fastest convergence. May produce large velocities
  when the target is far or near a singularity.
- @b 0.01 – 0.1 — moderate damping. Good for interactive use.
- @b > 0.5 — heavy damping. The arm will move slowly but safely toward
  unreachable targets.

The effective damping scales with the squared residual
(\f$\mu \cdot \|We\|^2\f$), so it only activates when the error is non-trivial.

### Posture cost

Weight of the posture regularisation task. Typical values are 2–3 orders of
magnitude below the frame cost. If set too high, the arm will resist moving
from its reference posture; if too low, it may adopt awkward configurations.

### Posture gain and LM damping

Same semantics as the frame task variants, but applied to the posture
regularisation. Usually left at defaults (gain=1, lm_damping=0).

### Damping cost

Weight of the `DampingTask` which penalises all joint velocity. Increase this
to smooth out jerky motions; decrease to allow faster response.

### Solver (Tikhonov) damping (\f$\lambda\f$)

A tiny constant added to the QP Hessian diagonal to ensure positive
definiteness. Should almost never need changing. Increase to \f$10^{-6}\f$ or
higher only if the solver reports numerical issues.

### Configuration limit gain (\f$\gamma\f$)

Controls how early the robot starts decelerating as it approaches joint limits:

| Value | Behaviour |
|-------|-----------|
| 1.0 | Only stop at the exact limit. May cause hard stops. |
| 0.5 | Decelerate when halfway to the limit. Smooth default. |
| 0.1 | Very conservative — slow down far from limits. |

---

## Barrier tuning

Barriers have their own parameters, set on the barrier object at construction
time. See @ref barriers_page for details.

### Barrier gain (\f$\alpha\f$)

Controls how aggressively the CBF condition is enforced.

| Value | Behaviour |
|-------|-----------|
| 0.5 | Relaxed — allows the robot to get closer to the boundary. |
| 1.0 | Standard enforcement. |
| 3.0+ | Very strict — keeps the robot far from the boundary. |

### Safe displacement gain (\f$\kappa\f$)

Controls the "push away" objective:

| Value | Behaviour |
|-------|-----------|
| 0.0 | Constraint only. No active avoidance. |
| 0.5–1.0 | Moderate active avoidance. |
| 3.0+ | Strong active avoidance (default for `BodySphericalBarrier`). |

### Minimum distance (\f$d_{\min}\f$)

For distance-based barriers (`BodySphericalBarrier`, `SelfCollisionBarrier`):

- Start conservative (e.g. 5+ cm) to account for mesh approximation error.
- Reduce after verifying safe behaviour.

---

## Tuning recipes

### Robot oscillates near target

The frame gain is too high or the frame task cost is too large relative to the
damping cost.

1. Lower `frame_gain` to 0.5 – 0.8.
2. Increase `damping_cost` by 10x.
3. Add a small `frame_lm_damping` (e.g. 0.01).

### Arm barely moves

The posture or damping cost is too high relative to the frame cost.

1. Decrease `posture_cost` (e.g. from 1e-3 to 1e-4).
2. Decrease `damping_cost` (e.g. from 1e-4 to 1e-6).
3. Increase `frame_position_cost`.

### Weird elbow configurations

The posture task is too weak or has no useful reference.

1. Set a sensible home posture via `setHomePosition()`.
2. Increase `posture_cost` to 1e-2.

### Jerky motions

1. Increase `damping_cost`.
2. Lower `frame_gain`.
3. Add a `LowAccelerationTask` via `addTask()` for smooth velocity profiles.
4. Add an `AccelerationLimit` via `addLimit()` for hard acceleration bounds.

### Solver reports infeasible

The constraints (limits + barriers) are contradictory or the target forces
violations.

1. Increase `config_limit_gain` toward 1.0 (less conservative limits).
2. Add LM damping so the solver gracefully handles unreachable targets.
3. Check that the velocity limits in the URDF are not too restrictive.
4. Reduce barrier gains or increase \f$d_{\min}\f$ to relax barrier constraints.

### Robot stops before reaching target (barrier too aggressive)

The barrier gain or safe displacement gain is too high.

1. Lower the barrier gain \f$\alpha\f$.
2. Set `safe_displacement_gain` to 0 (constraint only, no active avoidance).
3. Verify the barrier bounds are correct and not overly restrictive.

---

## GUI: IK Tuning panel

All built-in parameters above are exposed in the built-in ImGui @b IK Tuning
panel. The panel appears when the IK subsystem is active (after the first
Cartesian jog command).

- Sliders and drag-float inputs for every parameter.
- Changes take effect immediately — no restart needed.
- Solver damping uses a logarithmic slider due to its tiny scale.

Barrier parameters are @b not currently exposed in the GUI — they are set
programmatically at construction time. A future release may add barrier
tuning to the GUI panel.
