# Tuning Guide {#tuning_guide}

All built in IK parameters are stored in `torq::IKConfig` and have setters on
`torq::RobotSystem`. The same controls appear in the IK Tuning GUI panel, changes are applied at runtime.

## Defaults

| Parameter | Default | Setter | Notes |
|-----------|---------|--------|-------|
| Frame position cost     | 1.0    | `setFrameTaskPositionCost`     | Set 0 for orientation only tracking. |
| Frame orientation cost  | 1.0    | `setFrameTaskOrientationCost`  | Set 0 for position only tracking. |
| Frame gain (\f$\alpha\f$) | 1.0  | `setFrameTaskGain`             | 1.0 = dead beat; lower = smoother. |
| Frame LM damping (\f$\mu\f$) | 0.5 | `setFrameTaskLMDamping`     | Activates near singular or unreachable targets. |
| Posture cost            | 1e-3   | `setPostureTaskCost`           | Typically 2-3 orders below frame cost. |
| Posture gain            | 1.0    | `setPostureTaskGain`           | |
| Posture LM damping      | 0.5    | `setPostureTaskLMDamping`      | |
| Damping cost            | 1e-2   | `setDampingTaskCost`           | Higher = smoother motion. |
| Solver damping (\f$\lambda\f$) | 1e-12 | `setSolverDamping`     | Tikhonov term, rarely changed. |
| Configuration limit gain (\f$\gamma\f$) | 0.5 | `setConfigLimitGain` | Lower = earlier deceleration near joint limits. |

## Common scenarios

| Symptom | Try this |
|---------|----------|
| Oscillating near target  | Lower `frame_gain` (≈ 0.5–0.8); raise `damping_cost`; add `frame_lm_damping ≈ 0.01`. |
| Arm barely moves         | Lower `posture_cost` and `damping_cost`; raise `frame_position_cost`. |
| Awkward elbow / drift    | Set `setHomePosition()` and raise `posture_cost`. |
| Jerky motion             | Raise `damping_cost`; lower `frame_gain`. |
| Solver reports infeasible | Raise `config_limit_gain` toward 1.0; relax barrier gains. |
| Stops short of the target | Lower barrier gain or set `safe_displacement_gain = 0`. |

## How gains relate to convergence

With gain \f$\alpha\f$ the error decays approximately
\f$\|e_n\| \approx \|e_0\|(1-\alpha)^n\f$ over \f$n\f$ ticks. To reach 1%
of the initial error in \f$n\f$ ticks: \f$\alpha = 1 - 0.01^{1/n}\f$.

LM damping \f$\mu\f$ scales with the squared residual, so it only becomes effective
when the target is far or unreachable. Well tracked tasks pay zero cost.
