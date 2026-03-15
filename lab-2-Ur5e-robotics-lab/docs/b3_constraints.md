# B3: Joint Constraints and Safety

## Goal

Add a safety layer that enforces joint limits, velocity limits, torque saturation, and self-collision avoidance before commands reach the UR5e.

## Files

- Script: `src/b3_constraints.py`
- Output CSVs:
  - `docs/b3_limit_guard.csv`
  - `docs/b3_collision_cases.csv`
  - `docs/b3_stress_test.csv`

## Implementation

### 1. Joint Limit Checking and Clamping

- `check_joint_limits(q)` -- returns `(all_within_limits, margin)` where margin is the minimum distance to either limit per joint. Positive margin means within limits.
- `clamp_joint_positions(q)` -- hard-clips joint angles to `[JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER]`.

### 2. Joint Limit Repulsion (`joint_limit_repulsion()`)

- Applies a spring-like repulsive torque when a joint enters the buffer zone near its limits.
- `buffer = 0.1 rad` default (~5.7 degrees).
- `gain = 5.0` default.
- When a joint is within `buffer` of a limit, a torque proportional to `gain * (buffer - distance)` pushes it away.
- This provides a smooth gradient rather than a hard stop.

### 3. Velocity Limit Scaling (`scale_velocity()`)

- If any joint velocity exceeds its limit (from `VELOCITY_LIMITS` in `ur5e_common.py`), the entire velocity vector is scaled down proportionally.
- This preserves the direction of motion while respecting all per-joint limits.
- `scale_delta_q()` applies the same logic to position increments by converting to implied velocity first.

### 4. Torque Saturation (`saturate_torques()`)

- Clips commanded torques to `[-TORQUE_LIMITS, +TORQUE_LIMITS]` per joint.
- UR5e limits: 150 Nm for joints 1-3, 28 Nm for joints 4-6.

### 5. Self-Collision Detection (`check_self_collision()`)

- A geometric heuristic using Pinocchio FK frame positions.
- Computes distances between all non-adjacent link frames (skipping adjacent links since they are always close).
- If any pair is closer than `min_dist = 0.05 m`, a collision is flagged.
- `self_collision_score(q)` returns the minimum inter-link distance as a scalar proximity metric.
- Note: this is a simplified heuristic. Production use should employ HPP-FCL collision checking through Pinocchio.

### 6. Safe Command Filter (`safe_command()`)

Combines all constraints into a single function that takes `(q, qd, tau, dt)` and returns `(safe_qd, safe_tau, info_dict)`. The pipeline is:

1. Scale velocity to respect limits.
2. Saturate torques.
3. Add joint limit repulsion torques.
4. Re-saturate after adding repulsion.
5. Check if the predicted next position `q + qd*dt` violates joint limits; if so, clamp and adjust velocity.

The `info_dict` reports which constraints were active: `vel_scaled`, `tau_saturated`, `repulsion_active`, `position_clamped`, `min_joint_margin`.

### Demo

The demo tests each constraint module independently:
- Joint limit checking on configs: `home`, `near_limit`, `beyond_limit`.
- Velocity scaling on within-limits and exceeding-limits velocity vectors.
- Torque saturation on a torque vector exceeding the UR5e limits.
- Self-collision detection on `home`, `folded`, and `extended` configurations.
- Joint limit repulsion on a configuration near the limits.
- The full safe command filter combining all constraints.

## How to Run

```bash
python3 src/b3_constraints.py
```

## What to Study

1. Understand the difference between hard clamping and smooth repulsion: clamping is discontinuous, repulsion provides a gradient that controllers can work with.
2. Trace `scale_velocity()`: proportional scaling preserves the joint-space direction, which is important for coordinated motion.
3. Read `check_self_collision()` with awareness that this is a heuristic based on frame distances, not actual mesh geometry.
4. Study the `safe_command()` pipeline: this is the pattern used in real robot deployments where multiple safety layers are stacked.

## Next Step

Move to C1 (Pick and Place Pipeline) to integrate IK, trajectory generation, control, and constraints into a complete demo.
