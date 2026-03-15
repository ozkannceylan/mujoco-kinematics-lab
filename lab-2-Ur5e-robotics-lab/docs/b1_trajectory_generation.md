# B1: Trajectory Generation

## Goal

Generate smooth joint-space and Cartesian-space reference trajectories for the UR5e, with feasibility checking against velocity limits.

## Files

- Script: `src/b1_trajectory_generation.py`
- Output CSVs:
  - `docs/b1_cubic_joint_traj.csv`
  - `docs/b1_quintic_joint_traj.csv`
  - `docs/b1_trapezoidal_joint_traj.csv`
  - `docs/b1_cartesian_traj.csv`
  - `docs/b1_waypoint_traj.csv`

## Implementation

All trajectory generators return lists of `JointTrajectoryPoint(t, q, qd, qdd)` dataclass instances (or `(time, position)` tuples for Cartesian trajectories).

### 1. Cubic Polynomial (`cubic_trajectory()`)

- Boundary conditions: zero velocity at start and end.
- Coefficients: `a0 = q_start`, `a1 = 0`, `a2 = 3*dq/T^2`, `a3 = -2*dq/T^3`.
- Produces smooth position and velocity profiles but acceleration is discontinuous at endpoints.

### 2. Quintic Polynomial (`quintic_trajectory()`)

- Boundary conditions: zero velocity AND zero acceleration at start and end.
- Coefficients: `a0 = q_start`, `a3 = 10*dq/T^3`, `a4 = -15*dq/T^4`, `a5 = 6*dq/T^5`.
- Smoother than cubic -- continuous acceleration profile, suitable for computed torque control.

### 3. Trapezoidal Velocity Profile (`trapezoidal_trajectory()`)

- Three-phase motion: acceleration, cruise at `v_max`, deceleration.
- Takes per-joint `v_max` and `a_max` arrays.
- **Synchronized joints**: all joints are computed to finish at the same time (`T_total = max(T_per_joint)`), preserving the direction of motion in joint space.
- Falls back to a triangular profile if the displacement is too small for a cruise phase.

### 4. Minimum Jerk Cartesian (`minimum_jerk_trajectory()`)

- Generates a straight-line path in Cartesian space (position only, no orientation).
- Uses the minimum-jerk timing law: `s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5`.
- Returns `(time, position)` tuples.
- The demo computes start/end EE positions from `Q_HOME` and `q_end` via Pinocchio FK.

### 5. Multi-Segment Via-Point (`multi_segment_trajectory()`)

- Chains quintic segments through a list of joint-space waypoints.
- Each segment gets its own duration from the `durations` list.
- Time offsets are accumulated so the full trajectory is continuous.

### Feasibility Check

After generating all trajectories, the demo checks the maximum per-joint velocity against `VELOCITY_LIMITS` from `ur5e_common.py`. Any violations are flagged with the offending joint indices.

## How to Run

```bash
python3 src/b1_trajectory_generation.py
```

## What to Study

1. Derive the cubic and quintic coefficients from the boundary conditions by hand.
2. Verify endpoint conditions in the output: cubic should show zero start/end velocity; quintic should show zero start/end velocity AND acceleration.
3. Understand the trapezoidal profile's joint synchronization logic -- this is how industrial robots coordinate multi-joint motions.
4. Compare the minimum-jerk timing law with a simple linear interpolation: minimum jerk produces zero velocity and acceleration at endpoints.
5. Check the feasibility output to see whether any trajectory violates the UR5e velocity limits.

## Next Step

Move to B2 (Control Hierarchy) to track these trajectories with feedback controllers in MuJoCo.
