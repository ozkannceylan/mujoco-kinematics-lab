# B2: Control Hierarchy

## Goal

Implement and compare four controller levels on the same UR5e trajectory in MuJoCo, progressing from simple PD to full operational-space control.

## Files

- Script: `src/b2_control_hierarchy.py`
- Dependencies: `b1_trajectory_generation.py` (quintic trajectory), `mujoco_sim.py` (`UR5eSimulator`)
- Output CSVs:
  - `docs/b2_controller_summary.csv`
  - `docs/b2_pd_gravity_tracking.csv`
  - `docs/b2_computed_torque_tracking.csv`
  - `docs/b2_task_impedance_tracking.csv`
  - `docs/b2_osc_tracking.csv`

## Implementation

### Data structure

`ControlLog` dataclass records per-timestep: `name`, `times`, `pos_errors` (joint error norm), `torques` (torque norm), `ee_errors` (EE position error in meters).

### Controller 1: PD + Gravity Compensation (`pd_gravity_control()`)

```
tau = Kp * (q_des - q) + Kd * (qd_des - qd) + g(q)
```

- Gravity vector computed via `pin.computeGeneralizedGravity()`.
- Simplest stable controller. Does not account for inertia or Coriolis effects.

### Controller 2: Computed Torque (`computed_torque_control()`)

```
tau = M(q) * (qdd_des + Kp*e + Kd*ed) + C*qd + g
```

- Implemented via `pin.rnea(model, data, q, qd, a_cmd)` where `a_cmd = qdd_des + Kp*(q_des-q) + Kd*(qd_des-qd)`.
- RNEA computes the full `M*a + C*v + g` in one call, so the dynamics are exactly canceled.
- Should achieve the best joint-space tracking accuracy.

### Controller 3: Task-Space Impedance (`task_space_impedance_control()`)

```
tau = J.T * F + g(q)
```

where `F = [Kp_pos * e_pos - Kd_pos * xd_linear; Kp_rot * e_rot - Kd_rot * xd_angular]`.

- Position error: `e_pos = x_des - x_cur`.
- Orientation error: `e_rot = log3(R_des @ R_cur.T)`.
- Jacobian via `pin.getFrameJacobian()` in `LOCAL_WORLD_ALIGNED` frame.
- Cartesian velocity approximated as `xd = J @ qd`.
- Default gains: `Kp_pos=200`, `Kd_pos=40`, `Kp_rot=50`, `Kd_rot=10`.
- Provides compliant Cartesian behavior -- important for contact tasks.

### Controller 4: Operational Space Control (`osc_control()`)

```
tau = J.T * Lambda * (xdd_des + Kp*e - Kd*xd) + h
```

where `Lambda = (J * M^{-1} * J.T)^{-1}` is the task-space inertia matrix (Khatib 1987), and `h = RNEA(q, qd, 0)` captures gravity and Coriolis.

- Decouples task-space dynamics by shaping the apparent inertia.
- Regularization: `Lambda = inv(J*M_inv*J.T + 1e-6*I)` for numerical stability near singularities.

### Generic Simulation Loop (`run_controller()`)

- Takes any controller function with signature `(t, q, qd, traj_point) -> torques`.
- Resets the `UR5eSimulator` to the trajectory start.
- Steps MuJoCo at its native timestep, looking up the current trajectory point by time.
- Clips all torques to [-150, +150] Nm.
- Logs joint error norm, torque norm, and EE position error (compared to desired via Pinocchio FK) at every step.

### Comparison

The demo generates a 3-second quintic trajectory from `Q_HOME` to a target configuration and runs PD+g, computed torque, and task-space impedance on it. Results are printed in a table showing: RMS joint error (rad), RMS EE error (m), mean torque norm (Nm), and max torque norm (Nm).

Gains used: `Kp = [2000, 2000, 2000, 500, 500, 500]`, `Kd = [400, 400, 400, 100, 100, 100]`.

## How to Run

```bash
python3 src/b2_control_hierarchy.py
```

## What to Study

1. Start with the PD+g controller: understand why gravity compensation is needed even for a simple PD.
2. Compare computed torque vs PD+g: the RNEA-based cancellation should dramatically reduce tracking error.
3. Read the task-space impedance controller: understand the `J.T * F` mapping from Cartesian wrench to joint torques.
4. Study OSC last: focus on the task-space inertia `Lambda` and why it decouples the dynamics in operational space.
5. Compare the summary table: computed torque should have the lowest joint-space error, while impedance/OSC operate in task space.

## Next Step

Move to B3 (Constraints) to add joint limit safety, velocity scaling, and self-collision checking.
