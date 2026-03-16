---
title: "MuJoCo Robotics Lab — Part 4: Trajectory Generation and PD Control"
date: "2026-03-11"
description: "Generating smooth motion profiles with quintic polynomials and tracking them with PD control and gravity compensation."
tags: ["Robotics", "Trajectory Planning", "PD Control", "MuJoCo", "Python"]
---

Part 4 of the [Lab 1 series](../README.md). With FK, Jacobians, and IK established, we now generate smooth trajectories and build a feedback controller to follow them.

**Series:**
1. Setup and Modeling
2. Forward Kinematics, Workspace, and Jacobians
3. Inverse Kinematics: Analytic, Pseudo-inverse, and DLS
4. **Trajectory Generation and PD Control** (this post)
5. From Kinematics to Computed Torque Control: Drawing a Cartesian Square

---

## Companion Files In This Repo

- Overview: [../README.md](../README.md)
- Module notes: [../docs/b1_trajectory_generation.md](../docs/b1_trajectory_generation.md), [../docs/b2_pd_controller.md](../docs/b2_pd_controller.md), [../docs/b3_full_pipeline.md](../docs/b3_full_pipeline.md)
- Source scripts: [../src/b1_trajectory_generation.py](../src/b1_trajectory_generation.py), [../src/b2_pd_controller.py](../src/b2_pd_controller.py), [../src/b3_full_pipeline.py](../src/b3_full_pipeline.py)
- Artifacts: [../docs/b1_quintic_joint_traj.csv](../docs/b1_quintic_joint_traj.csv), [../docs/b2_tracking_gc.csv](../docs/b2_tracking_gc.csv), [../docs/b3_metrics.csv](../docs/b3_metrics.csv)
- Blog index: [README.md](README.md)

---

## Two Approaches to Trajectory Generation

### Joint-Space Trajectories

Joint angles are interpolated directly between start and goal configurations. The end-effector path is not a straight line in Cartesian space, but the computation is cheap and avoids IK at every sample.

### Cartesian Trajectories

End-effector positions are interpolated in Cartesian space (e.g., straight-line segments), then IK is solved at every sample to get the corresponding joint angles. The end-effector follows the intended path precisely, at the cost of solving IK per timestep.

Both approaches are implemented in the lab. The final square-drawing demo uses Cartesian trajectories with quintic profiles.

## Cubic vs. Quintic Polynomials

### Cubic Profile

A cubic polynomial satisfies four boundary conditions:

```
q(0) = q₀,  q(T) = q_f
q̇(0) = 0,   q̇(T) = 0
```

This ensures the trajectory starts and ends at rest, but **acceleration is discontinuous** at the boundaries — the torque jumps instantaneously, which can excite vibrations.

### Quintic Profile

A fifth-order polynomial satisfies six boundary conditions:

```
q(0) = q₀,    q(T) = q_f
q̇(0) = 0,     q̇(T) = 0
q̈(0) = 0,     q̈(T) = 0
```

The additional zero-acceleration constraints mean the torque profile starts and ends smoothly. This is significantly better for real hardware and for computed torque control, where the feedforward term depends on acceleration.

The quintic profile is the one used in the final square-drawing demo. Each side of the square gets its own quintic segment with guaranteed smooth transitions at the corners.

## PD Control

The basic joint-space tracking controller:

```
τ = Kp · (q_des - q) + Kd · (q̇_des - q̇) + g(q)
```

- **Kp** (proportional gain): responds to position error
- **Kd** (derivative gain): damps velocity error, prevents oscillation
- **g(q)** (gravity compensation): cancels gravity loading so the controller only needs to handle tracking error

### Why Gravity Compensation Matters

Without gravity compensation, the controller has to "fight" gravity constantly. This introduces steady-state error (the gains cannot be infinite) and wastes control effort. With gravity compensation, the PD terms only handle the tracking task — the steady-state error drops to near zero.

In the lab, two scenarios were tested:

| Scenario | Gravity Comp | Behavior |
|----------|-------------|----------|
| Fixed target, no GC | Off | Noticeable steady-state offset |
| Fixed target, with GC | On | Reaches target precisely |
| Trajectory tracking, no GC | Off | Larger RMS tracking error |
| Trajectory tracking, with GC | On | Reduced error throughout |

### Limitations of PD Control

PD + gravity compensation works well for slow, smooth trajectories. But it treats the robot as if it had no inertia or Coriolis effects — it does not account for the fact that the torque needed to produce a given acceleration depends on the configuration.

For fast motions or high accuracy, we need a controller that knows the dynamics: **computed torque control**.

## Full Pipeline Integration

Before the final demo, the lab includes a full pipeline integration test combining all modules:

```
target → IK → trajectory → PD → plant → metrics
```

Three demos validate the chain:
1. **Pick-and-place:** waypoints converted to joint-space via analytic IK, connected with quintic trajectories, tracked with PD control
2. **Circle tracking:** Cartesian circle target with IK at every sample
3. **Singularity edge:** targets near the workspace boundary, comparing pseudo-inverse vs. DLS solver robustness

These integration tests confirmed the pipeline works end-to-end and identified the precision limits of PD control — motivating the jump to computed torque control.

## Key Takeaways

1. **Quintic > cubic** for smooth motion — the extra boundary conditions eliminate torque discontinuities at segment transitions.
2. **Gravity compensation is non-negotiable** for any controller operating with gravity present.
3. **PD control is a strong baseline** but fundamentally limited — it doesn't model the arm's dynamics, so tracking accuracy degrades with speed and configuration changes.
4. **Pipeline integration** early in the project catches interface bugs before the final demo.

## What's Next

The final post brings everything together: computed torque control cancels the arm's nonlinear dynamics entirely, reducing the tracking problem to simple linear error dynamics. The result is a precise Cartesian square drawn with sub-millimetre accuracy.

## Navigation

- Previous: [Part 3](2026-03-10-mujoco-robotics-lab-part-3.md)
- Next: [Part 5](2026-03-12-mujoco-robotics-lab-part-5.md)
