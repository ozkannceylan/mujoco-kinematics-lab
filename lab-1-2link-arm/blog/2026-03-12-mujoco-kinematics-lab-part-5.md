---
title: "MuJoCo Kinematics Lab — Part 5: Computed Torque Control — Drawing a Cartesian Square"
date: "2026-03-12"
description: "The grand finale: cancelling nonlinear dynamics with computed torque control to draw a precise Cartesian square in MuJoCo."
tags: ["Robotics", "Computed Torque Control", "MuJoCo", "Dynamics", "Python"]
---

Final post in the [Lab 1 series](../README.md). Every module built so far — FK, Jacobians, IK, trajectory generation, PD control, and dynamics — comes together in a single script that draws a 10 cm Cartesian square with sub-millimetre accuracy.

**Series:**
1. Setup and Modeling
2. Forward Kinematics, Workspace, and Jacobians
3. Inverse Kinematics: Analytic, Pseudo-inverse, and DLS
4. Trajectory Generation and PD Control
5. **Computed Torque Control: Drawing a Cartesian Square** (this post)

---

## Companion Files In This Repo

- Overview: [../README.md](../README.md)
- Final module note: [../docs/c1_draw_square.md](../docs/c1_draw_square.md)
- Final source scripts: [../src/c1_draw_square.py](../src/c1_draw_square.py), [../src/c1_record_video.py](../src/c1_record_video.py)
- Demo media: [../media/c1_draw_square.gif](../media/c1_draw_square.gif), [../media/c1_draw_square.mp4](../media/c1_draw_square.mp4)
- Metrics: [../docs/b3_metrics.csv](../docs/b3_metrics.csv)
- Blog index: [README.md](README.md)

---

## Why Not Just PD?

PD control with gravity compensation treats the robot as a simple spring-damper system. It ignores the fact that the mass matrix M(q) changes with configuration and that Coriolis forces couple the joints at speed. For slow motions this approximation is acceptable. For precise, fast trajectory tracking it is not.

Computed torque control (CTC) solves this by explicitly cancelling the nonlinear dynamics.

## The Control Law

The robot's equation of motion:

```
M(q) · q̈ + C(q, q̇) · q̇ + g(q) = τ
```

CTC chooses the torque:

```
u = q̈_des + Kp · (q_des - q) + Kd · (q̇_des - q̇)
τ = M(q) · u + C(q, q̇) · q̇ + g(q)
```

Substituting back, the closed-loop error dynamics become:

```
ë + Kd · ė + Kp · e = 0
```

This is a **linear second-order system** — all nonlinearities have been cancelled. The gain selection determines the response:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| Kp | 400 | ωn² where ωn = 20 rad/s |
| Kd | 40 | 2·ωn → damping ratio ζ = 1.0 |
| Settling time | ~0.2 s | 4/(ζ·ωn) |

With ζ = 1.0 (critical damping): no overshoot, fastest non-oscillatory convergence, and minimal tracking error.

## The Full Pipeline

```
square corners → quintic Cartesian trajectory → analytic IK → J⁻¹ velocity mapping → computed torque → MuJoCo sim
```

### Step 1: Path Planning

Four corners of a 10 cm square centered at (0.30, 0.30) m. Each side is interpolated with a quintic profile that guarantees zero velocity and zero acceleration at the start and end — smooth stop/start at every corner.

### Step 2: Inverse Kinematics

At every timestep, `analytic_ik` converts the Cartesian target (x, y) to joint angles. The IK branch closest to the previous solution is selected for continuity. For the first point, elbow-down (q₂ < 0) is preferred to keep the arm away from the base.

### Step 3: Jacobian Velocity Mapping

The desired Cartesian velocity [ẋ, ẏ] is converted to joint velocities using the Jacobian inverse:

```
q̇_des = J⁻¹ · [ẋ, ẏ]
```

Near singularities (det(J) < 1e-8), a safe zero velocity is returned — the quintic profile already produces zero velocity at corners, so this is a natural fit.

### Step 4: Acceleration via Finite Differences

Joint acceleration q̈_des is computed via central finite differences:

```
q̈[i] = (q̇[i+1] - q̇[i-1]) / (2·dt)
```

Simpler than deriving the analytic J̇ and sufficiently accurate at dt = 0.002 s.

### Step 5: Torque Computation

At each simulation step, MuJoCo provides:
- `M(q)` via `mj_fullM(model, M, data.qM)` — the configuration-dependent mass matrix
- `qfrc_bias` — combined Coriolis + gravity terms
- `qfrc_passive` — joint damping forces

The control torque:

```python
u = qdd_des + KP * (q_des - q) + KD * (qd_des - qd)
tau = M @ u + qfrc_bias - qfrc_passive
```

The `qfrc_passive` subtraction accounts for the joint damping that MuJoCo applies automatically.

## Issues Discovered and Resolved

### 1. Angle Unit Mismatch

The model XML had `range="-3.14 3.14"` but MuJoCo defaults to degrees. This was interpreted as ±3.14° (±0.055 rad), causing joint limit constraints to activate during normal operation. **Fix:** `<compiler angle="radian"/>` in the XML.

### 2. Decorative Geom Collision

Base platform and joint visualizer geoms had default collision enabled. At certain configurations they overlapped with link capsules, generating large constraint forces. **Fix:** disabling all collisions at runtime:

```python
model.geom_contype[:] = 0
model.geom_conaffinity[:] = 0
```

### 3. IK Branch Selection

Near corners, both elbow branches have similar |q₂| values, and the positive branch folds toward the base. **Fix:** prefer elbow-down for the initial configuration, then select by proximity for continuity.

## Results

Headless simulation: 4001 steps, 8.0 seconds of simulated time.

| Metric | Value |
|--------|-------|
| RMS Cartesian error | **0.008 mm** |
| Max Cartesian error | **0.013 mm** |
| Final Cartesian error | 0.005 mm |
| Max applied torque | 0.077 N·m |
| Torque saturation events | None |
| Torque limit | 5.0 N·m |

Sub-millimetre tracking with less than 2% of the available torque budget used. The controller is not fighting the dynamics — it is working *with* them.

## Visualization

The MuJoCo viewer renders in real time:
- **Green lines** — the target square
- **Red dots** — end-effector trail (every 5 simulation steps)
- **Yellow sphere** — current end-effector position

The trail visually confirms that all four sides and corners of the square are tracked precisely with no overshoot at transitions.

## Modules Used

| Module | Function | Purpose |
|--------|----------|---------|
| A2/A3 | `fk_endeffector`, `analytic_jacobian` | FK and J⁻¹ velocity mapping |
| A4 | `analytic_ik` | Cartesian → joint-space conversion |
| A5 | `mj_fullM`, `qfrc_bias` | Mass matrix and bias forces |
| B1 | `quintic_profile` | Smooth Cartesian interpolation |
| B2 | Kp/Kd error terms | PD correction inside CTC |

## Series Takeaways

This project was built to show how robotics fundamentals connect into a working system. The deliberate choice of a 2-link arm keeps the math visible at every step — the same pipeline structure (IK → trajectory → model-based control) scales directly to higher-DOF manipulators.

Key engineering patterns that transfer:
- **Verify each module independently** before integration — FK against MuJoCo, Jacobian against finite differences, IK against FK
- **Quintic profiles** eliminate torque discontinuities at segment transitions
- **Computed torque control** linearizes the plant, making gain design straightforward (just pick ωn and ζ)
- **Branch continuity** in IK prevents configuration jumps during continuous motion
- **Debug systematically** — the angle unit bug and collision issue were caught by inspecting intermediate quantities, not by staring at the final output

The local lab folder already contains the source code, documentation, media, and CSV logs used throughout this series.

## Navigation

- Previous: [Part 4](2026-03-11-mujoco-kinematics-lab-part-4.md)
- Back to lab overview: [../README.md](../README.md)
