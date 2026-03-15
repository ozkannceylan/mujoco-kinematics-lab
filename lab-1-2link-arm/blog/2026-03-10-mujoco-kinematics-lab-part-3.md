---
title: "MuJoCo Kinematics Lab — Part 3: Inverse Kinematics — Analytic, Pseudo-inverse, and DLS"
date: "2026-03-10"
description: "Solving the inverse problem: comparing closed-form IK with numerical Jacobian methods and handling singularities."
tags: ["Robotics", "Inverse Kinematics", "MuJoCo", "Jacobian", "Python"]
---

Part 3 of the [Lab 1 series](../README.md). Having established FK and the Jacobian, we now tackle the inverse: given a target Cartesian position, find the joint angles that reach it.

**Series:**
1. Setup and Modeling
2. Forward Kinematics, Workspace, and Jacobians
3. **Inverse Kinematics: Analytic, Pseudo-inverse, and DLS** (this post)
4. Trajectory Generation and PD Control
5. From Kinematics to Computed Torque Control: Drawing a Cartesian Square

---

## Companion Files In This Repo

- Overview: [../README.md](../README.md)
- Module note: [../docs/a4_inverse_kinematics.md](../docs/a4_inverse_kinematics.md)
- Source script: [../src/a4_inverse_kinematics.py](../src/a4_inverse_kinematics.py)
- Benchmark artifact: [../docs/a4_ik_benchmark.csv](../docs/a4_ik_benchmark.csv)
- Blog index: [README.md](README.md)

---

## The IK Problem

```
Given:  (x_target, y_target)
Find:   (θ₁, θ₂)
```

Unlike FK, which has a unique answer, IK is generally non-unique (multiple joint configurations can reach the same point) and may have no solution (target outside the workspace).

For our 2-link arm with the effective link length `L₂_eff = 0.315 m`:
- **Reachable zone:** 0.015 m ≤ r ≤ 0.615 m (an annulus)
- **Solution multiplicity:** every reachable point has exactly two solutions (elbow-up and elbow-down)

## Method 1: Analytic (Closed-Form) IK

The 2-link planar arm admits a closed-form solution using the law of cosines:

```
cos(θ₂) = (x² + y² - L₁² - L₂_eff²) / (2 · L₁ · L₂_eff)
θ₂ = ± acos(cos(θ₂))
θ₁ = atan2(y, x) - atan2(L₂_eff · sin(θ₂), L₁ + L₂_eff · cos(θ₂))
```

The `±` on `θ₂` gives the two elbow configurations. For the target `(0.34, 0.28)`:

| Branch | θ₁ | θ₂ | FK Error |
|--------|----|----|----------|
| Elbow-down | -6.17° | 88.55° | 0.0 |
| Elbow-up | 85.11° | -88.55° | 0.0 |

Both solutions produce zero FK error — they are exact.

**Advantage:** instant computation, no iteration.
**Limitation:** only works for low-DOF systems where a closed-form exists.

## Method 2: Pseudo-inverse (Jacobian Inverse)

For systems without closed-form solutions, we iterate using the Jacobian:

```
Δθ = J⁻¹ · (x_target - x_current)
θ ← θ + α · Δθ
```

For the 2-link arm, J is square and invertible (away from singularities), so `J⁻¹` is exact. The iteration converges quickly: **12.5 iterations on average** across 20 random targets with a 100% success rate.

**Problem at singularities:** When `det(J) → 0`, the inverse blows up. In the stress test (target at near-maximum reach, initial guess at full extension), the pseudo-inverse method failed on the first step due to a singular Jacobian.

## Method 3: Damped Least Squares (DLS)

DLS adds a damping term to regularize the inverse:

```
Δθ = Jᵀ · (J · Jᵀ + λ² · I)⁻¹ · e
```

The damping factor `λ` trades off tracking accuracy for numerical stability. Near singularities, the damping prevents the joint velocity from exploding — the solver slows down instead of diverging.

**Singularity stress test result:**
- Same setup that crashed pseudo-inverse
- DLS converged in **161 iterations** with **9.83e-6 m** final error
- Slower, but robust where it matters

## Benchmark: 20 Random Targets

| Method | Success Rate | Avg Iterations |
|--------|-------------|----------------|
| Pseudo-inverse | 100% | 12.5 |
| DLS | 100% | 13.5 |

Both methods achieve 100% on randomly sampled feasible targets. The difference emerges only near singularities — exactly when it matters most for real trajectory tracking.

## Branch Selection for Continuous Motion

When IK is called repeatedly along a trajectory (as in our final square-drawing demo), branch continuity matters. The implementation selects the IK branch closest to the previous solution to avoid sudden configuration jumps. For the first point, elbow-down is preferred to keep the arm away from the base platform.

## Dynamics Context

Before moving to trajectory generation, one more foundation piece: the robot's dynamics equation.

```
τ = M(q) · q̈ + C(q, q̇) · q̇ + g(q)
```

MuJoCo computes these terms internally:
- **`data.qM`** → mass/inertia matrix M(q), expanded via `mj_fullM()`
- **`data.qfrc_bias`** → combined Coriolis + gravity term

M(q) is configuration-dependent — as the links change pose, the mass distribution shifts, and the torque required for a given acceleration changes. This becomes critical for computed torque control in the final post.

## Key Takeaways

1. **Analytic IK is ideal when available** — instant, exact, and gives all solution branches. But it only exists for simple geometries.
2. **Pseudo-inverse is fast but fragile** near singularities. Fine for offline planning, risky for real-time control.
3. **DLS is the practical choice** for real-time IK in the presence of singularities — the damping parameter is the engineering knob.
4. **Branch continuity** is essential when chaining IK solutions along a trajectory.

## What's Next

With IK in hand, we can convert Cartesian waypoints into joint-space targets. The next post covers trajectory generation (cubic and quintic profiles) and PD control to track those trajectories smoothly.

## Navigation

- Previous: [Part 2](2026-03-09-mujoco-kinematics-lab-part-2.md)
- Next: [Part 4](2026-03-11-mujoco-kinematics-lab-part-4.md)
