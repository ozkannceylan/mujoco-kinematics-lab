---
title: "MuJoCo Kinematics Lab — Part 2: Forward Kinematics, Workspace, and Jacobians"
date: "2026-03-09"
description: "Deriving the geometry of motion: FK equations, workspace analysis, and the Jacobian matrix for a 2-link planar arm."
tags: ["Robotics", "Kinematics", "Jacobian", "MuJoCo", "Python"]
---

Part 2 of the [Lab 1 series](../README.md). In this post we derive the forward kinematics equations, analyze the reachable workspace, and compute the Jacobian matrix that maps joint velocities to end-effector velocities.

**Series:**
1. Setup and Modeling
2. **Forward Kinematics, Workspace, and Jacobians** (this post)
3. Inverse Kinematics: Analytic, Pseudo-inverse, and DLS
4. Trajectory Generation and PD Control
5. From Kinematics to Computed Torque Control: Drawing a Cartesian Square

---

## Companion Files In This Repo

- Overview: [../README.md](../README.md)
- Module notes: [../docs/a2_forward_kinematics.md](../docs/a2_forward_kinematics.md), [../docs/a3_jacobian.md](../docs/a3_jacobian.md)
- Source scripts: [../src/a2_forward_kinematics.py](../src/a2_forward_kinematics.py), [../src/a3_jacobian.py](../src/a3_jacobian.py)
- FK artifacts: [../docs/a2_fk_workspace.png](../docs/a2_fk_workspace.png), [../docs/a2_fk_configurations.png](../docs/a2_fk_configurations.png), [../docs/a3_det_sweep.csv](../docs/a3_det_sweep.csv)
- Blog index: [README.md](README.md)

---

## Forward Kinematics

FK answers the most fundamental question in robotics: *given joint angles, where is the end-effector?*

```
Input:  θ₁, θ₂  (joint angles, rad)
Output: (x, y)   (end-effector position, m)
```

### Geometric Derivation

DH parameters are overkill for a 2-link arm. The geometric approach is more direct:

```
Joint 2 position (tip of link 1):
  x₁ = L₁ · cos(θ₁)
  y₁ = L₁ · sin(θ₁)

End-effector position:
  x = L₁ · cos(θ₁) + L₂ · cos(θ₁ + θ₂)
  y = L₁ · sin(θ₁) + L₂ · sin(θ₁ + θ₂)
```

The critical detail: `θ₂` is *relative* to link 1, so the absolute angle of link 2 is `θ₁ + θ₂`.

### Homogeneous Transformation

The same result can be obtained via 3×3 matrix multiplication (2D case):

```
T₀₁ = | cos(θ₁)  -sin(θ₁)  L₁·cos(θ₁) |
      | sin(θ₁)   cos(θ₁)  L₁·sin(θ₁) |
      |    0          0          1       |

T₁₂ = | cos(θ₂)  -sin(θ₂)  L₂·cos(θ₂) |
      | sin(θ₂)   cos(θ₂)  L₂·sin(θ₂) |
      |    0          0          1       |

T₀₂ = T₀₁ · T₁₂   →   end-effector = T₀₂[:2, 2]
```

This chain-rule approach scales cleanly to 6+ DOF robots where geometric intuition becomes unwieldy.

### Verification Against MuJoCo

10 different angle combinations were tested, comparing the analytic FK output against MuJoCo's `site_xpos` value:

| θ₁ | θ₂ | FK (x, y) | MuJoCo (x, y) | Error |
|---|---|---|---|---|
| 0° | 0° | (0.615, 0.000) | (0.615, 0.000) | 0 |
| 30° | 45° | (0.341, 0.454) | (0.341, 0.454) | 0 |
| 90° | -45° | (0.223, 0.523) | (0.223, 0.523) | 0 |
| 180° | 0° | (-0.615, 0.000) | (-0.615, 0.000) | 0 |

**Exact match** across all test cases. One subtlety: the `end_effector` site in the model XML sits 1.5 cm past the tip of link 2 (`pos="0.015 0 0"`). Including this offset as `L₂_eff = 0.3 + 0.015 = 0.315 m` in the FK computation gives the exact match.

## Workspace Analysis

The workspace is the set of all points the end-effector can reach:

- **Outer boundary:** r = L₁ + L₂_eff = 0.615 m (arms fully extended)
- **Inner boundary:** r = |L₁ - L₂_eff| = 0.015 m

Since the link lengths are nearly equal, the workspace is approximately a full disk centered at the base. With exactly equal links it would be a complete disk with no hole — here the tiny 1.5 cm offset creates a negligible inner boundary.

Understanding the workspace is essential for later stages: any IK target outside this annulus is unreachable, and targets near the boundaries invite singularities.

## The Jacobian Matrix

The Jacobian is the local linear map from joint velocities to end-effector velocities:

```
ẋ = J(θ) · θ̇
```

where `ẋ = [vx, vy]` and `θ̇ = [θ̇₁, θ̇₂]`.

### Analytic Derivation

Taking partial derivatives of the FK equations with the effective link length:

```
J = | -L₁·sin(θ₁) - L₂_eff·sin(θ₁+θ₂)   -L₂_eff·sin(θ₁+θ₂) |
    |  L₁·cos(θ₁) + L₂_eff·cos(θ₁+θ₂)    L₂_eff·cos(θ₁+θ₂) |
```

Each element has a physical interpretation:
- `J[0,0] = ∂x/∂θ₁` — how much x changes per radian of joint 1
- `J[1,1] = ∂y/∂θ₂` — how much y changes per radian of joint 2

### Verification

Three verification paths were implemented:

1. **Analytic vs. finite-difference Jacobian:** error < 1e-10
2. **Analytic vs. `mj_jacSite`:** exact match with MuJoCo's internal Jacobian computation
3. **Determinant sweep** along θ₂ for singularity analysis

### Singularity Analysis

The determinant of J tells us about the arm's ability to generate arbitrary end-effector velocities:

- `det(J) → 0` means the arm is approaching a **singularity** — it loses the ability to move in at least one Cartesian direction
- For this 2-link arm, singularities occur at `θ₂ ≈ 0` (fully extended) and `θ₂ ≈ π` (fully folded)

At singularity the arm can only move tangentially to a circle, not radially. This directly impacts IK solver behavior — which is the topic of the next post.

## Key Takeaways

1. **Geometric FK is simple but powerful** for low-DOF systems. The chain-rule formulation (homogeneous transforms) is what actually scales.
2. **Watch for site offsets** — the MuJoCo site position may not coincide with the link tip.
3. **The Jacobian is the derivative of FK** — it connects position-level kinematics to velocity-level control.
4. **Singularities are geometric, not numerical** — they correspond to physical configurations where the arm loses a degree of freedom in Cartesian space.

## What's Next

The next post tackles the inverse problem: given a target (x, y), find the joint angles. We compare analytic closed-form solutions with numerical methods (Pseudo-inverse and Damped Least Squares) and see how each handles the singularities identified here.

## Navigation

- Previous: [Part 1](2026-03-08-mujoco-kinematics-lab-part-1.md)
- Next: [Part 3](2026-03-10-mujoco-kinematics-lab-part-3.md)
