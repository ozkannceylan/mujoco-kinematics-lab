# A5: Dynamics

## Goal

Build the full rigid-body dynamics layer for the UR5e using Pinocchio's analytical algorithms and cross-validate against MuJoCo.

## Files

- Script: `src/a5_dynamics.py`
- MuJoCo wrapper: `src/mujoco_sim.py` (`UR5eSimulator`)
- Output: `docs/a5_dynamics_snapshot.csv`

## Implementation

The script covers seven analyses across five test configurations (`zeros`, `home`, `elbow_up`, `shoulder_only`, `random`):

### 1. Mass Matrix M(q)

- Computed via `pin.crba(model, data, q)`.
- CRBA only fills the upper triangle, so the code explicitly symmetrizes: `M = triu(M) + triu(M, 1).T`.
- For each configuration, `analyse_mass_matrix()` reports: shape, symmetry, positive definiteness, eigenvalues, condition number, and diagonal elements (effective inertias per joint).

### 2. Gravity Vector g(q)

- Computed via `pin.computeGeneralizedGravity(model, data, q)`.
- Reports the gravity torque vector and identifies the joint with the highest gravitational load.

### 3. Coriolis Matrix C(q, v)

- Computed via `pin.computeCoriolisMatrix(model, data, q, v)`.
- Reports `C @ v` (the Coriolis torque contribution) and the Frobenius norm of C at each configuration.
- **Skew-symmetry verification**: numerically approximates `M_dot` via finite differences, then checks that `N = M_dot - 2C` is skew-symmetric by verifying `||N + N.T||_F ~ 0`. This confirms the passivity property of the dynamics.

### 4. RNEA -- Inverse Dynamics

- `pin.rnea(model, data, q, v, a)` computes `tau = M*a + C*v + g`.
- When `v=0` and `a=0`, RNEA should return exactly `g(q)`. The script verifies this identity (`np.allclose` with `atol=1e-10`).

### 5. ABA -- Forward Dynamics

- `pin.aba(model, data, q, v, tau)` computes `qdd = M^{-1}(tau - C*v - g)`.
- With zero torque and zero velocity, reports the gravitational acceleration each joint would experience.
- **Verification**: compares ABA output against manual computation `M^{-1}(tau - g)` to confirm they match (`atol=1e-8`).

### 6. Cross-Validation: Pinocchio RNEA vs MuJoCo qfrc_bias

- At each configuration with `v=0`, Pinocchio RNEA gives `g(q)` and MuJoCo `qfrc_bias` (at zero velocity) also gives gravity torques.
- The `UR5eSimulator` wrapper is used to set joint positions and read `qfrc_bias`.
- Per-joint absolute errors are reported, with a pass threshold of < 0.1 Nm (typically < 0.01 Nm).

### 7. Condition Number Sweep

- Reports the condition number of `M(q)`, along with minimum and maximum eigenvalues, at each test configuration.
- High condition numbers indicate that some joint-space directions require much more torque than others.

## How to Run

```bash
python3 src/a5_dynamics.py
```

## What to Study

1. Start with the gravity vector `g(q)`: understand which joints carry the most gravitational load at different configurations.
2. Inspect the mass matrix diagonal: these are the effective inertias. Note how they change as the arm configuration changes.
3. Trace the skew-symmetry check: this property (`M_dot - 2C` is skew-symmetric) is fundamental to passivity-based control design.
4. Compare RNEA vs `g(q)` at zero velocity/acceleration to build intuition for what inverse dynamics does.
5. Study the cross-validation: MuJoCo and Pinocchio should agree within < 0.01 Nm, confirming both models are consistent.

## Next Step

Move to B1 (Trajectory Generation) to create smooth joint-space and Cartesian references that the dynamics model will need to track.
