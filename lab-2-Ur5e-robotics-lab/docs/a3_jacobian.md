# A3: Jacobian and Manipulability

## Goal

Compute the 6x6 Jacobian three different ways, cross-validate them, and use the Jacobian to analyze manipulability and detect singularities across the UR5e workspace.

## Files

- Script: `src/a3_jacobian.py`
- Output: `docs/a3_manipulability_heatmap.csv`

## Implementation

The script is organized into four demo sections:

### Part 1: Jacobian Cross-Validation

Three independent Jacobian computation methods are compared at five configurations (`zeros`, `home`, `elbow_open`, `wrist_flip`, `random`):

1. **Geometric Jacobian** (`geometric_jacobian()`) -- built from Pinocchio FK frame data. For each revolute joint, the function reads the joint placement `data.oMi[joint_id]` and determines the joint axis from the Pinocchio joint shortname (`RZ`, `RY`, or `RX`). The linear column is `z_i x (p_ee - p_i)` and the angular column is `z_i`, both in world frame.
2. **Pinocchio built-in Jacobian** (`pinocchio_jacobian()`) -- calls `pin.computeFrameJacobian()` with `LOCAL_WORLD_ALIGNED` reference frame, which produces a world-aligned Jacobian at the EE origin.
3. **Numerical Jacobian** (`numerical_jacobian()`) -- central finite differences with perturbation `eps=1e-6`. The angular part uses `pin.log3()` on the relative rotation `R_plus @ R_minus.T` to get the angular velocity approximation.

All three methods must agree within Frobenius norm < 1e-4 at every test configuration.

### Part 2: Manipulability Analysis

- **Yoshikawa manipulability index**: `w = sqrt(det(J @ J.T))`, computed via `manipulability()`.
- **Jacobian determinant**: `det(J)` via `jacobian_determinant()`.
- **Singular value decomposition** at the home configuration, printing all six singular values and the condition number `sigma_1 / sigma_6`.

### Part 3: Singularity Detection

The function `detect_singularity()` checks three classical UR5e singularity types plus a general manipulability threshold:

- **Wrist singularity**: `|q5| < 0.05 rad` (~2.9 degrees), where wrist axes 4 and 6 align.
- **Shoulder singularity**: EE xy-distance from the base z-axis < 0.02 m.
- **Elbow singularity**: `|q3| < 0.05 rad` or `|q3 - pi| < 0.05 rad` (links 2 and 3 collinear).
- **Low manipulability**: `w < 1e-3`.

Five probe configurations are tested, including known singular and non-singular cases.

### Part 4: Manipulability Heatmap

`compute_manipulability_heatmap()` sweeps q2 from `-pi` to `pi/6` and q3 from `-pi` to `pi` on a 25x25 grid, keeping all other joints fixed at `Q_HOME`. For each grid point, both manipulability and `det(J)` are recorded. The results are saved to `a3_manipulability_heatmap.csv` with columns `q2_rad, q3_rad, manipulability, det_j`. Summary statistics (min, max, mean, std) and the best/worst dexterity configurations are printed.

## How to Run

```bash
python3 src/a3_jacobian.py
```

## What to Study

1. Re-derive `J_linear[:, i] = z_i x (p_ee - p_i)` and `J_angular[:, i] = z_i` by hand for a revolute joint.
2. Understand why `LOCAL_WORLD_ALIGNED` is the correct reference frame for comparing with the geometric Jacobian.
3. Trace the `log3` step in the numerical Jacobian and understand why it is needed for the angular part (as opposed to simple subtraction).
4. Open `a3_manipulability_heatmap.csv` and observe how `q2` and `q3` affect dexterity. Identify the near-singular regions.
5. Compare the singularity probe results: the wrist singularity case should show collapsing `det(J)` and low manipulability.

## Next Step

Move to A4 (Inverse Kinematics) to use the Jacobian in iterative IK solvers.
