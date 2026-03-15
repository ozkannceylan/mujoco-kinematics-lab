# A4: Inverse Kinematics

## Goal

Solve full 6-DOF IK (position + orientation) for the UR5e using numerical methods, validate solutions via FK roundtrip and MuJoCo cross-validation, and benchmark solver performance on random reachable targets.

## Files

- Script: `src/a4_inverse_kinematics.py`
- Output: `docs/a4_ik_benchmark.csv`

## Implementation

The script provides two IK solvers, a validation pipeline, and a benchmark, organized into three demo sections.

### Data structure

`IKResult` dataclass holds: `success`, `q`, `iterations`, `final_error`, `position_error`, `orientation_error`, `elapsed_s`.

### Solver 1: Jacobian Pseudo-Inverse (`ik_pseudoinverse()`)

- Computes the 6D error as `e = [p_target - p_current; log3(R_target @ R_current.T)]`.
- Uses `pin.computeFrameJacobian()` in `LOCAL_WORLD_ALIGNED` frame.
- Applies the Moore-Penrose pseudo-inverse: `dq = pinv(J) @ e`.
- Step size `alpha = 0.5`, convergence tolerance `tol = 1e-4`, max 200 iterations.
- Joint limit clamping (`np.clip`) at every iteration.

### Solver 2: Damped Least Squares (`ik_damped_least_squares()`)

- Same 6D error and Jacobian as pseudo-inverse.
- Uses DLS update: `dq = J.T @ (J @ J.T + lambda^2 * I)^{-1} @ e`.
- **Adaptive lambda**: if error decreases, halve lambda (more aggressive); if error increases, double lambda (more conservative). Clamped to `[lam_min=1e-4, lam_max=1.0]`.
- Default: `alpha=0.5`, `tol=1e-4`, max 300 iterations.
- Joint limit clamping at every iteration.

### FK Roundtrip Validation (`validate_ik_solution()`)

Takes a solved `q`, runs FK, and checks that the resulting EE pose matches the target within position tolerance (1 mm default) and orientation tolerance (0.01 rad default).

### MuJoCo Cross-Validation (`apply_ik_in_mujoco()`)

Solves IK using Pinocchio (DLS method), then sets the joint angles in MuJoCo via `load_mujoco_model()`, runs `mj_forward()`, and reads back the EE site position. Reports the mismatch in mm -- should be < 1.0 mm.

### Benchmark (`run_benchmark()`)

- Generates 50 random reachable targets by sampling random joint configurations within joint limits and computing FK to get `(p_target, R_target)`.
- For each target, both pseudo-inverse and DLS are run from a perturbed initial guess (`Q_HOME + N(0, 0.3)`).
- Reports success rate, mean iterations, mean time, mean position error, and mean orientation error for each method.
- DLS typically achieves ~62% success rate vs pseudo-inverse ~54% on these random targets.

### Demo sections

1. **Single-target IK** -- solves for a known configuration with both methods, prints full results.
2. **MuJoCo verification** -- solves and cross-validates.
3. **Benchmark** -- 50-target comparison with summary statistics.

## How to Run

```bash
python3 src/a4_inverse_kinematics.py
```

## What to Study

1. Trace the 6D error construction: understand why `log3(R_target @ R_current.T)` gives the axis-angle rotation error.
2. Compare pseudo-inverse and DLS: understand how the damping factor prevents instability near singularities.
3. Study the adaptive lambda logic: it acts like a trust-region method, backing off when the error increases.
4. Check the FK roundtrip: this is the gold standard for validating any IK solver.
5. Read the MuJoCo cross-validation to see how Pinocchio and MuJoCo agree on the FK of the solved configuration.

## Next Step

Move to A5 (Dynamics) to compute the mass matrix, gravity vector, and Coriolis terms needed for model-based control.
