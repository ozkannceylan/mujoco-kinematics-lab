# Lab 1 Status Snapshot

This file is a lightweight progress note for the current state of Lab 1.

## What Is Solid

- `C1` square drawing is implemented and documented.
- The final tracked task reaches:
  - RMS Cartesian error: **0.008 mm**
  - Max Cartesian error: **0.013 mm**
- The lab contains:
  - source code under `src/`
  - English notes under `docs/`
  - Turkish notes under `docs-turkish/`
  - blog writeups under `blog/`
  - final media under `media/`

## Important Technical Notes

- The MuJoCo models use `compiler angle="radian"` to avoid joint-limit unit mistakes.
- The final demo relies on:
  - quintic Cartesian trajectories
  - analytic IK
  - Jacobian-based velocity mapping
  - computed torque control
- Some artifacts are conditional and depend on the local environment:
  - MuJoCo-specific CSV outputs require `mujoco`
  - plots and GIFs may require `matplotlib` or a compatible writer

## Suggested Next Iterations

- add more drawing tasks such as circles or polygons
- compare PD and computed torque under harder trajectories
- package the ROS2 bridge more formally if ROS2 integration becomes a priority
