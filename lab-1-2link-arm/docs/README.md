# Lab 1 Notes Index

This folder contains the English study notes for Lab 1.

## Recommended Study Order

1. [A1: MuJoCo Basics](a1_mujoco_basics.md)
2. [A2: Forward Kinematics](a2_forward_kinematics.md)
3. [A3: Jacobian](a3_jacobian.md)
4. [A4: Inverse Kinematics](a4_inverse_kinematics.md)
5. [A5: Dynamics Basics](a5_dynamics.md)
6. [B1: Trajectory Generation](b1_trajectory_generation.md)
7. [B2: PD Controller](b2_pd_controller.md)
8. [B3: Full Pipeline](b3_full_pipeline.md)
9. [B4: ROS2 Bridge](b4_ros2_bridge.md)
10. [C1: Draw Square](c1_draw_square.md)

## How To Use These Notes

For each module:

1. Read the note here.
2. Open the matching source file in `../src/`.
3. Inspect the CSV or image artifact mentioned in the note.
4. Then read the long-form explanation in `../blog/` if you want the narrative version.

## Important Scope Note

Some outputs in Lab 1 are conditional:

- CSV logs are generated directly by the Python scripts.
- Plots and GIFs depend on optional libraries such as `matplotlib` or Pillow.
- MuJoCo-specific outputs require the MuJoCo Python package to be installed.

That means a note may describe an artifact that is produced by the script, even if the file is not committed in the current repository snapshot.
