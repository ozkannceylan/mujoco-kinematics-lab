# Lab 2 Blog Series

This folder contains the long-form writeups for the UR5e lab.

If you want the shortest path through the project, use this order:

1. [Lab 2 README](../README.md)
2. The matching module note in [`../docs/`](../docs/)
3. The matching source file in [`../src/`](../src/)
4. The corresponding blog post below

## Posts

| Part | Topic | Companion Note | Source |
|---|---|---|---|
| 1 | Model loading, architecture, and validation | [A1 note](../docs/a1_environment_setup.md) | [a1_model_setup.py](../src/a1_model_setup.py), [ur5e_common.py](../src/ur5e_common.py), [mujoco_sim.py](../src/mujoco_sim.py) |
| 2 | FK, DH frames, Jacobians, and singularities | [A2 note](../docs/a2_forward_kinematics.md), [A3 note](../docs/a3_jacobian.md) | [a2_forward_kinematics.py](../src/a2_forward_kinematics.py), [a3_jacobian.py](../src/a3_jacobian.py) |
| 3 | Numerical IK and why damping matters | [A4 note](../docs/a4_inverse_kinematics.md) | [a4_inverse_kinematics.py](../src/a4_inverse_kinematics.py) |
| 4 | Dynamics, trajectory generation, and controller hierarchy | [A5 note](../docs/a5_dynamics.md), [B1 note](../docs/b1_trajectory_generation.md), [B2 note](../docs/b2_control_hierarchy.md), [B3 note](../docs/b3_constraints.md) | [a5_dynamics.py](../src/a5_dynamics.py), [b1_trajectory_generation.py](../src/b1_trajectory_generation.py), [b2_control_hierarchy.py](../src/b2_control_hierarchy.py), [b3_constraints.py](../src/b3_constraints.py) |
| 5 | Integrated demos, ROS 2 bridge, and the cube showcase | [C1 note](../docs/c1_full_pipeline.md), [C2 note](../docs/c2_ros2_bridge.md), [C3 note](../docs/c3_draw_cube.md) | [c1_pick_and_place.py](../src/c1_pick_and_place.py), [c2_ros2_bridge.py](../src/c2_ros2_bridge.py), [c3_draw_cube.py](../src/c3_draw_cube.py), [c3_record_video.py](../src/c3_record_video.py) |

## Direct Links

- [Part 1](2026-03-14-ur5e-robotics-lab-part-1.md)
- [Part 2](2026-03-14-ur5e-robotics-lab-part-2.md)
- [Part 3](2026-03-14-ur5e-robotics-lab-part-3.md)
- [Part 4](2026-03-14-ur5e-robotics-lab-part-4.md)
- [Part 5](2026-03-14-ur5e-robotics-lab-part-5.md)
