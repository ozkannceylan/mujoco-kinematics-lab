# Lab 2 Docs Index

This folder is the ordered study track for the UR5e lab. The notes are written to be used next to the source tree, not as isolated theory summaries.

## Before You Start

- These notes assume you are inside `lab-2-Ur5e-robotics-lab/`.
- The lab is not dependency-light anymore. `numpy` is required across the source tree, and most modules also need `mujoco` and `pinocchio`.
- `c2_ros2_bridge.py` has a scaffold mode without ROS 2, but full bridge execution still needs a sourced ROS 2 setup and its message packages.

## Recommended Study Order

| Step | Note | Main Script | Main Artifact |
|---|---|---|---|
| 1 | `a1_environment_setup.md` | `src/a1_model_setup.py` | `a1_environment_status.csv` |
| 2 | `a2_forward_kinematics.md` | `src/a2_forward_kinematics.py` | `a2_fk_validation.csv` |
| 3 | `a3_jacobian.md` | `src/a3_jacobian.py` | `a3_jacobian_validation.csv`, `a3_manipulability_heatmap.csv`, `a3_singularity_cases.csv` |
| 4 | `a4_inverse_kinematics.md` | `src/a4_inverse_kinematics.py` | `a4_ik_benchmark.csv` |
| 5 | `a5_dynamics.md` | `src/a5_dynamics.py` | `a5_dynamics_snapshot.csv` |
| 6 | `b1_trajectory_generation.md` | `src/b1_trajectory_generation.py` | `b1_*.csv` |
| 7 | `b2_control_hierarchy.md` | `src/b2_control_hierarchy.py` | `b2_*.csv` |
| 8 | `b3_constraints.md` | `src/b3_constraints.py` | `b3_*.csv` |
| 9 | `c1_full_pipeline.md` | `src/c1_pick_and_place.py` | `c1_pick_place_log.csv`, `c1_circle_log.csv`, `c1_metrics.csv` |
| 10 | `c2_ros2_bridge.md` | `src/c2_ros2_bridge.py` | runtime-only scaffold output |
| 11 | `c3_draw_cube.md` | `src/c3_draw_cube.py`, `src/c3_record_video.py` | `media/c3_draw_cube.gif`, `media/c3_draw_cube.mp4` |
| 12 | `d1_vla_bridge.md` | architecture note | none |
| 13 | `interview_cheatsheet.md` | review note | none |
| 14 | `portfolio_package.md` | packaging note | none |

## Code Map

- Scripts: `src/`
- Models: `models/`
- Media: `media/`
- Tests: `tests/`
- Blog writeups: `blog/`

## Artifact Notes

- The CSV files in this folder are versioned snapshots from earlier runs.
- The core A1-C3 scripts write the artifacts listed above.
- You may also see exploratory logs such as `c1_multi_waypoint_log.csv` or `c1_singularity_log.csv`. Treat those as side experiments, not the primary study path.

## How To Use This Folder

1. Read the module note.
2. Open the matching script in `src/`.
3. Inspect the artifact CSV or media file.
4. Read the matching blog post in `blog/` if you want the longer engineering narrative.
5. Re-run the script once your local dependencies are installed.

That flow keeps the mathematical intent, implementation details, and recorded results tightly coupled.
