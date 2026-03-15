# C1: Integrated Pick-and-Place Pipeline

## Goal

Combine the previous modules into a complete manipulation pipeline: solve Cartesian waypoints with IK, connect them with smooth joint trajectories, execute them in MuJoCo with model-based control, and log safety and tracking metrics.

## Files

- Script: `src/c1_pick_and_place.py`
- Key dependencies: `src/a4_inverse_kinematics.py`, `src/b1_trajectory_generation.py`, `src/b2_control_hierarchy.py`, `src/b3_constraints.py`, `src/mujoco_sim.py`
- Main outputs written by the script:
  - `docs/c1_pick_place_log.csv`
  - `docs/c1_circle_log.csv`
  - `docs/c1_metrics.csv`

Older exploratory logs may also be present in this folder, but the current core script writes the three files above.

## Implementation

### Core data structures

- `WaypointPose` stores a named Cartesian target: position plus rotation.
- `PipelineLog` records time, phase name, end-effector error, torque norm, joint-limit margin, and self-collision score.

### Pipeline Stages

`run_pipeline()` executes the same four-stage workflow for each demo:

1. **Solve IK for each waypoint**
   `solve_waypoints()` calls `ik_damped_least_squares()` and warm-starts each target from the previous solution.
2. **Generate a joint-space trajectory**
   `generate_pipeline_trajectory()` chains quintic segments between the solved joint waypoints.
3. **Execute in MuJoCo**
   The trajectory is tracked with `computed_torque_control()` and then clipped through `saturate_torques()`.
4. **Log metrics**
   The loop records tracking quality and safety margins at each step.

## Demo 1: Pick and Place

The first scenario keeps the home orientation fixed and visits six named stages:

1. `approach`
2. `pick`
3. `lift`
4. `move`
5. `place`
6. `retreat`

Each segment is executed over 1.5 seconds.

## Demo 2: Circle Tracking

The second scenario generates eight points on a Cartesian circle and returns to the start:

- Center: `[0.4, 0.0, 0.50]`
- Radius: `0.08 m`
- Segment duration: `1.0 s`

This demo is useful because it forces the IK solver to behave well across many closely spaced waypoints instead of just a few hand-picked poses.

## What Gets Logged

Per-step logs contain:

- `time`
- `phase`
- `ee_error`
- `torque_norm`
- `joint_margin`
- `collision_score`

The summary CSV stores, for each demo:

- RMS end-effector error
- maximum torque norm
- minimum joint-limit margin
- minimum self-collision distance

## Why This Module Matters

C1 is the point where the lab stops being a collection of isolated robotics topics. The kinematics, dynamics, trajectory generation, control, and safety code all have to coexist in one real loop.

That is exactly why it is worth studying:

- chained IK exposes solver continuity problems,
- quintic trajectory generation exposes timing and interpolation assumptions,
- computed torque exposes whether the dynamic model is good enough,
- safety logging exposes whether the motion is only accurate or also usable.

## How to Run

```bash
python3 src/c1_pick_and_place.py
```

## What to Study

1. Start with the waypoint list and see how Cartesian intent becomes joint-space motion.
2. Follow the computed-torque loop line by line and identify where A4, A5, B1, B2, and B3 enter the same control cycle.
3. Plot `ee_error` over time and check whether transitions between segments create spikes.
4. Compare the `pick_place` and `circle` rows in `c1_metrics.csv`; the circle task is usually the better stress test for solver continuity.
5. Check that joint margins stay positive and that the collision score does not collapse toward zero.

## Next Step

Move to C2 to see how the same MuJoCo-backed lab can be exposed to ROS 2 topics and then to C3 for the final full-pipeline drawing demo.
