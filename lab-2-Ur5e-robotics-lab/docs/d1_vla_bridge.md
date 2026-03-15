# D1: VLA Bridge Notes

## Why This Document Exists

The UR5e lab is not just a manipulator exercise. It is a compact way to strengthen the exact layers that matter again in humanoid and VLA systems:

- kinematics consistency,
- stable Cartesian control,
- constraint handling,
- logging and evaluation discipline.

## Direct Connections To VLA Work

### 1. Policy Output Layer

A VLA policy usually expresses intent in task space:

- target pose,
- target waypoint,
- target contact region,
- target sub-goal.

That maps directly onto A4, B1, and B2.

### 2. Controller Layer

A learned policy is not enough on its own.

You still need:

- IK or OSC,
- limit handling,
- collision checks,
- stable low-level execution.

That maps directly onto B2 and B3.

### 3. Data And Evaluation Layer

The lab forces you to log:

- pose error,
- torque magnitude,
- manipulability,
- constraint margin.

That same discipline should be carried into policy evaluation.

## Three Improvements This Lab Suggests For Future VLA Work

1. Keep a stronger analytical baseline around every learned policy output.
2. Log constraint margins explicitly instead of only task success.
3. Evaluate singularity and low-dexterity exposure as first-class metrics.

## Lessons From The Current Repo State

- computed torque already outperforms the simpler PD baseline on the current model
- the pipeline runs end to end without collisions
- IK is usable, but still the main quality bottleneck before a polished demo stage

## What To Add Later

When the rest of the UR5e stack is installed, update this note with:

- exact Pinocchio-vs-MuJoCo comparisons,
- rendered demos,
- a clearer mapping to your broader humanoid/VLA project.
