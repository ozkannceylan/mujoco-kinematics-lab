# Portfolio Package Notes

## Current Strengths

- complete lab-2 folder structure exists
- local URDF and MJCF assets are in the repo
- every phase script writes concrete CSV artifacts
- English and Turkish study notes exist in parallel

## Current Quantitative Snapshot

- A4 IK benchmark: `16 / 20` successful reachable-pose solves
- B2 computed torque RMS EE error: `0.00358264`
- C1 singularity stress RMS EE error: `0.00369054`
- C1 demos currently show zero collision samples in the logged runs

## What Is Still Missing For A Public Showcase

1. Install MuJoCo and Pinocchio, then regenerate all cross-check metrics.
2. Replace the branch-seeded IK scaffold with exact analytical UR5e IK.
3. Improve general-motion tracking in C1 before recording demo videos.
4. Add rendered media into `media/`.
5. Connect the ROS 2 bridge to a real ROS 2 runtime.

## Recommended README Structure Later

1. Overview
2. Why UR5e + MuJoCo + Pinocchio
3. Architecture diagram
4. Key metrics table
5. Demo clips
6. How to run
7. Folder structure
8. VLA / humanoid connection

## How To Use This Note

Treat this as the gap list between the current study-first repo state and the final portfolio-ready version.
