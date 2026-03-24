# Lab 4: Motion Planning & Collision Avoidance — Completion Report

Completion date: 2026-03-17 (canonical stack), updated 2026-03-24 (slalom redesign)

## Platform Lock

Lab 4 is completed on:

- MuJoCo Menagerie `universal_robots_ur5e`
- mounted MuJoCo Menagerie `robotiq_2f85`
- MuJoCo-exact executed geometry for collision truth
- Pinocchio matched to the executed stack for FK, IK, and gravity terms

## Completed Work

### Phase 0: Platform alignment
- Rebased Lab 4 on the same canonical UR5e + Robotiq stack used by Lab 3
- Reused the canonical Menagerie actuator mapping for executed torque commands

### Phase 1: Collision infrastructure
- Collision checker built on the executed MuJoCo geometry
- Preserved the Lab 4 collision-checking API (`is_collision_free`, `is_path_free`, `compute_min_distance`)
- Added `compute_min_obstacle_distance` for clearance-aware planning

### Phase 2: RRT / RRT*
- Kept the planner interface and behavior intact on the canonical stack
- Validated planning success, collision-free waypoints, edge validity, and deterministic seeded behavior

### Phase 3: Path processing and execution
- Preserved shortcutting and `parameterize_topp_ra(...)` with quintic fallback
- Added `densify_path()` to prevent TOPP-RA spline overshoot

### Phase 4: Slalom obstacle-avoidance capstone (2026-03-24)
- Replaced backward-moving capstone with forward slalom through obstacles
- 4 staggered tabletop boxes (10x10x20 cm) at alternating Y positions
- 9 Cartesian waypoints at z=0.56 with gap-midpoint via-points
- Per-segment RRT* planning with bounded sampling region and clearance margin
- 23-seed IK bank for robust collision-free waypoint solving
- Full pipeline: IK → per-segment RRT* → shortcut → densify → TOPP-RA → execute

## Final Validation (2026-03-24)

- Full test suite: `44 passed, 1 skipped`
- Slalom waypoints: 9
- Path waypoints: 24
- Planning time: ~168 s (8 segments)
- Trajectory duration: 15.22 s
- RMS tracking error: 0.0027 rad
- Final position error: 0.0018 rad
- Min waypoint clearance: 0.034 m

## Sign-Off Artifacts

- README: `lab-4-motion-planning/README.md`
- Capstone demo: `lab-4-motion-planning/src/capstone_demo.py`
- Demo video recorder: `lab-4-motion-planning/src/record_lab4_demo.py`
- Validation video recorder: `lab-4-motion-planning/src/record_lab4_validation.py`

## Residual Note

The current Python environment cannot build TOPP-RA from source because a system compiler is unavailable. Lab 4 remains validated because the public timing API is preserved and the fallback time-parameterization respects the configured velocity and acceleration limits under the tested scenarios.
