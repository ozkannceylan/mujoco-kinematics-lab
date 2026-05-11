# Lab 4: Motion Planning & Collision Avoidance — Architecture

Updated: 2026-03-24

## Architecture Status

The canonical Lab 4 architecture is now implemented with a slalom obstacle-avoidance capstone.

- MuJoCo executes the Menagerie UR5e with mounted Robotiq 2F-85
- Collision truth comes from that same executed MuJoCo geometry
- Pinocchio is retained for FK, IK, and gravity terms
- Capstone demonstrates multi-segment RRT* weaving through tabletop obstacles

## Module Map

```
lab-4-motion-planning/
├── src/
│   ├── lab4_common.py             # Constants, obstacles, waypoints, IK seed bank, helpers
│   ├── collision_checker.py       # MuJoCo-geometry collision checking
│   ├── rrt_planner.py             # RRT and RRT* planner
│   ├── trajectory_smoother.py     # Shortcutting + time parameterization
│   ├── trajectory_executor.py     # PD + gravity execution on the real stack
│   ├── capstone_demo.py           # Slalom obstacle-weaving demo
│   ├── record_lab4_demo.py        # Full demo video (metrics + simulation)
│   └── record_lab4_validation.py  # Native MuJoCo validation video
├── models/
├── tests/
├── docs/
├── docs-turkish/
├── media/
└── README.md
```

## Data Flow

```
SLALOM_WAYPOINTS (9 Cartesian targets)
        │
        ▼
solve_ik_xy (Lab 3) + IK seed bank
        │
        └─ 9 collision-free joint-space configs
        │
        ▼
per-segment RRT* (bounded sampling)
        │
        └─ concatenated collision-free path
        │
        ▼
shortcut_path + densify_path + parameterize_topp_ra
        │
        └─ timed trajectory (q, qd, times)
        │
        ▼
trajectory_executor.py (PD + gravity comp)
        │
        └─ MuJoCo execution + telemetry + validation media
```

## Key Interfaces

### `lab4_common.py`

```python
OBSTACLES: tuple[ObstacleSpec, ...]     # 4 staggered boxes
SLALOM_WAYPOINTS: tuple[np.ndarray, ...]  # 9 EE targets at z=0.56
SLALOM_LABELS: tuple[str, ...]
build_ik_seed_bank() -> list[np.ndarray]  # 23-seed IK bank
densify_path(path, max_step) -> list[np.ndarray]
load_mujoco_model(...) -> (model, data)
load_pinocchio_model(...) -> (model, data, ee_fid)
apply_arm_torques(...) -> None
```

### `capstone_demo.py`

```python
ClearanceAwareChecker(base, min_clearance)  # Margin-enforcing wrapper
solve_waypoints(cc) -> (configs, clearances)  # IK for all waypoints
plan_segments(checker, configs) -> (path, time, nodes)  # Per-segment RRT*
run_capstone() -> dict  # Full pipeline: IK → plan → execute → plots
```

### `collision_checker.py`

- `is_collision_free(q)`, `is_path_free(q1, q2)`, `compute_min_distance(q)`
- `compute_min_obstacle_distance(q)` — used by clearance-aware planning

## Video Production

All demo videos use the shared `tools/video_producer.py` three-phase pipeline.

- `src/record_lab4_demo.py` — full demo video (metrics + simulation) via video_producer
- `src/record_lab4_validation.py` — native MuJoCo validation video (slalom execution)
