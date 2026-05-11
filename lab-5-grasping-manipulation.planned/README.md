# Lab 5: Grasping & Manipulation

> **Status: in progress.** Code and documentation in this folder are work-in-progress and not yet portfolio-ready. See the [main README](../README.md) for the published labs.

A pick-and-place pipeline for the UR5e + custom parallel-jaw gripper in MuJoCo. The arm picks a 40mm cube from table position A and places it at position B using RRT* motion planning, DLS IK, and joint-space impedance control with gravity compensation.

---

## Key Results

| Metric | Value |
|--------|-------|
| IK position accuracy | < 0.1 mm |
| Joint tracking error | < 5 mrad |
| Box mass | 150 g |
| Gripper gap (open / closed) | 98 mm / 38 mm |
| Pick success rate | Reliable (fixed scene) |
| Planning time per segment | 200–600 ms (RRT*, 6000 iter) |

---

## Module Map

```
lab-5-grasping-manipulation/
├── models/
│   ├── ur5e_gripper.xml      UR5e (6 torque motors) + parallel-jaw gripper (1 position actuator)
│   └── scene_grasp.xml       includes ur5e_gripper.xml + table + box (freejoint) + target pad
│
├── src/
│   ├── lab5_common.py         Paths, constants, model loading (MuJoCo + Pinocchio), helpers
│   ├── gripper_controller.py  open/close/settle/contact detection API
│   ├── grasp_planner.py       DLS IK, GraspConfigs dataclass, compute_grasp_configs()
│   ├── grasp_state_machine.py GraspStateMachine: 11-state pick-and-place orchestrator
│   └── pick_place_demo.py     Capstone demo: runs full cycle, saves plots to media/
│
├── tests/
│   ├── test_gripper.py        Phase 1+2: scene loads, gripper open/close, contact detection, IK
│   ├── test_grasp_planner.py  Phase 2: IK accuracy, grasp config positions, joint limits
│   └── test_state_machine.py  Phase 3: collision checker, config integration, RRT* + TOPP-RA
│
├── docs/
│   ├── 01_contact_physics.md  condim, friction, solref, solimp explained
│   ├── 02_gripper_design.md   MJCF structure, equality constraint, position control
│   ├── 03_grasp_pipeline.md   State machine, IK flow, Lab 3+4 integration
│   └── 04_pick_place_results.md Timing, accuracy, contact analysis, known limits
│
├── docs-turkish/              Turkish translations of all 4 docs
├── blog/
│   └── lab5_blog_post.md      "Building a Pick-and-Place Pipeline from Scratch"
└── tasks/
    ├── PLAN.md                Implementation plan (4 phases, 14 steps)
    ├── ARCHITECTURE.md        Module map, data flow, key interfaces
    ├── TODO.md                Progress tracking (all phases complete)
    └── LESSONS.md             Bugs found, debug strategies, key insights
```

---

## How to Run

### Prerequisites

```bash
pip install mujoco pinocchio toppra numpy matplotlib
```

### Run all tests

```bash
# From repo root
python3 -m pytest lab-5-grasping-manipulation/tests/ -v
```

Expected: **33 passed**.

### Run the pick-and-place demo

```bash
cd lab-5-grasping-manipulation/src
python3 pick_place_demo.py
```

The demo:
1. Loads the MuJoCo scene and Pinocchio arm model
2. Computes all 4 grasp configurations via DLS IK
3. Runs the full pick-and-place state machine
4. Saves trajectory plots to `media/`
5. Prints state transition timing to console

---

## Architecture

```
Pinocchio (FK, DLS IK)
        ↓ GraspConfigs (5 × q)
GraspStateMachine
        ↓ each PLAN_* state:
Lab 4 RRT* + shortcutting → waypoints
Lab 4 TOPP-RA             → (times, q_traj, qd_traj)
        ↓ each EXEC_* state:
Lab 3 compute_impedance_torque (Kp·Δq + Kd·Δqd + g(q))
        ↓ ctrl torques
MuJoCo mj_step()
        ↓ qpos, qvel, contact forces
```

**Key design decision:** Pinocchio handles all analytical computation (FK, IK, Jacobians, gravity); MuJoCo handles simulation and contact. Never duplicate computation between the two.

---

## Scene Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `BOX_A_POS` | [0.35, +0.20, 0.335] m | Pick location |
| `BOX_B_POS` | [0.35, -0.20, 0.335] m | Place location |
| `GRIPPER_TIP_OFFSET` | 0.090 m | tool0 origin → fingertip center |
| `PREGRASP_CLEARANCE` | 0.150 m | Height above box for approach |
| `GRIPPER_OPEN` | 0.030 m (ctrl[6]) | Finger slide joint open setpoint |
| `GRIPPER_CLOSED` | 0.000 m (ctrl[6]) | Finger slide joint closed setpoint |
| `TABLE_TOP_Z` | 0.315 m | Table surface world Z |

---

## Cross-Lab Dependencies

| Component | Source |
|-----------|--------|
| `ur5e.urdf` (Pinocchio model) | `lab-3-dynamics-force-control/models/` |
| `CollisionChecker` | `lab-4-motion-planning/src/` |
| `RRTStarPlanner`, `shortcut_path` | `lab-4-motion-planning/src/` |
| `parameterize_topp_ra` | `lab-4-motion-planning/src/` |
| `compute_impedance_torque`, `ImpedanceGains` | `lab-3-dynamics-force-control/src/` |

Imports are managed via `add_lab_src_to_path()` in `lab5_common.py`.

---

## Known Issues

See `tasks/LESSONS.md` for full details. Summary:

- **Gripper minimum gap must be < object half-width**: verified geometrically before running any test
- **`is_gripper_in_contact` checks all finger geoms**, not just pads — finger body geoms contact first
- **Contact tests check during closing** (not after 1s settling) — box falls under gravity with no arm torque
- **`parameterize_topp_ra` returns 4-tuple** `(t, q, qd, qdd)` — unpack with `_` for unused `qdd`
