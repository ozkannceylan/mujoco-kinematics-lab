# Lab 6: Dual-Arm Coordination — Architecture

## Module Map

| File | Purpose |
|------|---------|
| `src/lab6_common.py` | Paths, constants, MuJoCo/Pinocchio loading for two arms |
| `src/dual_arm_ik.py` | DLS IK for each arm; `BimanualGraspConfigs` |
| `src/coordination_layer.py` | Trajectory synchronization; `ObjectFrame` abstraction |
| `src/bimanual_controller.py` | Dual-arm impedance controller (one MuJoCo step = both arms) |
| `src/bimanual_state_machine.py` | Pick-carry-place state machine for bimanual task |
| `src/bimanual_demo.py` | Entry point: run demo, log data, save plots |
| `models/dual_arm_scene.xml` | Self-contained MJCF scene with two UR5e+gripper trees |
| `tests/test_scene.py` | Verify model loads, actuator/joint counts |
| `tests/test_ik.py` | IK convergence for all 8 target configs |
| `tests/test_coordination.py` | Trajectory sync, independent motion |
| `tests/test_bimanual_demo.py` | Full-cycle smoke test |

---

## Data Flow

```
bimanual_demo.py
    │
    ├── load_mujoco_model()  ──────────────► (mj_model, mj_data)
    ├── load_pinocchio_model()  ───────────► (pin_L, data_L, ee_L), (pin_R, data_R, ee_R)
    │
    ├── compute_bimanual_configs(pin_L, pin_R, ...) ─► BimanualGraspConfigs
    │       uses: dual_arm_ik.compute_ik()
    │
    ├── BimanualStateMachine(mj_model, mj_data, pins, cfgs)
    │       │
    │       ├── _plan_and_smooth(q_start, q_goal, arm='left'|'right')
    │       │       uses: Lab4 CollisionChecker, RRTStarPlanner, TOPP-RA
    │       │
    │       ├── sync_trajectories(left_traj, right_traj)
    │       │       uses: coordination_layer
    │       │
    │       ├── BimanualController.step(mj_data, q_d_L, qd_d_L, q_d_R, qd_d_R)
    │       │       uses: bimanual_controller
    │       │       calls: pin.computeGeneralizedGravity (each arm independently)
    │       │
    │       └── logs: time, q_L, q_R, ee_L, ee_R, bar_pos, gripper_L, gripper_R
    │
    └── save plots to media/
```

---

## Key Interfaces

### `lab6_common.py`

```python
# Arm indices in qpos / ctrl
LEFT_QPOS  = slice(0, 6)    # left arm joint angles
LEFT_QFINGER_L = 6          # left arm left finger qpos index
LEFT_QFINGER_R = 7          # left arm right finger qpos index (equality-mirrored)
RIGHT_QPOS = slice(8, 14)   # right arm joint angles
RIGHT_QFINGER_L = 14
RIGHT_QFINGER_R = 15

LEFT_CTRL  = slice(0, 6)    # left arm motor ctrls
LEFT_GRIP_CTRL  = 6         # left gripper position actuator
RIGHT_CTRL = slice(7, 13)   # right arm motor ctrls
RIGHT_GRIP_CTRL = 13        # right gripper position actuator

def load_mujoco_model() -> tuple[MjModel, MjData]: ...
def load_pinocchio_model() -> tuple[tuple, tuple]:
    # returns (pin_L_model, pin_L_data, ee_L_fid),
    #         (pin_R_model, pin_R_data, ee_R_fid)
```

### `dual_arm_ik.py`

```python
@dataclass
class BimanualGraspConfigs:
    left: GraspConfigs   # q_home, q_pregrasp, q_grasp, q_preplace, q_place, R_topdown
    right: GraspConfigs

def compute_ik(pin_model, pin_data, ee_fid, x_target, R_target, q_init, ...) -> np.ndarray | None

def compute_bimanual_configs(
    pin_L, data_L, ee_fid_L,
    pin_R, data_R, ee_fid_R,
    bar_pos: np.ndarray = BAR_PICK_POS,
    bar_place_pos: np.ndarray = BAR_PLACE_POS,
) -> BimanualGraspConfigs
```

### `coordination_layer.py`

```python
def sync_trajectories(
    traj_L: tuple[np.ndarray, np.ndarray, np.ndarray],  # (times, q, qd)
    traj_R: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # returns (times_sync, q_L, qd_L, q_R, qd_R) — same time axis for both

@dataclass
class ObjectFrame:
    position: np.ndarray   # (3,) world position of object center
    rotation: np.ndarray   # (3,3) rotation matrix
    def ee_target_left(self, offset: np.ndarray) -> np.ndarray: ...
    def ee_target_right(self, offset: np.ndarray) -> np.ndarray: ...
```

### `bimanual_controller.py`

```python
class BimanualController:
    def __init__(self, pin_L, data_L, pin_R, data_R, Kp=400.0, Kd=40.0): ...
    def step(
        self,
        mj_data,
        q_d_L: np.ndarray, qd_d_L: np.ndarray,
        q_d_R: np.ndarray, qd_d_R: np.ndarray,
    ) -> None:
        # Computes tau = Kp*(q_d-q) + Kd*(qd_d-qd) + g(q) for each arm
        # Writes to mj_data.ctrl[LEFT_CTRL] and mj_data.ctrl[RIGHT_CTRL]
```

---

## Model Files

| File | Source | Notes |
|------|--------|-------|
| `models/dual_arm_scene.xml` | New (hand-crafted) | Monolithic MJCF; two UR5e+gripper trees with `left_` / `right_` prefixes |
| `lab-3-dynamics-force-control/models/ur5e.urdf` | Lab 3 | Pinocchio arm model (both arms reuse same URDF, loaded separately) |

---

## Dependencies on Previous Labs

| From Lab | Import | Used for |
|----------|--------|---------|
| Lab 3 | `b1_impedance_controller.ImpedanceGains, compute_impedance_torque` | Cartesian impedance during descend/lift |
| Lab 3 | `lab3_common.clip_torques` | Torque saturation |
| Lab 4 | `collision_checker.CollisionChecker` | Arm self-collision + table avoidance |
| Lab 4 | `rrt_planner.RRTStarPlanner` | Approach/transport trajectory planning |
| Lab 4 | `trajectory_smoother.parameterize_topp_ra, shortcut_path` | Time-optimal trajectory |
| Lab 4 | `lab4_common.ObstacleSpec` | Table spec |
| Lab 5 | `grasp_planner.GraspConfigs` | Reused dataclass |
| Lab 5 | `gripper_controller.*` | Gripper open/close/contact logic (adapted for dual arm) |

Cross-lab imports use `add_lab_src_to_path()` (defined in `lab6_common.py`).

---

## MuJoCo Scene Layout

```
worldbody
├── floor
├── left_base  (pos="0 -0.35 0")
│   └── left_shoulder → ... → left_wrist_3
│       └── left_tool0 → left_gripper_base
│           ├── left_left_finger (joint: left_left_finger_joint)
│           └── left_right_finger (joint: left_right_finger_joint)
├── right_base  (pos="0 0.35 0")
│   └── right_shoulder → ... → right_wrist_3
│       └── right_tool0 → right_gripper_base
│           ├── right_left_finger
│           └── right_right_finger
├── table
├── carry_bar  (freejoint — the cooperative carry object)
└── target_pad
```

Actuators (14 total):
- [0-5]: left arm motors
- [6]: left gripper (position actuator)
- [7-12]: right arm motors
- [13]: right gripper (position actuator)

Equality (2):
- left_finger_mirror: left_left_finger_joint = left_right_finger_joint
- right_finger_mirror: right_left_finger_joint = right_right_finger_joint

---

## Key Design Decisions

1. **Two separate Pinocchio models** (not a combined model): simpler, each arm uses Lab 3's `ur5e.urdf`, no need to build a custom dual-arm URDF. FK/IK/dynamics computed independently.

2. **Object-centric frame**: The bar's world pose defines both arm targets. This prevents drift: if one arm slips, the other stays anchored to the object frame.

3. **Synchronized trajectory timing**: `sync_trajectories()` pads the shorter trajectory (interpolate at final config) so both arms start and finish at the same time. This is critical — an arm arriving at the grasp point 500ms before its partner means one arm is pushing air while the other approaches.

4. **Symmetric impedance**: Both arms use the same Kp/Kd gains. Equal stiffness on both sides means neither arm dominates, avoiding unbalanced forces on the bar.

5. **No RRT arm-arm collision**: For this lab, the two arms work on OPPOSITE X-ENDS of the bar (left at x≈0.35, right at x≈0.55) and their workspaces barely overlap. Simple bounding-box checks suffice; full arm-arm HPP-FCL checking adds complexity without benefit at this geometry.
