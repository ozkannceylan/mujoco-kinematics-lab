# Lab 6: Dual-Arm Coordination — Implementation Plan

## Phase 1: Dual-Arm Setup

### Step 1.1: Create MJCF dual-arm scene
- Build `models/dual_arm_scene.xml` with two UR5e+gripper body trees (prefixed `left_` and `right_`)
- Left arm base: `(0, -0.35, 0)`, right arm base: `(0, 0.35, 0)`
- Table at `(0.45, 0, 0.300)`, large bar at `(0.45, 0, 0.335)` (240×50×40 mm)
- 14 actuators: left [0-5] arm + [6] gripper, right [7-12] arm + [13] gripper
- Expected output: model loads, `mj_model.nu == 14`

### Step 1.2: Create `lab6_common.py`
- Paths, constants, qpos/ctrl index maps for left and right arms
- `Q_HOME_LEFT`, `Q_HOME_RIGHT` (mirrored shoulder_pan)
- `load_mujoco_model()`, `load_pinocchio_model()` returning two Pinocchio (model, data, ee_fid) tuples
- Expected output: both models load, FK at Q_HOME gives plausible EE positions

### Step 1.3: Create `dual_arm_ik.py`
- Reuse DLS IK from Lab 5 (parameterized, not copy-paste)
- `BimanualGraspConfigs` dataclass: left + right `GraspConfigs`
- `compute_bimanual_configs()`: derive left/right IK targets from object-centric frame
  - Bar pick: left at `(0.35, 0, bar_z)`, right at `(0.55, 0, bar_z)` — each grips one X-end
  - Bar place: same X offset, moved `+0.20m` in Y
- Expected output: IK converges for both arms, position error < 5 mm

### Step 1.4: Tests — Phase 1
- `tests/test_scene.py`: model loads, actuator/joint counts correct
- `tests/test_ik.py`: IK converges for all 8 configs (2 arms × 4 phases)

---

## Phase 2: Coordinated Motion

### Step 2.1: Create `coordination_layer.py`
- `sync_trajectories(left_traj, right_traj)`: time-normalize two TOPP-RA trajectories to share the same duration (max of both)
- `ObjectFrame` dataclass: (position, orientation) of the shared object; derives each arm's EE target
- Expected output: synced trajs have same length, same duration

### Step 2.2: Create `bimanual_controller.py`
- `BimanualController`: holds two sets of impedance gains + arm indices
- `step(mj_data, q_d_left, qd_d_left, q_d_right, qd_d_right)`: computes and applies torques for BOTH arms in a single MuJoCo step
- Gravity compensation from Pinocchio for each arm independently
- Expected output: both arms track a held joint config with < 5 mm EE error

### Step 2.3: Tests — Phase 2
- `tests/test_coordination.py`: trajectory sync, independent arm motion without collision

---

## Phase 3: Cooperative Manipulation

### Step 3.1: Create `bimanual_state_machine.py`
- States (mirroring Lab 5 with dual-arm coordination):
  ```
  IDLE → PLAN_APPROACH → EXEC_APPROACH → DESCEND → CLOSE
       → LIFT → PLAN_TRANSPORT → EXEC_TRANSPORT
       → DESCEND_PLACE → RELEASE → RETRACT → DONE
  ```
- Both arms execute EACH state simultaneously; timing is synchronized
- Gripper actions: left gripper ctrl[6], right gripper ctrl[13]
- Logging: left/right EE pos, bar body pose, gripper states
- Expected output: full cycle completes, bar moves from A to B

### Step 3.2: Create `bimanual_demo.py`
- Headless simulation run (no viewer) with matplotlib plots at end
- Plots: both EE trajectories, bar trajectory, gripper states vs time
- Expected output: all plots saved to `media/`

### Step 3.3: Tests — Phase 3
- `tests/test_bimanual_demo.py`: smoke-test one full cycle headless

---

## Phase 4: Documentation & Blog

### Step 4.1: Write `docs/LAB_06.md`
- Theory: dual-arm kinematics, object-centric frames, symmetric impedance
- Architecture with module diagram
- Results with plots and metrics

### Step 4.2: Write `docs-turkish/LAB_06_TR.md`
- Turkish translation of LAB_06.md

### Step 4.3: Write `blog/lab_06_dual_arm.md`
- Public-facing blog post: "From One Arm to Two — The Coordination Challenge"

### Step 4.4: Write `README.md`
- Lab overview, how to run, key results
