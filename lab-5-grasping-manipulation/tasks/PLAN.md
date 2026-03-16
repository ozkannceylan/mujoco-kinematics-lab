# Lab 5: Grasping & Manipulation — Implementation Plan

## Phase 1: Gripper Integration

### Step 1.1: Build MJCF scene with UR5e + parallel-jaw gripper
- Create `models/ur5e_gripper.xml`: UR5e arm (torque-controlled) + custom 2-finger
  parallel jaw gripper (position-controlled) attached to tool0
- Create `models/scene_grasp.xml`: includes ur5e_gripper.xml + table + graspable box
  (free joint body) + target marker
- Gripper: two sliding fingers (left/right), equality-mirrored, one position actuator
- **Output:** Scene loads in MuJoCo; gripper visible at arm tip; box sitting on table

### Step 1.2: Create common module (`lab5_common.py`)
- Paths: SCENE_PATH, URDF_PATH (Lab 3 arm-only URDF), MEDIA_DIR
- Constants: NUM_JOINTS=6, GRIPPER_IDX=6, DT=0.001
- Scene geometry: TABLE spec, BOX_A_POS, BOX_B_POS, GRIPPER_TIP_OFFSET
- Model loading helpers: load_mujoco_model, load_pinocchio_model (arm only)
- R_TOPDOWN: target EE orientation for top-down grasps (computed from FK at Q_HOME)
- **Output:** Module importable by all Lab 5 scripts

### Step 1.3: Implement gripper controller (`gripper_controller.py`)
- `open_gripper(ctrl, open_pos=0.030)`: set ctrl[GRIPPER_IDX] = open_pos
- `close_gripper(ctrl)`: set ctrl[GRIPPER_IDX] = 0.0
- `wait_for_gripper(model, data, timeout_s)`: step sim until finger velocity < threshold
- `is_gripper_in_contact(model, data)`: check if finger pad geoms touch any object
- **Output:** Open/close verified on standalone arm at Q_HOME

### Step 1.4: Write Phase 1 tests
- `test_gripper.py`: scene loads, gripper opens/closes, contact detection works
- **Verify:** All tests pass

## Phase 2: Contact Physics Tuning

### Step 2.1: Tune contact parameters
- Experiment with box condim (3 vs 4), friction, solref, solimp
- Test: arm picks up box, moves, box does NOT slip or fly off
- Document each parameter's effect in LESSONS.md
- Target: box held securely at 30° arm tilt with no slippage
- **Output:** Verified contact params in scene_grasp.xml; documented in LESSONS.md

### Step 2.2: IK solver for grasp configurations (`grasp_planner.py`)
- `compute_ik(model, data, ee_fid, x_target, R_target, q_init)`: DLS iterative IK
- Returns q_solution or None if IK diverges
- `compute_grasp_configs(box_pos)`: returns dict with keys:
  - `q_pregrasp`: above box (15 cm above, fingers aligned)
  - `q_grasp`: at box center height (fingers at grasp level)
  - `q_preplace`: above target (15 cm above)
  - `q_place`: at target height
  - `q_home`: Q_HOME
- Uses GRIPPER_TIP_OFFSET to correctly offset tool0 target from box center
- **Output:** IK configs verified collision-free via MuJoCo forward step + ncon check

### Step 2.3: Write Phase 2 tests
- `test_grasp_planner.py`: IK convergence, EE position accuracy, configs collision-free
- **Verify:** All tests pass

## Phase 3: Pick and Place Pipeline

### Step 3.1: Implement grasp state machine (`grasp_state_machine.py`)
- Class `GraspStateMachine` with states:
  - IDLE → PLAN_APPROACH → EXEC_APPROACH → DESCEND → CLOSE → LIFT
  - → PLAN_TRANSPORT → EXEC_TRANSPORT → DESCEND_PLACE → RELEASE → RETRACT → DONE
- PLAN_*: calls Lab 4 RRT* + shortcutting + TOPP-RA (collision-free path)
- EXEC_*: runs joint-space impedance controller (Lab 3 pattern: Kp*(q_d-q) + g(q))
- DESCEND/LIFT: Cartesian impedance Z-axis motion (Lab 3 compute_impedance_torque)
- CLOSE/RELEASE: sets ctrl[6] and waits for finger settlement
- `run(box_start_pos, box_target_pos) -> dict`: full simulation loop, returns log
- **Output:** State machine drives simulation step-by-step, box moves from A to B

### Step 3.2: Run pick and place demo (`pick_place_demo.py`)
- Load scene, run full pick-and-place cycle
- Log: arm q, EE position, gripper position, contact forces, state timeline
- Plot: EE trajectory in 3D, joint tracking error, gripper vs contact
- Save plots to media/
- Print: each state transition with timing
- **Output:** Box at target position, plots saved, clear console output

### Step 3.3: Write Phase 3 tests
- `test_state_machine.py`: state transitions correct, box moves, no crashes
- **Verify:** All tests pass

## Phase 4: Documentation & Blog

### Step 4.1: Write English documentation (`docs/`)
- `01_contact_physics.md`: condim, friction, solref/solimp — what each does, values used
- `02_gripper_design.md`: parallel jaw mechanics, MJCF equality constraint, position control
- `03_grasp_pipeline.md`: state machine, IK flow, approach/grasp/lift/transport/place
- `04_pick_place_results.md`: plots, contact analysis, failure modes

### Step 4.2: Write Turkish documentation (`docs-turkish/`)
- Translate docs/ to Turkish (same 4 files)

### Step 4.3: Write blog post
- "Building a Pick-and-Place Pipeline from Scratch"
- Cover: why grasping is hard, contact modeling, state machines, Lab 3+4 integration

### Step 4.4: Write README.md
- Lab overview, module map, how to run the demo, key results table
