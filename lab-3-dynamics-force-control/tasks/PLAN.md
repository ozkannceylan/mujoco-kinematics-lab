# Lab 3: Dynamics & Force Control — Implementation Plan

## Phase 1: Dynamics Fundamentals

### Step 1.1: Create torque-mode MJCF scene
- Copy Lab 2's `ur5e.xml` and `scene.xml` as starting point
- Verify actuators are `motor` type (direct torque control), not `general`/position
- Add `<compiler angle="radian"/>` if missing
- Add `<option gravity="0 0 -9.81" timestep="0.001"/>`
- Load model in MuJoCo and Pinocchio, print joint names and verify mapping
- **Verify:** `mj_model.nu == 6`, all actuators are torque-mode

### Step 1.2: Create common module (`lab3_common.py`)
- Define Lab 3 paths (models, docs, media)
- Re-export shared constants from Lab 2 pattern (joint names, limits, DH params, quat converters)
- Write `load_mujoco_model()` and `load_pinocchio_model()` for Lab 3 paths
- Write `get_ee_frame_id()` and `get_mj_ee_site_id()` helpers
- **Verify:** Both models load without errors, FK cross-validation passes at Q_HOME

### Step 1.3: Compute and visualize M(q), C(q,q̇), g(q)
- Use Pinocchio: `pin.crba()` for M(q), `pin.computeCoriolisMatrix()` for C, `pin.computeGeneralizedGravity()` for g
- Cross-validate gravity vector: compare `pin.computeGeneralizedGravity(model, data, q)` with `mj_data.qfrc_bias` (at zero velocity, qfrc_bias ≈ g(q))
- Cross-validate mass matrix: compare Pinocchio CRBA vs MuJoCo `mj_fullM()`
- Generate plots: M(q) heatmap, g(q) bar chart per joint, C variation over a trajectory
- **Verify:** Pinocchio g(q) matches MuJoCo `qfrc_bias` within 1% at 5+ configurations

### Step 1.4: Implement gravity compensation controller
- Control law: `τ = g(q)` — compute gravity torque from Pinocchio at each timestep
- Simulation loop: read `mj_data.qpos` → compute `g(q)` → set `mj_data.ctrl = g(q)` → `mj_step()`
- Test: start at Q_HOME, apply gravity comp, arm should "float" in place
- Perturbation test: externally push arm (apply `mj_data.qfrc_applied`), arm should drift then hold new pose
- **Verify:** Joint position drift < 0.01 rad over 5 seconds with gravity comp active

### Step 1.5: Write Phase 1 tests
- `test_dynamics.py`: M(q) symmetry and positive-definiteness, g(q) cross-validation, equations of motion consistency (τ = M*qdd + C*qd + g matches RNEA)
- `test_gravity_comp.py`: gravity comp holds position, perturbation recovery
- **Verify:** All tests pass with `pytest lab-3-dynamics-force-control/tests/`

## Phase 2: Cartesian Impedance Controller

### Step 2.1: Implement task-space impedance controller
- Control law: `F = K_p · (x_d - x) + K_d · (ẋ_d - ẋ) + F_d`
- Joint torques: `τ = J^T · F + g(q)`
- Compute current EE pose (position + orientation) from Pinocchio FK
- Compute current EE velocity from `J · q̇`
- Compute Jacobian from Pinocchio: `pin.computeFrameJacobian()` in LOCAL_WORLD_ALIGNED frame
- Start with translational-only impedance (3D position, K_p = diag(500, 500, 500), K_d = diag(50, 50, 50))
- **Verify:** Arm moves to commanded Cartesian target from Q_HOME

### Step 2.2: Add orientation impedance
- Extend to 6D impedance: position (3) + orientation (3)
- Orientation error via rotation matrix: `e_R = 0.5 * (R_d^T R - R^T R_d)` (skew-symmetric extraction)
- Full 6×6 stiffness and damping matrices
- **Verify:** Arm reaches target pose with correct orientation

### Step 2.3: Tunable compliance demo
- Run same target with 3 stiffness levels: soft (K=100), medium (K=500), stiff (K=2000)
- Apply external perturbation (impulse force at EE) during tracking
- Plot: position tracking error vs time for each stiffness level
- Plot: applied torques vs time for each stiffness level
- Demonstrate: soft = large deflection but low force, stiff = small deflection but high force
- **Verify:** Clear visual and quantitative difference between compliance levels

### Step 2.4: Write Phase 2 tests
- `test_impedance.py`: steady-state position error < 5mm for K=500, orientation error < 2°, stiffness scaling is monotonic
- **Verify:** All tests pass

## Phase 3: Force Control & Contact

### Step 3.1: Add contact surface to scene
- Add a flat table/surface body to the MJCF scene at a known Z height
- Configure contact parameters: `solref` and `solimp` for realistic contact
- Add a force sensor (site-based) at the end-effector to read contact forces
- Add `<sensor><force ... /></sensor>` or use `mj_data.sensordata` for contact force reading
- **Verify:** When EE is pushed into surface, `mj_data.sensordata` reports nonzero force

### Step 3.2: Implement hybrid position-force controller
- Control law: position control in XY plane, force control in Z
- Selection matrix S: `S_p = diag(1,1,0)` for position, `S_f = diag(0,0,1)` for force
- Position part: `F_p = K_p · S_p · (x_d - x) + K_d · S_p · (ẋ_d - ẋ)`
- Force part: `F_f = S_f · (F_d + K_f_i · ∫(F_d - F_measured)dt)`
- Combined: `τ = J^T · (F_p + F_f) + g(q)`
- Use PI controller for force to eliminate steady-state error
- **Verify:** With F_d = 5N in Z, measured force converges to 5N ± 1N

### Step 3.3: Capstone — constant-force line tracing
- EE descends from above to table surface
- Once contact detected (force > threshold), switch to hybrid mode
- Maintain 5N downward force while tracing a straight line in XY
- Record: force profile, position trajectory, torques
- Plot: force vs time (should be ~5N steady), XY path vs desired path
- **Verify:** Force within 5 ± 1N for >90% of trace, XY tracking error < 5mm

### Step 3.4: Write Phase 3 tests
- `test_force_control.py`: force convergence, hybrid mode switching, steady-state force accuracy
- **Verify:** All tests pass

## Phase 4: Documentation & Blog

### Step 4.1: Write English documentation (`docs/`)
- Theory: equations of motion derivation, impedance control, hybrid force/position
- Architecture: module map, data flow diagram
- Results: plots, metrics, capstone demo description
- Include code snippets for key algorithms

### Step 4.2: Write Turkish documentation (`docs-turkish/`)
- Translate docs/ content to Turkish
- Same structure and depth

### Step 4.3: Write blog post
- Focus: "Why force control matters for manipulation"
- Cover: the conceptual leap from position to force control
- Include: capstone demo gif/video, key plots, lessons learned

### Step 4.4: Record capstone demo
- Record video/gif of constant-force line tracing
- Record video/gif of impedance compliance demo (soft vs stiff)
- Save to `media/`

### Step 4.5: Write README.md
- Lab overview, objectives, key results
- How to run demos and tests
- Module descriptions
- Link to docs and blog
