# Lab 3: Dynamics & Force Control — Architecture

## Module Map

```
lab-3-dynamics-force-control/
├── src/
│   ├── lab3_common.py              # Paths, constants, model loaders, utilities
│   ├── a1_dynamics_fundamentals.py # Compute & visualize M(q), C(q,q̇), g(q)
│   ├── a2_gravity_compensation.py  # Gravity comp controller + perturbation demo
│   ├── b1_impedance_controller.py  # Cartesian impedance (position + orientation)
│   ├── b2_compliance_demo.py       # Tunable stiffness comparison (soft/medium/stiff)
│   ├── c1_force_control.py         # Hybrid position-force controller
│   ├── c2_capstone_force_trace.py  # Constant-force line tracing on table
│   └── c2_record_video.py          # Headless video recording of capstone
├── models/
│   ├── ur5e.xml                    # UR5e with torque-mode actuators (from Lab 2)
│   ├── scene_torque.xml            # Lab 3 scene: robot + floor (Phase 1-2)
│   └── scene_table.xml             # Lab 3 scene: robot + table surface (Phase 3)
├── tests/
│   ├── test_dynamics.py            # M, C, g cross-validation, EOM consistency
│   ├── test_gravity_comp.py        # Gravity comp holds position
│   ├── test_impedance.py           # Impedance tracking accuracy
│   └── test_force_control.py       # Force convergence, hybrid control
├── docs/                           # English documentation
├── docs-turkish/                   # Turkish documentation
├── media/                          # GIFs, videos, plots
└── README.md
```

## Data Flow

```
                    ┌─────────────────────────────────┐
                    │         lab3_common.py           │
                    │  - Model loading (Pin + MuJoCo)  │
                    │  - Paths, constants, quat utils  │
                    └──────────┬──────────────────────┘
                               │ imported by all modules
            ┌──────────────────┼──────────────────────┐
            ▼                  ▼                       ▼
   Phase A (Dynamics)   Phase B (Impedance)    Phase C (Force)
   ┌──────────────┐    ┌─────────────────┐    ┌──────────────────┐
   │ a1_dynamics   │    │ b1_impedance    │    │ c1_force_control │
   │ a2_grav_comp  │    │ b2_compliance   │    │ c2_capstone      │
   └──────┬───────┘    └────────┬────────┘    └────────┬─────────┘
          │                     │                       │
          ▼                     ▼                       ▼
   ┌──────────────────────────────────────────────────────────┐
   │                    Control Loop (shared pattern)          │
   │                                                           │
   │  1. Read state: q, q̇ ← mj_data.qpos, mj_data.qvel      │
   │  2. Pinocchio FK: x, R ← pin.forwardKinematics(q)       │
   │  3. Pinocchio Jacobian: J ← pin.computeFrameJacobian(q) │
   │  4. Pinocchio dynamics: g ← pin.computeGeneralizedGravity│
   │  5. Compute control torque: τ = J^T·F + g(q)            │
   │  6. Apply: mj_data.ctrl = τ                              │
   │  7. Step: mujoco.mj_step(mj_model, mj_data)            │
   └──────────────────────────────────────────────────────────┘
```

### Controller Hierarchy (each builds on the previous)

```
a2: τ = g(q)                                          # gravity compensation only
b1: τ = J^T · [K_p·Δx + K_d·Δẋ] + g(q)              # impedance control
c1: τ = J^T · [S_p·F_impedance + S_f·F_force] + g(q) # hybrid position-force
```

## Key Interfaces

### `lab3_common.py`

```python
# Paths
PROJECT_ROOT: Path          # repo root
MODELS_DIR: Path            # lab-3-dynamics-force-control/models/
SCENE_TORQUE_PATH: Path     # models/scene_torque.xml
SCENE_TABLE_PATH: Path      # models/scene_table.xml
URDF_PATH: Path             # Lab 2's ur5e.urdf (shared)

# Constants (re-exported from Lab 2 pattern)
NUM_JOINTS: int = 6
Q_HOME: np.ndarray          # [-π/2, -π/2, π/2, -π/2, -π/2, 0]
JOINT_NAMES: tuple[str, ...]
TORQUE_LIMITS: np.ndarray

# Model loaders
def load_mujoco_model(scene_path: Path | None = None) -> tuple[MjModel, MjData]: ...
def load_pinocchio_model() -> tuple[pin.Model, pin.Data, int]: ...  # returns (model, data, ee_frame_id)

# Utilities
def mj_quat_to_pin(quat_wxyz: np.ndarray) -> np.ndarray: ...
def pin_quat_to_mj(quat_xyzw: np.ndarray) -> np.ndarray: ...
def get_ee_pose(pin_model, pin_data, ee_fid, q) -> tuple[np.ndarray, np.ndarray]: ...
    # Returns (position[3], rotation_matrix[3x3])
def get_ee_velocity(pin_model, pin_data, ee_fid, q, qd) -> np.ndarray: ...
    # Returns 6D velocity (linear[3], angular[3])
```

### `a1_dynamics_fundamentals.py`

```python
def compute_mass_matrix(pin_model, pin_data, q: np.ndarray) -> np.ndarray: ...
    # Returns M(q) [6x6] via pin.crba()

def compute_coriolis_matrix(pin_model, pin_data, q, qd) -> np.ndarray: ...
    # Returns C(q,q̇) [6x6] via pin.computeCoriolisMatrix()

def compute_gravity_vector(pin_model, pin_data, q) -> np.ndarray: ...
    # Returns g(q) [6] via pin.computeGeneralizedGravity()

def cross_validate_gravity(pin_model, pin_data, mj_model, mj_data, q) -> float: ...
    # Compare Pinocchio g(q) vs MuJoCo qfrc_bias at zero velocity. Returns max error.

def cross_validate_mass_matrix(pin_model, pin_data, mj_model, mj_data, q) -> float: ...
    # Compare Pinocchio CRBA vs MuJoCo mj_fullM(). Returns max error.

def plot_dynamics(pin_model, pin_data, q_trajectory): ...
    # Generate M heatmap, g bar chart, C variation plots
```

### `a2_gravity_compensation.py`

```python
def gravity_compensation_controller(pin_model, pin_data, q: np.ndarray) -> np.ndarray: ...
    # Returns τ = g(q)

def run_gravity_comp_sim(duration: float = 5.0, q_init: np.ndarray = Q_HOME,
                         perturb_at: float | None = None) -> dict: ...
    # Run simulation, optionally apply perturbation. Returns trajectory data.
```

### `b1_impedance_controller.py`

```python
@dataclass
class ImpedanceGains:
    K_p: np.ndarray   # Stiffness [6x6] or [3x3] for position-only
    K_d: np.ndarray   # Damping [6x6] or [3x3]

def impedance_control(pin_model, pin_data, ee_fid: int,
                      q: np.ndarray, qd: np.ndarray,
                      x_des: np.ndarray, R_des: np.ndarray,
                      xd_des: np.ndarray,
                      gains: ImpedanceGains) -> np.ndarray: ...
    # Returns joint torques τ = J^T·F + g(q)

def orientation_error(R_des: np.ndarray, R_cur: np.ndarray) -> np.ndarray: ...
    # Returns 3D orientation error vector from rotation matrices

def run_impedance_sim(target_pose, gains: ImpedanceGains, duration: float) -> dict: ...
    # Run impedance tracking simulation. Returns trajectory data.
```

### `c1_force_control.py`

```python
@dataclass
class HybridGains:
    K_p: np.ndarray       # Position stiffness (XY)
    K_d: np.ndarray       # Position damping (XY)
    K_f_p: float          # Force proportional gain (Z)
    K_f_i: float          # Force integral gain (Z)
    F_desired: float      # Desired contact force (N)

def hybrid_position_force_controller(
    pin_model, pin_data, ee_fid: int,
    q, qd, x_des, xd_des,
    F_measured: float,
    force_integral: float,
    gains: HybridGains
) -> tuple[np.ndarray, float]: ...
    # Returns (joint torques, updated force_integral)

def read_contact_force(mj_data, sensor_id: int) -> float: ...
    # Read Z-component of contact force from MuJoCo sensor
```

### `c2_capstone_force_trace.py`

```python
def run_force_trace_demo(
    line_start: np.ndarray,   # XY start of line on table
    line_end: np.ndarray,     # XY end of line on table
    F_desired: float = 5.0,   # Target normal force (N)
    duration: float = 10.0
) -> dict: ...
    # Full capstone: approach → contact → trace. Returns trajectory data.
```

## Model Files

### `models/ur5e.xml`
- Copied from Lab 2's `ur5e.xml`
- Already has `<motor>` actuators (torque mode) — no changes needed
- Includes mesh references, joint definitions, body hierarchy

### `models/scene_torque.xml`
- Phase 1-2 scene: robot on floor, no table
- Includes `ur5e.xml`, floor plane, lighting, camera
- No contact surfaces other than floor

### `models/scene_table.xml`
- Phase 3 scene: robot on floor + flat table surface at known Z height
- Table body with box geom, configured contact parameters (`solref`, `solimp`)
- Force sensor at end-effector: `<sensor><force name="ee_force" site="attachment_site"/></sensor>`
- Table height chosen so EE at Q_HOME is above the surface (~0.3m)

### `models/ur5e.urdf`
- Symlink or copy from Lab 2 — same URDF used for Pinocchio
- Alternatively, reference Lab 2's URDF path directly in `lab3_common.py`

## Dependencies on Previous Labs

### From Lab 2 (reference, not import)
- `ur5e.xml` model file — copied to Lab 3 models/
- `ur5e.urdf` — referenced or copied for Pinocchio
- Physical constants pattern (DH params, joint limits, torque limits) — re-defined in `lab3_common.py`
- Quaternion conversion utilities — re-implemented in `lab3_common.py`
- Model loading pattern — same approach, different paths

### Design Decision: No cross-lab imports
Each lab is self-contained. Lab 3 copies what it needs from Lab 2 rather than importing across labs. This keeps labs independently runnable and avoids circular dependencies.

## Simulation Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Timestep | 0.001 s (1 kHz) | Standard for torque control |
| Gravity | [0, 0, -9.81] | Earth standard |
| Joint damping | 1.0 Nm·s/rad | From Lab 2 ur5e.xml defaults |
| Joint armature | 0.01 kg·m² | From Lab 2 ur5e.xml defaults |
| Table solref | [0.02, 1.0] | Default MuJoCo contact (tune in Phase 3) |
| Table solimp | [0.9, 0.95, 0.001] | Slightly stiff contact |
| Impedance K_p | 100–2000 N/m | Low to high stiffness range |
| Impedance K_d | 2√K_p (critical damping) | Standard damping ratio |
| Force control F_d | 5.0 N | Per lab brief |
