# C3: Draw a 3D Cube — Full Pipeline Demo

## Goal

Draw a complete 3D cube (12 edges) with the UR5e end-effector in MuJoCo, integrating every module from the lab into a single demo with sub-millimeter tracking precision.

## Files

- Interactive demo: `src/c3_draw_cube.py`
- Video recorder: `src/c3_record_video.py`
- Output video: `media/c3_draw_cube.mp4`
- Output GIF: `media/c3_draw_cube.gif`

## Pipeline

The demo integrates six lab modules into a single pipeline:

```
Cube Geometry → IK (A4) → Quintic Trajectory (B1) → Position Control + GC + VFF → MuJoCo Sim
                                                            ↑
                                                    FK (A2) + Dynamics (A5)
```

### Stage 1: Cube Geometry

- `cube_vertices()` generates 8 vertices around a centre point
- `cube_edges()` defines the 12 edges as index pairs
- `cube_drawing_path()` produces a 13-waypoint Eulerian-like path that draws all 12 edges continuously without lifting

Configuration: centre = [0.3, 0.2, 0.55], side = 10 cm. This position is offset in Y to keep the arm clear of the table at all IK solutions.

### Stage 2: Inverse Kinematics (A4)

`solve_cube_ik()` uses Damped Least Squares IK at each path vertex with:
- Fixed EE orientation (home orientation)
- Sequential seeding: each solution seeds the next, ensuring smooth joint-space transitions
- Tolerance: 1e-4 (achieves < 0.1 mm IK error at every waypoint)

### Stage 3: Trajectory Generation (B1)

`build_cube_trajectory()` chains 12 quintic polynomial segments (2.0 s each, 24 s total). Quintic polynomials guarantee zero velocity and acceleration at segment boundaries, producing smooth motion.

### Stage 4: Position Control with Feedforward

The MuJoCo Menagerie UR5e uses `general` actuators with built-in PD position servos:

```
tau = Kp * (ctrl - qpos) - Kd * qvel
```

Naive position control (`ctrl = q_desired`) suffers from:
1. **Gravity droop**: steady-state offset because the servo must fight gravity through position error
2. **Tracking lag**: the servo chases a moving reference with a delay proportional to velocity

The solution is to augment ctrl with two feedforward terms:

```python
ctrl = q_des + g(q) / Kp + Kd * qd_des / Kp
```

- **Gravity compensation (GC)**: `g(q) / Kp` — pre-offsets ctrl so the servo's position error produces the exact torque needed to counteract gravity. Read from `data.qfrc_bias`.
- **Velocity feedforward (VFF)**: `Kd * qd_des / Kp` — cancels the damping term's drag against the desired velocity, so the servo tracks the reference without lag.

This makes the effective control law:
```
tau ≈ Kp * (q_des - qpos) - Kd * (qvel - qd_des) + g(q)
```

### Settling

Before the trajectory begins, the robot holds at the first waypoint for 3 s with GC-compensated ctrl, allowing transients to decay.

## Results

| Metric | Value |
|---|---|
| RMS Cartesian error | 0.088 mm |
| Max Cartesian error | 0.234 mm |
| Max actuator torque | 16.50 N·m |
| Total duration | 24.0 s |
| Video frames | 707 |

## Key Lessons

1. **Table collision matters**: The original cube at [0.4, 0.0, 0.50] caused the upper arm and forearm links to collide with the table surface (z ≈ 0.41 m). Contact forces pushed the robot off its desired trajectory, causing 133 mm RMS error. Moving the cube to [0.3, 0.2, 0.55] with a Y-offset eliminated all collisions.

2. **GC + VFF is essential for position-controlled servos**: Without feedforward, tracking error was 13+ mm even with no collisions. With GC alone, error dropped slightly. Adding velocity feedforward brought RMS below 0.1 mm — a 1500x improvement over the original.

3. **Actuator saturation is the enemy**: When the PD servo demands more torque than the force range allows (150 Nm for joints 1-3), the robot loses tracking authority. Proper feedforward keeps actuator forces well within limits (16.5 Nm peak).

## How to Run

```bash
# Interactive viewer (requires display)
python3 src/c3_draw_cube.py

# Record video (headless)
python3 src/c3_record_video.py
```

## What to Study

1. Read the `cube_drawing_path()` function and trace the Eulerian traversal on paper.
2. Compare `ctrl = q_des` (naive) vs `ctrl = q_des + g/Kp + Kd*qd/Kp` (GC+VFF) — modify the record script to try both and compare RMS.
3. Change `CUBE_CENTER` to [0.4, 0.0, 0.50] and observe how table collisions destroy tracking.
4. Try different `SEGMENT_DURATION` values to see how trajectory speed affects tracking with and without feedforward.

## Next Step

This completes the UR5e Robotics Lab. Review all modules from A1 to C3 and use them as building blocks for more advanced projects (VLA integration, MoveIt2 planning, etc.).
