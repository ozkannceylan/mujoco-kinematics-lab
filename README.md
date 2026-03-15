# MuJoCo Kinematics Lab

A hands-on robotics lab series for rebuilding robotics fundamentals in simulation with **MuJoCo**. Each lab focuses on a different robot, progressing from simple planar arms to industrial 6-DOF manipulators.

The labs build up the same core pipeline step by step: **forward kinematics, Jacobians, inverse kinematics, dynamics, trajectory generation, control, and a final integration demo**.

---

## Labs

### Lab 1: 2-Link Planar Arm

A minimal 2-DOF planar robot arm. Everything is built from first principles — the math stays visible and every concept maps directly to code.

**Final demo**: Draws a precise **10 cm Cartesian square** with computed torque control.

![Lab 1 — Square Drawing](lab-1-2link-arm/media/c1_draw_square.gif)

| Metric | Value |
|---|---|
| Square tracking RMS error | 0.008 mm |
| Max torque | 0.076 Nm |
| IK success rate | 100% |

[Go to Lab 1](lab-1-2link-arm/)

---

### Lab 2: UR5e Industrial Robot Arm

A full 6-DOF industrial manipulator using the **UR5e** model from MuJoCo Menagerie, with **Pinocchio** for analytical computations and **MuJoCo** for physics simulation.

**Final demo**: Draws a **3D cube** (12 edges) with sub-millimeter precision using gravity compensation + velocity feedforward.

![Lab 2 — Cube Drawing](lab-2-Ur5e-robotics-lab/media/c3_draw_cube.gif)

| Metric | Value |
|---|---|
| Cube tracking RMS error | 0.088 mm |
| Max torque | 16.50 Nm |
| IK waypoint error | < 0.1 mm |

[Go to Lab 2](lab-2-Ur5e-robotics-lab/)

---

## Repository Structure

```
mujoco-kinematics-lab/
├── lab-1-2link-arm/              # Lab 1: 2-Link Planar Arm
│   ├── src/                      #   Source scripts (A1–C1)
│   ├── models/                   #   MuJoCo XML models
│   ├── docs/                     #   English documentation
│   ├── docs-turkish/             #   Turkish documentation
│   ├── media/                    #   Videos and GIFs
│   ├── tests/                    #   Unit tests
│   └── README.md                 #   Lab overview
│
├── lab-2-Ur5e-robotics-lab/      # Lab 2: UR5e 6-DOF Arm
│   ├── src/                      #   Source scripts (A1–C3)
│   ├── models/                   #   URDF, MJCF, Menagerie files
│   ├── docs/                     #   English documentation
│   ├── docs-turkish/             #   Turkish documentation
│   ├── media/                    #   Videos and GIFs
│   ├── tests/                    #   Unit tests
│   └── README.md                 #   Lab overview
│
├── CLAUDE.md                     # Project instructions for AI assistant
└── README.md                     # This file
```

Each lab is self-contained with its own source code, models, documentation, tests, and media. New labs follow the same structure.

---

## Quick Start

### Install dependencies

```bash
pip install mujoco numpy pinocchio imageio[ffmpeg] matplotlib
```

### Run a lab demo

```bash
# Lab 1: 2-link square drawing
python3 lab-1-2link-arm/src/c1_draw_square.py

# Lab 2: UR5e cube drawing
python3 lab-2-Ur5e-robotics-lab/src/c3_draw_cube.py
```

---

## Topics Covered

Both labs cover the same fundamental topics at different scales:

| Topic | Lab 1 (2-DOF) | Lab 2 (6-DOF) |
|---|---|---|
| Forward Kinematics | Analytic 2-link FK | DH + Pinocchio + MuJoCo cross-validation |
| Jacobian | 2x2 analytic | Geometric, Pinocchio, numerical + singularity analysis |
| Inverse Kinematics | Analytic + pseudo-inverse + DLS | Pseudo-inverse + adaptive DLS |
| Dynamics | M, C, g from MuJoCo | Pinocchio RNEA, ABA, CRBA + cross-validation |
| Trajectory | Cubic, quintic | Cubic, quintic, trapezoidal, min-jerk, multi-segment |
| Control | PD + gravity compensation | PD+g, computed torque, task-space impedance, OSC |
| Integration | Square drawing | Pick-and-place pipeline + 3D cube drawing |

---

## Core Architecture (Lab 2)

```
Pinocchio = analytical brain (FK, Jacobian, M, C, g, IK)
MuJoCo   = physics simulator (step, render, contact, sensor)
```

Both engines are cross-validated against each other at every stage to ensure correctness.

---

## Documentation

Each lab has full English and Turkish documentation in its `docs/` and `docs-turkish/` folders. See the individual lab READMEs for links.
