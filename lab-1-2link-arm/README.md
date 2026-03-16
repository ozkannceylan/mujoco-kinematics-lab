# Lab 1: 2-Link Planar Arm

A self-contained robotics fundamentals lab built around a minimal **2-link planar robot arm** in MuJoCo.

The goal of Lab 1 is to keep the math visible while still building a complete manipulator stack end to end: forward kinematics, Jacobians, inverse kinematics, dynamics, trajectory generation, PD control, and computed torque control.

## Showcase

![Square Drawing Demo](media/c1_draw_square.gif)

> The final demo draws a **10 cm Cartesian square** using **computed torque control**, **quintic trajectory generation**, **analytic inverse kinematics**, and **Jacobian-based velocity mapping**.

## Key Results

| Metric | Value |
|---|---|
| Square tracking RMS error | 0.008 mm |
| Square tracking max error | 0.013 mm |
| Max torque used | 0.076 Nm |
| Analytic vs. numeric Jacobian error | < 1e-10 |
| IK success rate (20 random targets) | 100% |

---

## Modules

### A — Foundations

| Module | Topic | Script |
|---|---|---|
| A1 | MuJoCo setup and interactive demo | `src/a1_mujoco_setup.py` |
| A2 | Forward kinematics and workspace analysis | `src/a2_forward_kinematics.py` |
| A3 | Analytic Jacobian and singularity analysis | `src/a3_jacobian.py` |
| A4 | Inverse kinematics (analytic, pseudo-inverse, DLS) | `src/a4_inverse_kinematics.py` |
| A5 | Dynamics basics (M, C, g from MuJoCo) | `src/a5_dynamics_basics.py` |

### B — Control and Trajectory

| Module | Topic | Script |
|---|---|---|
| B1 | Cubic and quintic trajectory generation | `src/b1_trajectory_generation.py` |
| B2 | PD control and gravity compensation | `src/b2_pd_controller.py` |
| B3 | Full pipeline demos (pick-and-place, circle tracking) | `src/b3_full_pipeline.py` |

### C — Integration

| Module | Topic | Script |
|---|---|---|
| C1 | Cartesian square drawing with computed torque control | `src/c1_draw_square.py` |

---

## Quick Start

```bash
# Enter the lab folder
cd lab-1-2link-arm

# Install dependencies
pip install mujoco numpy imageio[ffmpeg]

# Run the final demo (opens MuJoCo viewer)
python3 src/c1_draw_square.py

# Record the demo video (headless)
python3 src/c1_record_video.py
```

---

## How To Study This Lab

1. Start with the module notes in `docs/` or `docs-turkish/`.
2. Open the matching script in `src/`.
3. Inspect the generated CSV artifacts in the docs folders.
4. Finish with the blog posts in `blog/` for the longer-form narrative.

This order keeps the code, notes, and write-ups aligned.

---

## Structure

```
lab-1-2link-arm/
├── src/              Source scripts (A1–C1)
├── models/           MuJoCo XML robot models
├── docs/             English documentation
├── docs-turkish/     Turkish documentation
├── blog/             English blog-style writeups for the series
├── media/            Recorded videos and GIFs
├── tests/            Unit tests
└── ros2_bridge/      ROS 2 bridge node
```

---

## Documentation

| Module | English | Turkish |
|---|---|---|
| A1 | [MuJoCo Basics](docs/a1_mujoco_basics.md) | [MuJoCo Temelleri](docs-turkish/a1_mujoco_temelleri.md) |
| A2 | [Forward Kinematics](docs/a2_forward_kinematics.md) | [Ileri Kinematik](docs-turkish/a2_forward_kinematics.md) |
| A3 | [Jacobian](docs/a3_jacobian.md) | [Jacobian](docs-turkish/a3_jacobian.md) |
| A4 | [Inverse Kinematics](docs/a4_inverse_kinematics.md) | [Ters Kinematik](docs-turkish/a4_inverse_kinematics.md) |
| A5 | [Dynamics](docs/a5_dynamics.md) | [Dinamik](docs-turkish/a5_dynamics.md) |
| B1 | [Trajectory Generation](docs/b1_trajectory_generation.md) | [Yorunge Uretimi](docs-turkish/b1_trajectory_generation.md) |
| B2 | [PD Control](docs/b2_pd_controller.md) | [PD Kontrol](docs-turkish/b2_pd_controller.md) |
| B3 | [Full Pipeline](docs/b3_full_pipeline.md) | [Tam Pipeline](docs-turkish/b3_full_pipeline.md) |
| C1 | [Draw Square](docs/c1_draw_square.md) | [Kare Cizimi](docs-turkish/c1_draw_square.md) |

---

## Blog Series

The longer-form writeups live in [`blog/`](blog/README.md):

| Part | Topic | Post |
|---|---|---|
| 1 | Setup and modeling | [Part 1](blog/2026-03-08-mujoco-robotics-lab-part-1.md) |
| 2 | FK, workspace, Jacobian | [Part 2](blog/2026-03-09-mujoco-robotics-lab-part-2.md) |
| 3 | IK, pseudo-inverse, DLS | [Part 3](blog/2026-03-10-mujoco-robotics-lab-part-3.md) |
| 4 | Trajectory generation and PD | [Part 4](blog/2026-03-11-mujoco-robotics-lab-part-4.md) |
| 5 | Computed torque and square drawing | [Part 5](blog/2026-03-12-mujoco-robotics-lab-part-5.md) |

---

## Repo-Local Entry Points

- Lab overview: [README.md](README.md)
- English notes: [docs/](docs/)
- Turkish notes: [docs-turkish/](docs-turkish/)
- Blog index: [blog/README.md](blog/README.md)
- Final demo script: [src/c1_draw_square.py](src/c1_draw_square.py)

---

## License

Unless noted otherwise, the Lab 1 source code and original documentation are
covered by the repository root [Apache-2.0 license](../LICENSE).
