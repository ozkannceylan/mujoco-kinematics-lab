---
title: "MuJoCo Kinematics Lab — Part 1: Setup and Modeling a 2-Link Robot"
date: "2026-03-08"
description: "Setting up a MuJoCo simulation environment and defining a 2-link planar manipulator in MJCF from scratch."
tags: ["MuJoCo", "Robotics", "MJCF", "Simulation", "Python"]
---

This is the first post in a five-part series documenting [Lab 1: 2-Link Planar Arm](../README.md) — a project that builds a complete robotics control pipeline from first principles on a 2-link planar robot arm.

**Series overview:**
1. **Setup and Modeling** (this post)
2. Forward Kinematics, Workspace, and Jacobians
3. Inverse Kinematics: Analytic, Pseudo-inverse, and DLS
4. Trajectory Generation and PD Control
5. From Kinematics to Computed Torque Control: Drawing a Cartesian Square

---

## Companion Files In This Repo

- Overview: [../README.md](../README.md)
- Module note: [../docs/a1_mujoco_basics.md](../docs/a1_mujoco_basics.md)
- Source script: [../src/a1_mujoco_setup.py](../src/a1_mujoco_setup.py)
- Model files: [../models/two_link.xml](../models/two_link.xml), [../models/two_link_torque.xml](../models/two_link_torque.xml)
- Blog index: [README.md](README.md)

All paths in this post assume you are working inside `lab-1-2link-arm/`.

---

## Why MuJoCo?

MuJoCo (Multi-Joint dynamics with Contact) is a physics engine built for robotics and reinforcement learning research. Compared to Gazebo or Isaac Sim, it is lighter, directly accessible via a Python API, and optimized for headless batch simulation — you can run thousands of simulations in parallel.

For a fundamentals lab the key advantage is the clean separation between model definition (XML) and simulation logic (Python). This makes it easy to inspect every quantity the engine computes.

## MJCF: The Robot Description Language

Everything in MuJoCo is defined using MJCF (MuJoCo XML Format). The mental model:

```xml
<mujoco>
  <option>        <!-- physics parameters: gravity, timestep -->
  <worldbody>     <!-- scene hierarchy: body → joint → geom -->
  <actuator>      <!-- motors attached to joints -->
  <sensor>        <!-- measurements: position, velocity, force -->
</mujoco>
```

Three core concepts form the building blocks:

| Concept | Purpose | Analogy |
|---------|---------|---------|
| **Body** | Coordinate frame — an invisible reference point | A skeletal joint |
| **Geom** | Physical shape (collision + visual), attached to a body | The bone/flesh |
| **Joint** | Degree of freedom between two bodies, defined inside a body | The direction of motion |

Bodies are nested in a parent-child hierarchy. A joint defines how the child body moves relative to its parent. A geom gives the body its physical presence.

## The 2-Link Planar Manipulator

The robot used throughout this series is intentionally minimal — two links, two hinge joints, operating in the XY plane:

```
         joint1          joint2         end-effector
  (base) ──○──── Link1 ────○──── Link2 ────●
           │    (0.3 m)     │    (0.3 m)
           │ z-axis         │ z-axis
           │ hinge          │ hinge
```

Key properties:
- **2 links**, each 0.3 m long
- **2 hinge joints** rotating about the z-axis
- **Link masses:** 1.0 kg (link 1), 0.7 kg (link 2), 0.05 kg (end-effector)
- **Gravity off** initially — isolating kinematics from dynamics

### The MJCF Model

The core of the model definition is the body hierarchy:

```xml
<body name="link1" pos="0 0 0">
  <joint name="joint1" type="hinge" axis="0 0 1"/>
  <geom fromto="0 0 0  0.3 0 0" size="0.022" mass="1"/>

  <body name="link2" pos="0.3 0 0">
    <joint name="joint2" type="hinge" axis="0 0 1"/>
    <geom fromto="0 0 0  0.3 0 0" size="0.018" mass="0.7"/>

    <body name="ee_body" pos="0.3 0 0">
      <site name="end_effector" pos="0.015 0 0"/>
    </body>
  </body>
</body>
```

`pos="0.3 0 0"` on `link2` means it begins at the tip of `link1`. Thanks to this parent-child relationship, when `joint1` rotates, `link2` rotates along with it — that is the kinematic chain.

The `site` element has no physical effect; it is a tracking point. The end-effector position is read via `data.site_xpos` during simulation.

Two model variants exist in the repository:
- **`two_link.xml`** — position-controlled actuators (internal PD), used for kinematics work
- **`two_link_torque.xml`** — direct torque actuators, used for computed torque control in the final demo

## MuJoCo Python API Basics

The simulation loop in Python is straightforward:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("models/two_link.xml")
data  = mujoco.MjData(model)

# Simulation step
mujoco.mj_step(model, data)

# Read state
data.qpos          # joint angles [θ₁, θ₂]
data.qvel          # joint velocities [θ̇₁, θ̇₂]
data.site_xpos[i]  # Cartesian position of a site

# Send motor commands
data.ctrl[0] = 5.0
data.ctrl[1] = -3.0
```

The `model` object holds constant parameters (link lengths, masses, joint limits) and never changes. The `data` object holds the instantaneous state (joint angles, velocities, forces) and is updated with every `mj_step()`.

## Verification

The setup script runs four checks:

| Check | Result |
|-------|--------|
| `mj_step()` runs 100 steps without error | 0.2 s simulation time |
| `data.qpos` returns 2 elements | shape = (2,) |
| Motor command changes joint angles | ctrl = [5, -3] → measurable Δθ |
| End-effector position is readable | `site_xpos` returns (x, y, z) |

Once these pass, the simulation environment is verified and ready for kinematics work.

## What's Next

With the model loaded and verified, the next post derives the **forward kinematics** equations that map joint angles to end-effector position, validates them against MuJoCo's internal computation, and analyzes the robot's workspace and the **Jacobian** matrix that relates joint velocities to Cartesian velocities.

## Navigation

- Next: [Part 2](2026-03-09-mujoco-kinematics-lab-part-2.md)
