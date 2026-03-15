# Lab 2, Part 1: Building a Trustworthy UR5e Model Stack

Lab 2 stops being a toy problem the moment the robot becomes a 6-DOF industrial arm. The main change is not just more joints. The bigger change is that the project needs a reliable separation between analytical robotics and simulation.

That is why this lab is built around a simple rule:

```text
Pinocchio explains the robot.
MuJoCo executes the robot.
```

The first module, [`a1_model_setup.py`](../src/a1_model_setup.py), exists to make that split explicit and testable.

## Why A1 Matters More Than It Looks

With a 2-link arm, it is still realistic to keep most of the math in your head. With a UR5e, that stops being productive very quickly. Once the robot has six revolute joints, multiple frame conventions, real actuator gains, and imported models from external assets, the first engineering problem is no longer "how do I compute FK?" It is "which model do I trust?"

A1 answers that by checking three things:

1. The dependencies are present.
2. The MuJoCo scene and URDF load correctly.
3. Both engines agree on the end-effector pose across multiple configurations.

If any one of those fails, everything downstream becomes hard to interpret.

## The Shared Contract Lives in `ur5e_common.py`

[`ur5e_common.py`](../src/ur5e_common.py) is the quiet backbone of the lab. It centralizes:

- model paths,
- joint names,
- home and zero configurations,
- joint, velocity, and torque limits,
- DH parameters,
- helper loaders for MuJoCo and Pinocchio,
- the frame and site mapping used for end-effector comparisons.

This is a small design choice, but it removes a common source of robotics bugs: duplicated assumptions. When the same joint order or model path is retyped in three different modules, one of them eventually drifts.

## MuJoCo and Pinocchio Are Not Competing Here

One easy mistake in robotics projects is to let different libraries solve overlapping parts of the same problem without a clear reason. This lab avoids that.

Pinocchio is used for:

- forward kinematics,
- Jacobians,
- inverse dynamics,
- mass matrix computation,
- inverse kinematics.

MuJoCo is used for:

- stepping the simulation,
- rendering,
- actuator behavior,
- contact behavior,
- extracting the simulated state.

That division keeps the code mentally clean. When something looks wrong in the robot motion, you can ask a more precise question: is the analytical model wrong, or is the simulated execution wrong?

## Cross-Validation Is the First Real Quality Gate

The most important part of A1 is not the import check. It is the FK comparison between:

- MuJoCo's `attachment_site`, and
- Pinocchio's `ee_link`.

That comparison is the first strong evidence that both libraries describe the same physical robot. It is also a rehearsal for the rest of the lab. Later modules keep repeating the same philosophy:

- compare Jacobians from multiple constructions,
- compare inverse-dynamics terms against simulator bias forces,
- validate IK by sending the result back through FK.

The habit is more important than the one-off result. Good robotics code is usually not "trust me, this formula is correct." It is "here are two independent ways to get the same answer."

## A Thin Wrapper Is Better Than Raw Simulator Access

[`mujoco_sim.py`](../src/mujoco_sim.py) adds another important piece: a narrow wrapper around MuJoCo. Instead of letting every script manipulate `qpos`, `qvel`, `ctrl`, and site arrays directly, the lab exposes a stable interface through `UR5eSimulator`.

That does two useful things:

- the higher-level modules become easier to read,
- simulator-specific array handling stops leaking into every file.

This is the sort of cleanup that feels optional at the beginning and essential by the time you reach the integrated demos.

## What You Should Take Away From Part 1

The first module is not just setup work. It establishes the project discipline that makes the rest of the lab believable:

- one shared model contract,
- one analytical engine,
- one simulation engine,
- repeated cross-checks between them.

That is the difference between a robotics notebook that "seems to move" and a robotics lab that can support control and integration work.

## Companion Files

- [Lab 2 README](../README.md)
- [A1 note](../docs/a1_environment_setup.md)
- [a1_model_setup.py](../src/a1_model_setup.py)
- [ur5e_common.py](../src/ur5e_common.py)
- [mujoco_sim.py](../src/mujoco_sim.py)

## Navigation

- Previous: [Blog Index](README.md)
- Next: [Part 2](2026-03-14-ur5e-robotics-lab-part-2.md)
