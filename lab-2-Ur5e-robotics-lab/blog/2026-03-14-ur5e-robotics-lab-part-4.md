# Lab 2, Part 4: Dynamics and Control Are Where the Lab Starts Feeling Real

Up to this point, the project is mostly about describing motion. Part 4 is about causing motion with enough structure that the robot can actually track what you ask for.

That shift is handled by four modules:

- [`a5_dynamics.py`](../src/a5_dynamics.py),
- [`b1_trajectory_generation.py`](../src/b1_trajectory_generation.py),
- [`b2_control_hierarchy.py`](../src/b2_control_hierarchy.py),
- [`b3_constraints.py`](../src/b3_constraints.py).

Taken together, they answer a practical question: if the UR5e should move from one useful configuration to another, what should the reference look like, what torques should be applied, and what limits must stay active while it happens?

## Dynamics Turns Geometry Into Effort

The point of A5 is not only to compute matrices with serious names. It is to make joint effort legible.

Once you have:

- the mass matrix,
- gravity torques,
- Coriolis structure,
- inverse dynamics,
- forward dynamics,

you stop thinking of the robot as a chain of rigid transforms and start thinking of it as a system that resists, accelerates, and couples motion.

The MuJoCo comparison is especially valuable here. Matching Pinocchio's bias terms against simulator quantities is exactly the kind of sanity check that prevents control work from becoming mythology.

## Good Trajectories Remove Needless Controller Work

[`b1_trajectory_generation.py`](../src/b1_trajectory_generation.py) is easy to underestimate because trajectory generation often looks like a support topic. In reality, poor reference design forces the controller to compensate for avoidable discontinuities.

That is why the lab includes:

- cubic profiles,
- quintic profiles,
- trapezoidal motion,
- minimum-jerk Cartesian motion,
- multi-segment waypoint trajectories.

The important lesson is not that one trajectory family is always best. It is that the reference should match the problem:

- smooth start and stop behavior,
- bounded velocities,
- consistent timing between joints,
- manageable acceleration demands.

If the reference is violent, the controller ends up looking bad for reasons that are not really the controller's fault.

## The Control Hierarchy Is a Ladder, Not a Grab Bag

Part of what makes B2 useful is the progression:

1. PD plus gravity compensation,
2. computed torque control,
3. task-space impedance,
4. operational space control.

That sequence teaches the right lesson. More advanced control is not magic. It is the result of adding more model information and more carefully choosing the space in which you regulate error.

PD plus gravity compensation is the baseline. Computed torque says: use the model to cancel much of the plant. Impedance says: regulate behavior in Cartesian space. OSC says: reason in terms of task-space inertia, not just mapped forces.

Seeing them on the same UR5e trajectory makes the differences much easier to feel than reading them as separate textbook chapters.

## Constraints Are Not an Afterthought

[`b3_constraints.py`](../src/b3_constraints.py) matters because control code that ignores limits is not really ready for integration. The lab adds:

- joint limit checks,
- clamping and repulsion,
- velocity scaling,
- torque saturation,
- self-collision heuristics.

These are not glamorous features, but they are the difference between "the robot tracks the reference" and "the robot tracks the reference without obviously unsafe behavior."

This is also where the project starts to resemble real robotics engineering. The best controller in the world is still not useful if it only works when you pretend the robot has no limits.

## What Part 4 Sets Up

By the time you finish these four modules, you have all the ingredients needed for integrated tasks:

- analytical models you trust,
- references you can execute,
- controllers of increasing sophistication,
- constraint handling that keeps the motion usable.

That is enough to move beyond isolated module demos and into whole-task behavior, which is exactly where Part 5 goes.

## Companion Files

- [A5 note](../docs/a5_dynamics.md)
- [B1 note](../docs/b1_trajectory_generation.md)
- [B2 note](../docs/b2_control_hierarchy.md)
- [B3 note](../docs/b3_constraints.md)
- [a5_dynamics.py](../src/a5_dynamics.py)
- [b1_trajectory_generation.py](../src/b1_trajectory_generation.py)
- [b2_control_hierarchy.py](../src/b2_control_hierarchy.py)
- [b3_constraints.py](../src/b3_constraints.py)

## Navigation

- Previous: [Part 3](2026-03-14-ur5e-robotics-lab-part-3.md)
- Next: [Part 5](2026-03-14-ur5e-robotics-lab-part-5.md)
