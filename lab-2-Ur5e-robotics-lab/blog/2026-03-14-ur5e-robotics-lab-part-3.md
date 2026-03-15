# Lab 2, Part 3: Numerical IK Is Where the Robot Stops Being Forgiving

Inverse kinematics on a 2-link arm is a nice exercise. Inverse kinematics on a 6-DOF industrial robot is where the project starts to feel like robotics instead of geometry homework.

[`a4_inverse_kinematics.py`](../src/a4_inverse_kinematics.py) is the point where all the earlier groundwork becomes necessary. Without a trustworthy FK model, without a well-understood Jacobian, and without some awareness of singular behavior, numerical IK quickly turns into unexplained failures.

## The Real Problem Is Not "Can I Solve IK?"

On paper, the problem sounds simple: given a target pose, find a joint configuration. In practice, the harder questions are:

- does the solver converge reliably,
- does it stay stable near singularities,
- does it respect joint limits,
- does the returned answer actually reproduce the target pose when pushed back through FK?

That is why A4 ships two methods instead of one:

- pseudo-inverse IK,
- damped least squares IK.

The comparison is useful because the pseudo-inverse method shows the idealized version of the algorithm, while DLS shows the version you are more likely to trust in a real pipeline.

## Why Damping Matters

The pseudo-inverse behaves well when the Jacobian is nicely conditioned. The trouble starts when the Jacobian loses rank or approaches singularity. Then the raw inverse-like update becomes too aggressive and joint updates can blow up.

Damped least squares fixes that by giving up a bit of exactness in exchange for stability. That tradeoff is exactly the right one in most practical manipulator code. You do not need the mathematically most aggressive update. You need the update that keeps converging when the target, the seed, or the local geometry gets uncomfortable.

The adaptive lambda strategy in A4 is especially important because it makes the solver less brittle:

- if the error goes down, reduce damping and move more aggressively,
- if the error goes up, increase damping and become more conservative.

That is not a full trust-region solver, but it captures the right intuition.

## Validation Is Part of the Solver, Not an Optional Extra

One of the strongest choices in this module is the round-trip validation step. After IK returns a configuration, the code checks the answer by running FK again and measuring:

- position error,
- orientation error.

That matters because IK is often over-celebrated too early. A solver can return "success" while still landing outside the tolerance that the rest of your pipeline actually needs.

The MuJoCo cross-check adds another layer of confidence. The configuration is not only validated analytically through Pinocchio, but also pushed into the simulator to verify that the simulated end-effector lands where expected.

## Benchmarking Changes the Conversation

It is easy to fool yourself with one clean demo target. The benchmark in A4 is important because it forces the solver to deal with a distribution of reachable poses instead of a single happy-case configuration.

That changes the evaluation from:

- "did the solver work once?"

to:

- "how often does it work, how quickly, and with what final error?"

That is the right question if you eventually want to feed the solver into trajectories, tracking loops, and integrated demos.

## What You Should Carry Forward

By the end of A4, inverse kinematics should no longer feel like a black box. It should feel like a numerical correction loop with three visible ingredients:

- a pose error,
- a Jacobian,
- a stabilization strategy.

That view is powerful because it prepares you for the control side of the lab. Once you see IK as "pose error plus model structure," the jump to model-based control becomes much smaller.

## Companion Files

- [A4 note](../docs/a4_inverse_kinematics.md)
- [a4_inverse_kinematics.py](../src/a4_inverse_kinematics.py)

## Navigation

- Previous: [Part 2](2026-03-14-ur5e-robotics-lab-part-2.md)
- Next: [Part 4](2026-03-14-ur5e-robotics-lab-part-4.md)
