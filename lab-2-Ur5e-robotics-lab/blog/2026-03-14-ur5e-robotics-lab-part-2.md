# Lab 2, Part 2: Making a 6-DOF Arm Understandable With FK and Jacobians

Once the UR5e model stack is trustworthy, the next challenge is interpretation. A six-joint manipulator becomes hard to reason about unless you can switch comfortably between three viewpoints:

- the geometric chain,
- the end-effector pose,
- the local motion sensitivity encoded in the Jacobian.

That is the job of [`a2_forward_kinematics.py`](../src/a2_forward_kinematics.py) and [`a3_jacobian.py`](../src/a3_jacobian.py).

## Why Keep the DH Version at All?

Lab 2 already has a URDF and a MuJoCo scene, so it would be easy to say "Pinocchio already computes FK, job done." But that would throw away the educational value of the chain itself.

The DH implementation in A2 is useful precisely because it is not the production reference. It makes the kinematic structure visible:

- each transform is explicit,
- each parameter has a mechanical meaning,
- each intermediate frame can be inspected.

The key lesson is that the DH chain and the URDF/MJCF chain represent the same physical robot while using different frame conventions. That distinction matters. A robotics project can be mathematically correct and still appear inconsistent if two modules silently use different frames.

## FK Is More Than End-Effector Position

A common beginner trap is to reduce FK to "give me `x, y, z`." On a 6-DOF arm, that is far too narrow. FK is really the map from joint configuration to rigid-body pose.

In practical terms, that means A2 is about:

- position,
- rotation,
- frame conventions,
- intermediate link placements,
- agreement between the educational and production models.

If those foundations are fuzzy, the next steps in IK and control become fragile because they all depend on pose errors, not just positions.

## The Jacobian Is Where Motion Becomes Local

If FK tells you where the end-effector is, the Jacobian tells you how it will move next.

That sounds obvious, but it is the real bridge from geometry to algorithms:

- IK updates use it,
- singularity analysis uses it,
- task-space control uses it,
- manipulability metrics use it.

A3 is especially strong because it does not settle for one Jacobian implementation. It compares:

1. a geometric construction,
2. Pinocchio's built-in frame Jacobian,
3. a finite-difference numerical Jacobian.

That three-way comparison is exactly the right move for a project like this. If two versions disagree, you still do not know which one is wrong. If three versions line up, your confidence jumps.

## Singularities Stop Being Abstract Here

In a textbook, singularities often look like a side note under Jacobians. In an actual manipulator project, they are one of the central reasons numerical methods become unstable.

The UR5e gives concrete examples:

- wrist singularities when wrist axes align,
- elbow singularities when the arm extends,
- shoulder-related singular behavior depending on how the arm sits around the base axis.

This is also why A3 includes manipulability instead of only determinant checks. The determinant can tell you whether a square Jacobian has collapsed, but manipulability gives you a more useful scalar picture of how much dexterity you still have before you are fully singular.

## Why This Matters Before IK

The cleanest way to understand numerical IK is to treat it as a Jacobian-driven correction process. That only makes sense if the Jacobian itself is trustworthy.

By the time you finish A2 and A3, you should be able to answer:

- which end-effector frame you are controlling,
- which reference frame the Jacobian lives in,
- what happens to the local motion map near singularity,
- why a solver that ignores those issues can become unstable.

That is exactly the foundation needed for Part 3.

## Companion Files

- [A2 note](../docs/a2_forward_kinematics.md)
- [A3 note](../docs/a3_jacobian.md)
- [a2_forward_kinematics.py](../src/a2_forward_kinematics.py)
- [a3_jacobian.py](../src/a3_jacobian.py)

## Navigation

- Previous: [Part 1](2026-03-14-ur5e-robotics-lab-part-1.md)
- Next: [Part 3](2026-03-14-ur5e-robotics-lab-part-3.md)
