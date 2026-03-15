# Interview Cheatsheet

## Explain FK vs IK

Forward kinematics maps joint angles to end-effector pose. Inverse kinematics does the opposite and often has multiple valid joint solutions for the same pose.

## What Is A Jacobian?

The Jacobian maps joint velocity to end-effector twist. It is also the main local linearization used by numerical IK, manipulability analysis, and task-space control.

## How Do You Handle Singularities?

I detect them through low manipulability or a collapsing Jacobian determinant, and I handle them with damped least squares, branch selection, trajectory shaping, and constraint-aware motion.

## Explain Computed Torque Control

Computed torque uses the robot dynamics model to cancel much of the nonlinear behavior. That makes the closed-loop error dynamics much closer to a linear second-order system.

## Explain OSC

Operational Space Control works directly in Cartesian space. Instead of solving IK first, it maps task-space errors to torques through the Jacobian and the task-space inertia.

## How Does This Relate To Humanoids Or VLA?

High-level policies usually want to command Cartesian intent. The low-level stack still needs kinematics, dynamics, constraints, and evaluation discipline to execute that intent safely and smoothly.
