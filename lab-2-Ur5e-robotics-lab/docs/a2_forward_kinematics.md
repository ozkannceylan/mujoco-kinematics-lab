# A2: Forward Kinematics

## Goal

The goal of this step is to clearly understand how we take the **6 joint angles of the UR5e robot** and convert them into the **3D end-effector pose**.

The most critical point here is this:

* In the 2-link planar robot, we were only thinking about `(x, y)`.
* In the UR5e, we now have to think about **3D space**, **6 revolute joints**, and **position + orientation** together.

So now we are not only calculating **where** the tip of the robot arm ends up, but also **which direction it is facing**.

That is why Forward Kinematics (FK) is the foundation of “going from joint space to Cartesian space” in robotics.

---

## Why Simple Geometry Is No Longer Enough

In a 2-link robot, it was possible to move forward with trigonometry:

* for the first link: `cos(theta1), sin(theta1)`
* for the second link: `cos(theta1+theta2), sin(theta1+theta2)`

This approach is very instructive in small examples. But in a 6-DOF robot like the UR5e:

* the shoulder rotates,
* the elbow bends,
* the wrist reorients around its own axes,
* and all of this stacks in 3D space.

Trying to build the geometry one piece at a time here becomes chaotic very quickly. The number of equations grows, the axes keep changing, and it becomes difficult to track which rotation is applied in which order.

To solve this problem, robotics uses a standard framework:

**Denavit-Hartenberg (DH) parameters**.

The great strength of the DH approach is this:

> No matter how complex the robot is, we can describe each joint relative to the previous one with a few standard parameters and build the whole chain systematically.

---

## Main Files

* Script: `src/a2_forward_kinematics.py`
* Shared math: `src/ur5e_common.py`
* Output: `a2_fk_validation.csv`

In this lab, the main division of work is as follows:

* `ur5e_common.py` carries the shared geometric/mathematical infrastructure for the UR5e.
* `a2_forward_kinematics.py` uses this infrastructure to build and test the FK chain.
* `a2_fk_validation.csv` lets us inspect the resulting outputs for different joint configurations.

---

## What Exactly Does Forward Kinematics Produce?

The input to forward kinematics is:

```text
q = [q1, q2, q3, q4, q5, q6]
```

Here, each `qi` represents the angle of the corresponding joint.

The output of FK is typically a **4x4 homogeneous transformation matrix**:

```text
T_base_to_ee
```

This matrix gives us two things at once:

1. **Position**

   * The `(x, y, z)` location of the end-effector relative to the base coordinate system
2. **Orientation**

   * How the end-effector is rotated in space

The matrix structure is:

```text
[ R  p ]
[ 0  1 ]
```

Here:

* `R` = `3x3` rotation matrix
* `p` = `3x1` position vector

So this single matrix tells us both **where** the robot tip is and **how** it is oriented.

---

## DH Parameters: 4 Basic Definitions for Each Joint

In the DH approach, each joint is described relative to the previous one using four parameters:

### 1. `theta` — Joint Angle

This is the joint’s own rotation angle.

* For a revolute joint, this is the variable parameter.
* This is the main value read from the motor encoder.
* It typically represents rotation around the `z` axis.

### 2. `d` — Link Offset

This is the translation along the `z` axis from one frame to the next.

* It can be thought of as a vertical or axial distance.
* It represents physical mounting offsets.

### 3. `a` — Link Length

This is the distance along the `x` axis.

* It can be thought of as the effective arm length extending from one joint to the next.
* It is an important part of the robot link geometry.

### 4. `alpha` — Twist Angle

This tells us how tilted the next joint’s `z` axis is relative to the current `z` axis.

* It is defined as a rotation around the `x` axis.
* It becomes especially critical in 3D robots when the axes are not parallel.

With these four parameters, even a complex spatial robot is turned into a disciplined kinematic chain.

---

## Critical Detail in This Repo: Not Plain Standard DH

In theory, many sources explain “standard DH” or “modified DH.” But in practice, projects do not always use the exact textbook version. Sometimes they use **a hybrid or fixed-transform-based version** that is more convenient for implementation.

The implementation note in this repo is especially important:

```text
T_fixed = Rot_x(alpha) * Trans_x(a) * Trans_z(d)
T_joint = T_fixed * Rot_z(theta)
```

So here the chain is built with the following logic:

1. First, the fixed geometric part of the link is applied:

   * `Rot_x(alpha)`
   * `Trans_x(a)`
   * `Trans_z(d)`
2. Then, the variable joint angle is applied:

   * `Rot_z(theta)`

This is an implementation that preserves the spirit of the classical DH table, but makes the separation clearer:

* **fixed geometry** is separate,
* **joint motion** is separate.

This separation is very useful when reading code because it forces you to answer this clearly:

* Is this transform coming from the robot’s physical body geometry?
* Or is it coming from the fact that the joint angle changed at that moment?

Being able to see that distinction is a serious engineering reflex in robotics software.

---

## What Is Inside `ur5e_common.py`?

This file contains the shared kinematic building blocks for the UR5e. Typically it includes things like:

* DH rows or equivalent link parameters
* rotation and translation matrix functions
* helper functions used in building the chain
* orientation conversions (such as `rotation_to_rpy`)
* tool offset or flange offset definitions

This file is basically the “kinematic dictionary” of the robot.

Even without a URDF, we carry the mathematical skeleton of the robot in this kind of shared file.

---

## Building the Chain: What Does `forward_chain()` Do?

The essence of forward kinematics is multiplying the chain in order.

For each joint, we generate a transform matrix:

```text
T_1, T_2, T_3, T_4, T_5, T_6
```

Then we combine them in sequence:

```text
T_base_to_ee = T_1 * T_2 * T_3 * T_4 * T_5 * T_6
```

If there is a tool offset at the end, we add it as well:

```text
T_base_to_tool = T_1 * T_2 * T_3 * T_4 * T_5 * T_6 * T_tool
```

The logic here is extremely important:

* Each new joint redefines the coordinate system.
* The next transform is then applied on the new frame created by the previous one.
* That is why the multiplication order is critical.

Matrix multiplication here is not just algebra; it carries physical meaning:

> “First the shoulder rotates, then the elbow rotates around its new axis, then the wrist stacks on top of that.”

If you multiply in the wrong order, you define a physically different robot.

---

## Physical Interpretation of the Homogeneous Transform Matrix

You need to be able to interpret the parts of the 4x4 FK result physically.

### Position

The column vector in the upper-right part of the matrix:

```text
[x, y, z]^T
```

This is the end-effector’s position in space relative to the base.

For example, in the current repo the home pose result is:

```text
(-0.8172, -0.3679, 0.0628) m
```

This tells you:

* how far the robot tip is forward/backward on the x-axis relative to the base,
* how far sideways on the y-axis,
* how far up/down on the z-axis.

### Orientation

The upper-left `3x3` block of the matrix is the rotation matrix.

This matrix tells us how the axes of the end-effector frame are rotated relative to the base frame.

This is critical because in robotics applications, position alone is often not enough.

Examples:

* if you want to pick up a part from above, the tool must point downward,
* if you are doing screwdriving, the axis alignment must be correct,
* if you are welding or soldering, a particular angle must be preserved.

So you can be at exactly the same `(x, y, z)` position and still be facing the completely wrong direction. FK computes that too.

---

## Why Do We Extract Roll–Pitch–Yaw?

In code outputs, we often include not only the matrix itself but also more human-readable orientation formats. One of the most common is:

* roll
* pitch
* yaw

So a function like `rotation_to_rpy` converts the `3x3` rotation matrix into three more interpretable angles.

This is useful because:

* you can interpret results quickly in CSV,
* you can compare configurations,
* you can track orientation changes more intuitively.

But there is an important engineering note here:

RPY is only for easier interpretation. Mathematically, the more robust representations are:

* rotation matrix,
* quaternion,
* or axis-angle and similar safer formats.

Because Euler/RPY representations can create singularities and interpretation issues at certain angles.

Still, at lab level and for quick validation, RPY is very useful.

---

## Why Is the Tool Offset Added at the End?

The final mechanical frame of the robot is often not the same as the physical tool tip.

Usually the structure is something like:

* last joint frame,
* flange frame,
* tool center point (TCP)

So at the end of the FK chain we apply one more fixed transform:

```text
T_tool
```

This transform represents the transition from the last link frame to the real tool center.

In practice this matters a lot because:

* the tip of the mesh you see in simulation,
* and the real operational point you want to control,
  may not be the same thing.

If the TCP is defined incorrectly, the robot may look correct but perform the task at the wrong point.

---

## Current Result

The home pose end-effector position obtained from the local FK implementation is:

```text
(-0.8172, -0.3679, 0.0628) m
```

This result shows that the current kinematic chain produces a pose numerically and that the chain is at least internally consistent.

But there is an important warning here:

This alone does **not** mean “the model is definitely correct.”

Because an FK implementation can be:

* internally consistent,
* but still have a wrong axis convention,
* a wrong link offset,
* an incorrectly defined tool offset,
* or a base frame direction that does not match the simulator.

That is why the next step is always **external validation**.

---

## What Does `a2_fk_validation.csv` Store?

The CSV contains the following fields:

* tested joint configuration,
* Cartesian position,
* roll, pitch, yaw,
* optional MuJoCo and Pinocchio position error columns.

This file is very valuable because FK is not just about “a function ran.” You need to see what happens across different poses.

Thanks to the CSV, you can investigate questions like:

* when joint 2 changes, how does the tip move?
* are the wrist joints mostly changing orientation or position?
* do poses like home, ready, and over_table make sense?
* are there unexpected jumps in some configurations?

Tables like this are one of the basic tools of debugging in robotics software.

---

## Why Are the MuJoCo and Pinocchio Columns Important?

The CSV is designed to optionally include comparison columns with MuJoCo and Pinocchio.

In the current repo state, these columns remain empty because the relevant external libraries are not installed.

But this design shows a very correct engineering mindset.

Because writing FK manually is one thing; validating it against an industry-standard reference is another.

### Why Pinocchio?

Pinocchio is a very strong library for robotics kinematics and dynamics. It is especially valuable for:

* URDF-based modeling,
* fast kinematic computations,
* Jacobian and dynamics operations

and serves as a serious reference.

### Why MuJoCo?

MuJoCo, as a simulation engine, does not only provide visualization; it also offers a good comparison ground for the geometric and physical consistency of the model.

If your manual FK output is close to the MuJoCo/Pinocchio output, then you gain confidence that:

* your frame definitions are mostly correct,
* your link lengths are reasonable,
* your axis ordering matches the simulation model.

So ideally, the target is for the difference between your manual FK and the reference model to be very small.

Seeing error on the order of millimeters is usually a good sign.

---

## DH vs URDF: Which One Is Actually Used in Industry?

This point is very important.

For learning, DH parameters are very powerful because they teach you the mathematical chain of the robot. But in modern robotics software ecosystems, people often do not work directly with DH.

Instead, they usually use:

* **URDF**
* and libraries built on top of it.

### Advantage of DH

* it is educational,
* it forces you to build the chain systematically,
* it establishes matrix intuition,
* it makes the transition to Jacobians easier when taking derivatives.

### Limitation of DH

* the joint axes and frame placement are more constrained,
* sometimes you need artificial frames instead of directly expressing the physical robot,
* in complex mechanical layouts its natural expressiveness is limited.

### Advantage of URDF

* it provides more freedom of expression,
* it describes the real link/joint structure more naturally,
* it integrates well with modern tools such as ROS2, MoveIt2, Pinocchio, and MuJoCo.

So the right mental model is this:

> Learning DH builds the mathematical foundation of robotics. Learning URDF connects that mathematics to the real software ecosystem.

That is why your current effort to compare manual FK with modern tools is so valuable. It brings academic understanding and industrial validation together in the same place.

---

## How Should You Read the `forward_chain()` Function?

To understand this code, go through it line by line with the following lens:

### 1. What is the starting frame?

Usually the chain starts with the identity matrix:

```text
T = I
```

This means “no transform has been applied yet.”

### 2. Which fixed parameters are coming in for each joint?

Find the corresponding `alpha`, `a`, and `d` values.

These are the geometric skeleton of the robot.

### 3. Which variable joint angle is being added?

The `theta` used there comes from the current joint state.

### 4. How is the accumulated matrix updated?

Typical logic:

```text
T = T @ T_joint
```

Here `@` means matrix multiplication.

### 5. Is there a tool transform?

If yes, it is added at the end.

### 6. What parts are returned at the end?

* position
* rotation matrix
* RPY
* validation row

When you read the function with this perspective, the code stops being just “numpy operations” and becomes physical robot motion.

---

## Why Should You Draw Poses Like Home, Ready, and Over-Table?

The recommendation in the document is very good:

1. Read the DH rows in `ur5e_common.py`.
2. Follow the `forward_chain()` function step by step.
3. Inspect how the pose changes as the configuration changes in `a2_fk_validation.csv`.
4. Draw the `home`, `ready`, and `over_table` states.

This last point is especially important. Because the way to truly understand FK is not just to see the formula, but to mentally visualize the relationship between configuration space and physical space.

For example:

* `home` may be a more neutral starting pose,
* `ready` may be a safe standby pose before operation,
* `over_table` may be a task pose extending over the workspace.

If you draw these poses, you begin to understand intuitively:

* which joints dominate reach,
* which joints are more critical for orientation,
* how much the wrist joints change position,
* what the robot’s workspace shape looks like.

This intuition will be very useful when moving on to Jacobians and singularities.

---

## The Biggest Engineering Gain From This Lab

This work gives you three core reflexes:

### 1. Seeing the relationship between joint space and task space

The robot controller works with joint angles.

But the user usually wants things like:

* “take the tip over the table”
* “make the gripper face downward”
* “approach this point with this angle”

FK is the first bridge between those two worlds.

### 2. Learning to think in frames

In robotics, everything is a frame:

* base frame
* joint frame
* tool frame
* world frame

When you work through FK, what you are really doing is chaining coordinate systems together.

Once you understand that, you start truly reading robotics software.

### 3. Cross-validating simulation and mathematics

Comparing your hand-written FK with tools like MuJoCo and Pinocchio moves you beyond just using code.

It pushes you toward becoming someone who:

* validates models,
* catches frame mistakes,
* questions axis conventions,
* and builds engineering confidence.

---

## Important Limitation

This module is the **starting point of the repo for lab 2**; it is not yet a final, industry-grade, fully validated model.

So this limitation should be stated clearly:

* The FK structure here is educational and functional.
* But it should not be treated as an absolute reference model.
* The first serious validation step should be a numerical comparison of this module once MuJoCo and Pinocchio are installed.

So this stage is the beginning of answering the question “is the foundation correct?”; it is not the final answer.

---

## Summary

In this step, what we did was systematically convert the UR5e’s 6 joint angles into the end-effector pose through a kinematic chain.

Main ideas:

* 2D geometry is no longer enough; we now need 3D matrix-based thinking.
* Each joint is defined using DH-like parameters.
* The repo uses a transform structure that separates fixed geometry and joint rotation.
* All joint matrices are multiplied in sequence to go from base to end-effector.
* The result contains not only position but also orientation.
* The CSV output allows us to observe how different joint configurations affect the pose.
* Comparison with MuJoCo and Pinocchio will be the first serious validation step of this manual kinematics.

In short:

> Forward kinematics is the mechanism that translates the language of “joint angles” into the physical-world language of “position and orientation.”

Without a solid foundation here, it is not safe to move on to advanced topics like Jacobians, inverse kinematics, singularities, velocity control, and task-space control.

---

## Next Logical Step

The natural next step from here is:

* **Jacobian (6x6)**
* velocity kinematics
* singularity analysis

Because FK answers the question:

> “Where is the robot at these joint angles?”

The Jacobian moves one level higher and asks:

> “If I move the joints a tiny amount, how do the end-effector velocity and orientation change?”

That transition is the engineering threshold where you stop merely posing the robot and start truly controlling it.
