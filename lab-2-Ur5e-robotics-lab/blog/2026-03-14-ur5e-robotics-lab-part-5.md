# Lab 2, Part 5: Integration Is the Real Test, and the Cube Demo Earns Its Place

The earlier modules are only useful if they survive contact with a full task. That is what the last stretch of Lab 2 is about.

The three integration modules do not add much new theory. Instead, they force the existing theory to cooperate:

- [`c1_pick_and_place.py`](../src/c1_pick_and_place.py) turns IK, trajectories, control, and safety into one loop.
- [`c2_ros2_bridge.py`](../src/c2_ros2_bridge.py) shows how the simulation can talk to the outside world through ROS 2 topics.
- [`c3_draw_cube.py`](../src/c3_draw_cube.py) is the final demonstration that the whole stack can produce precise, readable task-space behavior.

## C1 Is the First Place Where Local Choices Become System Behavior

In C1, every earlier design decision becomes visible:

- if IK is unstable, waypoint chaining looks messy,
- if trajectories are poorly timed, segment transitions spike,
- if the dynamics model is weak, computed torque loses authority,
- if safety checks are missing, the pipeline may be accurate but not acceptable.

That is why C1 is so valuable for studying. It turns module quality into system quality. You can finally ask not just whether a single function works, but whether the stack behaves coherently over time.

## C2 Keeps the Lab Honest About Integration

Many projects stop at a nice simulator demo and never face an interface boundary. C2 avoids that trap. Even as a scaffold, it enforces a useful mindset:

- publish robot state,
- expose end-effector information,
- receive commands from the outside,
- make the simulator look like a system component rather than a sealed script.

That matters if your long-term goals include MoveIt2, real robot drivers, or VLA-style system integration. A robotics project becomes much more valuable once it can participate in a larger software graph.

## Why the Cube Demo Is Better Than a Simple Point-to-Point Demo

The cube drawing task in C3 is strong for one reason: it is easy to understand visually and difficult enough to expose weak assumptions.

It demands:

- repeated IK across a sequence of geometric waypoints,
- smooth multi-segment joint trajectories,
- steady tracking over a long horizon,
- awareness of collisions and workspace placement,
- stable actuator behavior from the Menagerie servo model.

The task is also honest. If the robot drifts, clips the table, or lags at segment transitions, you can see it immediately.

## The Most Important C3 Lesson Is About Actuators, Not Geometry

The standout engineering lesson in the cube demo is the control law built around the Menagerie UR5e position servos.

Naive position commands are not enough. The demo becomes precise only when the control input is augmented with:

- gravity compensation,
- velocity feedforward.

That is a meaningful result because it reflects a real integration truth: even when the analytical pipeline is correct, actuator behavior still decides whether the robot actually tracks the intended motion.

In other words, C3 is not just "draw a cube." It is "close the gap between the ideal reference and the actuator model you actually have."

## What Lab 2 Achieves

By the end of the UR5e lab, the project has moved through the full stack:

- model loading,
- kinematics,
- Jacobians,
- inverse kinematics,
- rigid-body dynamics,
- trajectory generation,
- controller design,
- safety constraints,
- task integration,
- ROS 2 interfacing,
- portfolio-ready final behavior.

That is exactly the kind of structure that scales to stronger robotics work later. The specific robot may change, but the discipline does not.

## Companion Files

- [C1 note](../docs/c1_full_pipeline.md)
- [C2 note](../docs/c2_ros2_bridge.md)
- [C3 note](../docs/c3_draw_cube.md)
- [c1_pick_and_place.py](../src/c1_pick_and_place.py)
- [c2_ros2_bridge.py](../src/c2_ros2_bridge.py)
- [c3_draw_cube.py](../src/c3_draw_cube.py)
- [c3_record_video.py](../src/c3_record_video.py)

## Navigation

- Previous: [Part 4](2026-03-14-ur5e-robotics-lab-part-4.md)
- Next: [Blog Index](README.md)
