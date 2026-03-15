# C2: ROS 2 Bridge

## Goal

Bridge the MuJoCo UR5e simulation to the ROS 2 ecosystem, enabling joint state publishing, EE pose publishing, and joint command subscription.

## Files

- Script: `src/c2_ros2_bridge.py`
- Dependencies: `mujoco_sim.py` (`UR5eSimulator`), `ur5e_common.py`

## Implementation

### ROS 2 Topics

| Direction  | Topic             | Message Type                     | Content                          |
|------------|-------------------|----------------------------------|----------------------------------|
| Publish    | `/joint_states`   | `sensor_msgs/JointState`         | Joint positions and velocities   |
| Publish    | `/ee_pose`        | `geometry_msgs/Pose`             | EE position + quaternion         |
| Subscribe  | `/joint_commands` | `std_msgs/Float64MultiArray`     | Torque commands (6 values)       |

### Node: `UR5eBridgeNode`

- Extends `rclpy.node.Node` with name `ur5e_mujoco_bridge`.
- Holds a `BridgeConfig` dataclass with topic names and loop rate (default 500 Hz).
- Creates a `UR5eSimulator` instance initialized to `Q_HOME`.
- Timer callback at 500 Hz:
  1. Applies the latest received joint commands via `sim.set_ctrl()`.
  2. Steps the simulation via `sim.step()`.
  3. Reads the robot state via `sim.get_state()`.
  4. Publishes `JointState` with joint names from `JOINT_NAMES`, positions, and velocities.
  5. Publishes `Pose` with EE position and orientation quaternion (converted from rotation matrix via `scipy.spatial.transform.Rotation`).
- Subscription callback stores the incoming `Float64MultiArray` data as the current command vector.

### Demo Mode (No ROS 2)

When `rclpy` is not installed, the script:
1. Prints the bridge design (topics, message types, loop rate).
2. Runs a standalone simulation loop: initializes `UR5eSimulator` at `Q_HOME`, steps 1000 times, and prints the initial and final EE positions with the elapsed sim time.
3. Exits cleanly with a scaffold-complete message.

This ensures the script is always runnable for study purposes, even without a ROS 2 installation.

## How to Run

**With ROS 2 Humble:**
```bash
source /opt/ros/humble/setup.bash
python3 src/c2_ros2_bridge.py
```

**Without ROS 2 (demo mode):**
```bash
python3 src/c2_ros2_bridge.py
```

## What to Study

1. Read the topic names and message types: these follow standard ROS 2 conventions that MoveIt2 and other tools expect.
2. Understand the timer-based control loop: the 500 Hz rate matches a typical real-time control frequency.
3. Note the quaternion handling: MuJoCo rotation matrices are converted to quaternions for ROS Pose messages. The convention differences between MuJoCo (w,x,y,z) and ROS/scipy (x,y,z,w) are handled by scipy.
4. Compare the bridge architecture with a real robot driver: the same publish/subscribe pattern is used by `ros2_control` hardware interfaces.

## Next Step

With the bridge running, you can use standard ROS 2 tools to interact with the simulated UR5e:
- `ros2 topic echo /joint_states` to monitor joint state.
- `ros2 topic pub /joint_commands std_msgs/Float64MultiArray` to send torque commands.
- Connect to MoveIt2 for motion planning via the published joint states.
