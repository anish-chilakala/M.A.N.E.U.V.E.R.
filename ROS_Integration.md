# 🤖 ROS Integration with MANEUVER

This document explains how **MANEUVER** interoperates with the Robot Operating System (ROS 1 & ROS 2), allowing gradual adoption and seamless coexistence with existing robotics stacks.

---

## Goals of ROS Integration

* Allow MANEUVER programs to **act as ROS nodes**
* Enable **zero/low-copy message passing** between MANEUVER and ROS
* Preserve MANEUVER guarantees (units, frames, timing) across ROS boundaries
* Support **incremental migration** from C++/Python ROS code

---

## Architecture Overview

```
+-------------------+        +-------------------+
|   MANEUVER Code   | <----> |     ROS Graph     |
|                   |        | (Nodes, Topics,   |
|  Verified Runtime |        |  Services, Actions)|
+-------------------+        +-------------------+
          |                           |
          +---- ROS Adapter Layer ----+
```

The **ROS Adapter Layer** translates between:

* MANEUVER types ↔ ROS messages
* MANEUVER tasks ↔ ROS node execution
* MANEUVER timing guarantees ↔ ROS executors

---

## Creating ROS Nodes in MANEUVER

### Basic ROS Node

```maneuver
ros_node perception_node:
    name: "maneuver_perception"
    namespace: "/autonomy"
```

This compiles into a native ROS node:

* ROS 1: `ros::NodeHandle`
* ROS 2: `rclcpp::Node`

---

## Publishers

```maneuver
publisher lidar_pub:
    topic: "/lidar/points"
    type: PointCloud2
    rate: 10 Hz

on lidar.data:
    publish lidar.data to lidar_pub
```

### Guarantees

* Publish rate is **statically verified**
* Message units and frames must match the ROS message definition
* Zero-copy shared memory used when supported (ROS 2 + CycloneDDS)

---

## Subscribers

```maneuver
subscriber cmd_vel_sub:
    topic: "/cmd_vel"
    type: Twist

on cmd_vel_sub.message:
    move robot with velocity cmd_vel_sub.linear.x m/s
```

The compiler ensures:

* Units are respected (`m/s`, `rad/s`)
* Frame consistency (e.g., `base_link`)

---

## Services

```maneuver
ros_service reset_odometry:
    request: Empty
    response: Bool

on reset_odometry.call:
    reset odometry
    return true
```

Maps to:

* ROS 1: `ros::ServiceServer`
* ROS 2: `rclcpp::Service`

---

## Actions

```maneuver
action navigate_to_pose:
    goal: PoseStamped
    feedback: DistanceRemaining
    result: Bool

on navigate_to_pose.goal:
    plan path to goal.pose
    follow path
    return success
```

Action feedback frequency and deadlines are **verified at compile time**.

---

## Message Type Mapping

| ROS Type                | MANEUVER Equivalent           |
| ----------------------- | ----------------------------- |
| geometry_msgs/Twist     | Velocity2D<m/s, rad/s>        |
| nav_msgs/Odometry       | Pose + Velocity + Covariance  |
| sensor_msgs/Image       | Image<width, height, format>  |
| sensor_msgs/PointCloud2 | PointCloud<Frame, Resolution> |

Custom `.msg` and `.idl` files are supported.

---

## Timing & Executors

```maneuver
ros_node control_node:
    frequency: 100 Hz
    deadline: 8ms
```

* Maps to real-time ROS executors
* Compiler rejects configurations ROS cannot meet

---

## Launch Files

MANEUVER can **generate ROS launch files automatically**:

```bash
maneuver build --ros-launch
```

Supports:

* ROS 1 `.launch`
* ROS 2 Python launch files

---

## Migration Strategy

1. Wrap existing ROS nodes using FFI
2. Replace performance-critical nodes with MANEUVER
3. Gradually move safety-critical logic into verified MANEUVER code

---

## Limitations (Current)

* ROS dynamic reconfigure partially supported
* rqt plugins require Python bindings

---

## Future Work

* Native ROS 2 executor written in MANEUVER
* Verified DDS QoS policies
* Deterministic multi-robot ROS graphs

---

**MANEUVER + ROS = Verified robotics without rewriting your stack.**
