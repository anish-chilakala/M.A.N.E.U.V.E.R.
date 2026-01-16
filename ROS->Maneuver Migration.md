# 🔁 Migrating from ROS to MANEUVER

*A practical, step-by-step guide to replacing ROS C++/Python nodes with verified MANEUVER code—without breaking your existing system.*

---

## Why Migrate Gradually?

MANEUVER is **not a ROS replacement**—it is a **language upgrade**.

You do **not** need to:

* Rewrite your entire stack
* Abandon ROS tools (rviz, rosbag, rqt)
* Change your message graph

Instead, you:

1. Keep ROS as the middleware
2. Replace individual nodes with MANEUVER
3. Gain **performance, safety, and clarity** node by node

---

## Migration Strategy Overview

```
ROS Graph (Before):            ROS Graph (After):

[LIDAR] -> [C++ Perception]    [LIDAR] -> [MANEUVER Perception]
                |                               |
             [Planner]                     [Planner]
                |                               |
            [Controller]                [Controller]
```

Only the **perception node** changed. Everything else stays the same.

---

## Step 0: Identify a Good First Node

Best candidates for first migration:

✅ High CPU or GPU usage
✅ Clear inputs / outputs
✅ Minimal ROS parameter magic
✅ Deterministic logic

**Great first nodes:**

* Sensor preprocessing
* State estimation
* Control loops
* Motion primitives

Avoid initially:

* Large state machines
* Nodes with heavy dynamic reconfiguration

---

## Step 1: Original ROS Node (C++)

### Example: Velocity Controller (ROS C++)

```cpp
// controller_node.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher motor_pub;

void cmdCallback(const geometry_msgs::Twist& msg) {
    double left = msg.linear.x - msg.angular.z;
    double right = msg.linear.x + msg.angular.z;
    // No unit safety, no timing guarantees
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    motor_pub = nh.advertise<geometry_msgs::Twist>("/motors", 10);
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmdCallback);
    ros::spin();
}
```

**Problems:**

* No unit checking
* Frame assumptions implicit
* No real-time guarantees
* Runtime-only error detection

---

## Step 2: Equivalent MANEUVER Node

### Same Node, Written in MANEUVER

```maneuver
ros_node velocity_controller:
    name: "controller"
    frequency: 100 Hz
    deadline: 5ms

subscriber cmd_vel:
    topic: "/cmd_vel"
    type: Twist

publisher motor_cmd:
    topic: "/motors"
    type: Velocity2D<m/s>

on cmd_vel.message:
    left_motor  = cmd_vel.linear.x - cmd_vel.angular.z
    right_motor = cmd_vel.linear.x + cmd_vel.angular.z

    publish [left_motor, right_motor] to motor_cmd
```

### What You Gained Instantly

| Feature             | ROS C++ | MANEUVER |
| ------------------- | ------- | -------- |
| Unit safety         | ❌       | ✅        |
| Timing verification | ❌       | ✅        |
| Frame awareness     | ❌       | ✅        |
| Determinism         | ⚠️      | ✅        |
| Readability         | ❌       | ✅        |

---

## Step 3: Build as a ROS Node

```bash
maneuver build --ros
```

This generates:

* A native ROS node binary
* Message bindings
* Launch-compatible metadata

You can now launch it like any other node:

```bash
rosrun my_pkg controller
```

or in ROS 2:

```bash
ros2 run my_pkg controller
```

---

## Step 4: Verify Behavior (No Guesswork)

### Timing Proof

```bash
maneuver verify --timing controller
```

Output:

```
✓ Worst-case execution: 2.3ms
✓ Deadline: 5ms
✓ Schedulable at 100 Hz
```

### Unit & Frame Proof

```bash
maneuver verify --types
```

If you accidentally mix frames or units, compilation fails.

---

## Step 5: Incremental Performance Upgrade

Once the node works:

```maneuver
#[optimize(aggressive)]
ros_node velocity_controller:
```

Now the compiler:

* Auto-vectorizes math
* Removes runtime checks
* Inlines ROS message handling

**Same behavior. Faster execution.**

---

## Example: Migrating a Perception Node

### ROS C++ (Typical)

* OpenCV image pipeline
* Multiple kernel launches
* CPU–GPU round trips

~33ms per frame

### MANEUVER

```maneuver
edges = camera.capture()
    .to_grayscale()
    .gaussian_blur(sigma: 5)
    .canny_edges()
```

✔ Single fused GPU kernel
✔ 2ms per frame
✔ Same ROS topic output

---

## Coexisting with Python Nodes

MANEUVER nodes appear to Python exactly like normal ROS nodes:

```python
# No changes needed
rospy.Subscriber("/cmd_vel", Twist, callback)
```

You can also expose MANEUVER logic *to* Python via FFI if needed.

---

## Migration Checklist

✔ Node compiles as ROS node
✔ Topics unchanged
✔ Message types compatible
✔ Timing verified
✔ Units & frames proven
✔ Performance measured

---

## Common Pitfalls

❌ Relying on dynamic ROS parameters inside control loops
❌ Mixing implicit frames (`base_link` assumed)
❌ Porting entire systems at once

**Best practice:** one node at a time.

---

## When to Go Further

After 2–3 migrated nodes, teams usually:

* Move safety logic into MANEUVER
* Replace planners and controllers
* Use MANEUVER for simulation + deployment

At that point, ROS becomes a **transport layer**, not a source of bugs.

---

## Final Takeaway

> **ROS orchestrates. MANEUVER guarantees.**

You don’t rewrite your robot.
You **upgrade its brain**—safely, incrementally, and provably.

---

**Next Steps**

* See `ROS_INTEGRATION.md`
* See `FFI.md`
* Try migrating your first node today
