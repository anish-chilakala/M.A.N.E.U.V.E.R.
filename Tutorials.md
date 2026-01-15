# MANEUVER Tutorials

> **Structured Learning Path from Beginner to Expert**

[![Difficulty](https://img.shields.io/badge/Difficulty-Beginner_to_Expert-blue)]()
[![Tutorials](https://img.shields.io/badge/Tutorials-30+-green)]()
[![Time](https://img.shields.io/badge/Total_Time-40+_hours-orange)]()

---

## 🎯 Learning Paths

Choose your path based on your goals:

1. **[Absolute Beginner](#beginner-path)** - Never programmed before
2. **[Hobbyist Path](#hobbyist-path)** - Build fun robots
3. **[Student Path](#student-path)** - Learn robotics for school
4. **[Professional Path](#professional-path)** - Production robotics
5. **[Researcher Path](#researcher-path)** - Advanced algorithms

---

## 🌱 Beginner Path

**Goal:** Learn basics, build your first robot  
**Time:** 10-15 hours  
**Prerequisites:** None!

### Tutorial 1: Your First Robot (30 min)
**What you'll learn:** Basic movement, waiting, output

```maneuver
robot my_first_bot:
    say "Hello, I'm alive!"
    move forward 50cm
    turn right 90 degrees
    wait 1 second
    say "I moved!"
```

**Exercise:** Make your robot draw a square

[Start Tutorial 1](./tutorials/01-first-robot.md)

---

### Tutorial 2: Working with Sensors (1 hour)
**What you'll learn:** Reading sensors, making decisions

```maneuver
robot obstacle_avoider:
    sensor distance:
        type: ultrasonic
        location: front
    
    loop:
        if distance < 20cm:
            stop
            turn right 90 degrees
        else:
            move forward 10cm
```

**Exercise:** Add multiple sensors for better obstacle avoidance

[Start Tutorial 2](./tutorials/02-sensors.md)

---

### Tutorial 3: Variables and Math (45 min)
**What you'll learn:** Storing data, calculations

```maneuver
robot calculator_bot:
    speed: 0.5 m/s
    distance_traveled: 0 meters
    
    loop 10 times:
        move forward 10cm at speed
        distance_traveled = distance_traveled + 10cm
    
    say "Total distance: ${distance_traveled}"
```

**Exercise:** Calculate time based on speed and distance

[Start Tutorial 3](./tutorials/03-variables.md)

---

### Tutorial 4: Functions (1 hour)
**What you'll learn:** Reusable code, organization

```maneuver
function turn_around():
    turn right 180 degrees

function dance():
    repeat 4 times:
        turn right 90 degrees
        move forward 10cm

robot dancer:
    dance()
    turn_around()
    dance()
```

**Exercise:** Create a patrol function

[Start Tutorial 4](./tutorials/04-functions.md)

---

### Tutorial 5: Arrays and Loops (1 hour)
**What you'll learn:** Collections, iteration

```maneuver
robot waypoint_follower:
    waypoints: [
        (x: 0m, y: 0m),
        (x: 1m, y: 0m),
        (x: 1m, y: 1m),
        (x: 0m, y: 1m)
    ]
    
    for point in waypoints:
        move to point
        wait 2 seconds
```

**Exercise:** Add more waypoints and make it loop forever

[Start Tutorial 5](./tutorials/05-arrays-loops.md)

---

## 🎮 Hobbyist Path

**Goal:** Build cool projects  
**Time:** 15-20 hours  
**Prerequisites:** Complete Beginner Path

### Tutorial 6: Line Following Robot (2 hours)
**What you'll build:** Robot that follows a black line

**Components needed:**
- 2 IR sensors
- 2 DC motors
- Motor driver

**Skills learned:**
- Multiple sensor fusion
- PID control basics
- Real-time decision making

[Start Tutorial 6](./tutorials/06-line-follower.md)

---

### Tutorial 7: Remote Control (1.5 hours)
**What you'll build:** Control robot with keyboard/phone

**Components needed:**
- WiFi module (ESP32 or Pi)

**Skills learned:**
- Network communication
- Event handling
- User interfaces

[Start Tutorial 7](./tutorials/07-remote-control.md)

---

### Tutorial 8: Object Following (2 hours)
**What you'll build:** Robot that follows colored objects

**Components needed:**
- Camera
- Raspberry Pi or Jetson Nano

**Skills learned:**
- Computer vision basics
- Object detection
- Tracking algorithms

[Start Tutorial 8](./tutorials/08-object-following.md)

---

### Tutorial 9: Voice Control (1.5 hours)
**What you'll build:** Robot controlled by voice commands

**Components needed:**
- Microphone
- Raspberry Pi

**Skills learned:**
- Speech recognition
- Natural language processing
- Audio processing

[Start Tutorial 9](./tutorials/09-voice-control.md)

---

### Tutorial 10: Autonomous Navigation (3 hours)
**What you'll build:** Robot that maps and navigates

**Components needed:**
- LIDAR or depth camera
- IMU
- Powerful computer (Pi 4 or better)

**Skills learned:**
- SLAM basics
- Path planning
- Localization

[Start Tutorial 10](./tutorials/10-autonomous-navigation.md)

---

## 🎓 Student Path

**Goal:** Learn robotics for school/competitions  
**Time:** 20-25 hours  
**Prerequisites:** Complete Beginner Path

### Tutorial 11: Understanding Coordinate Frames (2 hours)
**What you'll learn:** Spatial transformations, reference frames

```maneuver
frame world
frame robot_base
frame camera

robot_base.set_parent(world):
    position: (1m, 2m, 0m)
    rotation: 45 degrees around z

point_world = (5m, 3m, 0m) in world
point_robot = point_world.transform_to(robot_base)
```

**Exercise:** Calculate relative positions

[Start Tutorial 11](./tutorials/11-coordinate-frames.md)

---

### Tutorial 12: Introduction to Kinematics (2.5 hours)
**What you'll learn:** Forward/inverse kinematics

**Topics:**
- DH parameters
- Joint space vs task space
- Reachability

[Start Tutorial 12](./tutorials/12-kinematics.md)

---

### Tutorial 13: PID Control Deep Dive (2 hours)
**What you'll learn:** Tuning controllers

```maneuver
pid: PIDController = PIDController(
    kp: 1.0,
    ki: 0.1,
    kd: 0.05
)

loop:
    error = setpoint - current_value
    output = pid.compute(error, dt)
    apply_control(output)
```

**Exercise:** Tune PID for stable control

[Start Tutorial 13](./tutorials/13-pid-control.md)

---

### Tutorial 14: Sensor Fusion (2.5 hours)
**What you'll learn:** Combining multiple sensors

**Topics:**
- Kalman filters
- Complementary filters
- Sensor noise handling

[Start Tutorial 14](./tutorials/14-sensor-fusion.md)

---

### Tutorial 15: Path Planning Algorithms (3 hours)
**What you'll learn:** A*, RRT, Dijkstra

**Implementation:**
- Grid-based planning
- Sampling-based planning
- Optimization-based planning

[Start Tutorial 15](./tutorials/15-path-planning.md)

---

### Tutorial 16: Computer Vision Basics (2.5 hours)
**What you'll learn:** Image processing pipeline

**Topics:**
- Color spaces
- Filtering
- Edge detection
- Object detection

[Start Tutorial 16](./tutorials/16-computer-vision.md)

---

### Tutorial 17: Building a Competition Robot (4 hours)
**What you'll build:** Complete competition-ready robot

**Features:**
- Autonomous and manual modes
- Multiple strategies
- Robust error handling
- Quick debugging

[Start Tutorial 17](./tutorials/17-competition-robot.md)

---

## 💼 Professional Path

**Goal:** Production-ready robotics systems  
**Time:** 25-30 hours  
**Prerequisites:** Complete Beginner + Student Paths

### Tutorial 18: Real-Time Systems (3 hours)
**What you'll learn:** Meeting timing constraints

```maneuver
task critical_control:
    frequency: 1000 Hz
    deadline: 900μs
    priority: critical
    
    // Guaranteed to meet timing
    data = read_sensors()
    control = compute_control(data)
    send_commands(control)
```

**Topics:**
- Real-time scheduling
- Deadline guarantees
- Priority inversion
- Resource management

[Start Tutorial 18](./tutorials/18-realtime-systems.md)

---

### Tutorial 19: Safety and Reliability (2.5 hours)
**What you'll learn:** Building safe robots

**Topics:**
- Formal verification
- Fault tolerance
- Emergency stops
- Safety certifications

[Start Tutorial 19](./tutorials/19-safety-reliability.md)

---

### Tutorial 20: Performance Optimization (3 hours)
**What you'll learn:** Making code faster

**Techniques:**
- Profiling
- GPU acceleration
- Algorithm optimization
- Memory optimization

[Start Tutorial 20](./tutorials/20-performance-optimization.md)

---

### Tutorial 21: Multi-Robot Systems (3 hours)
**What you'll learn:** Coordinating robot fleets

```maneuver
// Robot 1
broadcast my_position to fleet

// Robot 2
on message from fleet:
    if message.type == "position":
        update_formation(message.position)
```

**Topics:**
- Communication protocols
- Distributed algorithms
- Swarm intelligence

[Start Tutorial 21](./tutorials/21-multi-robot-systems.md)

---

### Tutorial 22: Industrial Integration (2.5 hours)
**What you'll learn:** Connecting to industrial systems

**Topics:**
- PLC integration
- OPC UA
- Safety PLCs
- Industrial protocols

[Start Tutorial 22](./tutorials/22-industrial-integration.md)

---

### Tutorial 23: Testing and CI/CD (2 hours)
**What you'll learn:** Professional development practices

```maneuver
#[test]
function test_forward_kinematics():
    angles = [0°, 0°, 0°, 0°, 0°, 0°]
    pose = ur5.forward_kinematics(angles)
    assert pose.position.z == 0.817m within 0.001m
```

**Topics:**
- Unit testing
- Integration testing
- Hardware-in-the-loop testing
- Continuous deployment

[Start Tutorial 23](./tutorials/23-testing-cicd.md)

---

### Tutorial 24: Documentation and Maintenance (2 hours)
**What you'll learn:** Long-term project management

**Topics:**
- Code documentation
- API design
- Version control
- Maintenance strategies

[Start Tutorial 24](./tutorials/24-documentation-maintenance.md)

---

## 🔬 Researcher Path

**Goal:** Advanced algorithms and novel research  
**Time:** 30-40 hours  
**Prerequisites:** Complete all previous paths

### Tutorial 25: Advanced Kinematics (4 hours)
**What you'll learn:** Complex manipulator control

**Topics:**
- Differential kinematics
- Redundancy resolution
- Singularity avoidance
- Task-space control

[Start Tutorial 25](./tutorials/25-advanced-kinematics.md)

---

### Tutorial 26: Model Predictive Control (4 hours)
**What you'll learn:** Optimal control

```maneuver
mpc: MPCController = MPCController(
    model: robot_dynamics,
    horizon: 10,
    cost: quadratic_cost,
    constraints: state_constraints
)

optimal_trajectory = mpc.solve(current_state, goal_state)
```

**Topics:**
- Optimization theory
- Constraint handling
- Receding horizon control

[Start Tutorial 26](./tutorials/26-model-predictive-control.md)

---

### Tutorial 27: SLAM Deep Dive (5 hours)
**What you'll learn:** Simultaneous Localization and Mapping

**Topics:**
- FastSLAM
- Graph-based SLAM
- Visual SLAM
- Loop closure detection

[Start Tutorial 27](./tutorials/27-slam-deep-dive.md)

---

### Tutorial 28: Machine Learning Integration (4 hours)
**What you'll learn:** Using ML in robotics

**Topics:**
- Model training
- Inference optimization
- Transfer learning
- End-to-end learning

[Start Tutorial 28](./tutorials/28-machine-learning.md)

---

### Tutorial 29: Reinforcement Learning for Robotics (5 hours)
**What you'll learn:** Learning-based control

**Topics:**
- Policy gradient methods
- Q-learning
- Sim-to-real transfer
- Safe RL

[Start Tutorial 29](./tutorials/29-reinforcement-learning.md)

---

### Tutorial 30: Custom Algorithms (4 hours)
**What you'll learn:** Implementing research papers

**Topics:**
- Algorithm analysis
- Implementation best practices
- Benchmarking
- Publication-quality code

[Start Tutorial 30](./tutorials/30-custom-algorithms.md)

---

## 🎯 Specialized Topics

### Mobile Manipulation
- Combining navigation with manipulation
- Dynamic base repositioning
- Whole-body control

### Aerial Robotics
- Quadrotor control
- Vision-based navigation
- Swarm coordination

### Humanoid Robotics
- Bipedal locomotion
- Balance control
- Human-robot interaction

### Soft Robotics
- Compliant control
- Soft sensor integration
- Novel actuation

---

## 📚 Tutorial Format

Each tutorial includes:

### 1. Overview
- Learning objectives
- Prerequisites
- Time estimate
- Difficulty level

### 2. Theory
- Concepts explained
- Mathematical foundations (when relevant)
- Real-world applications

### 3. Hands-On
- Step-by-step instructions
- Code examples
- Hardware setup (if needed)

### 4. Exercises
- Practice problems
- Challenges
- Solutions provided

### 5. Projects
- Capstone project
- Extensions
- Further exploration

---

## 💡 Learning Tips

### For Beginners
1. **Don't skip tutorials** - Build foundation properly
2. **Type code manually** - Don't copy-paste
3. **Experiment** - Try changing things
4. **Ask questions** - Use Discord/Forum
5. **Build something** - Apply what you learn

### For Intermediate
1. **Focus on understanding** - Not just doing
2. **Read documentation** - Learn to find answers
3. **Optimize later** - Make it work first
4. **Share projects** - Get feedback
5. **Help others** - Teaching reinforces learning

### For Advanced
1. **Read research papers** - Stay current
2. **Contribute** - Improve libraries
3. **Benchmark** - Measure everything
4. **Document** - Share knowledge
5. **Innovate** - Combine concepts creatively

---

## 🎓 Certifications (Coming Soon)

### MANEUVER Certified Developer
- Complete 20 core tutorials
- Pass practical exam
- Build capstone project

### MANEUVER Certified Professional
- Advanced tutorials
- Production deployment
- Code review participation

### MANEUVER Certified Instructor
- Deep understanding
- Teaching experience
- Community contribution

---

## 📊 Progress Tracking

Track your learning:

```bash
# Check your progress
maneuver tutorials progress

# Mark tutorial complete
maneuver tutorials complete 01-first-robot

# Get recommendations
maneuver tutorials recommend
```

---

## 🤝 Contributing Tutorials

Want to create a tutorial?

1. **Choose a topic** - Fill a gap
2. **Follow format** - Use template
3. **Test thoroughly** - With real users
4. **Submit PR** - Share with community

See [tutorial template](./tutorials/TEMPLATE.md)

---

## 📞 Getting Help

**Stuck on a tutorial?**

1. **Re-read carefully** - Solution might be there
2. **Check FAQ** - Common issues documented
3. **Ask on Discord** - #tutorials channel
4. **Search forum** - Others may have asked
5. **Create issue** - If it's a bug or unclear

**Community support:**
- Average response time: < 2 hours
- Friendly, helpful community
- No stupid questions!

---

## 🗺️ Suggested Order

### Complete Beginner
```
Tutorials 1-5 → Build simple project → Tutorials 6-10
```

### Some Programming Experience
```
Tutorials 1-3 (review) → 6-10 → 11-17
```

### Experienced Developer, New to Robotics
```
Tutorials 1, 2, 11-17 → 18-24
```

### Robotics Student/Professional
```
Pick relevant tutorials → Focus on areas of interest
```

---

## 📈 Learning Metrics

After completing tutorials, you'll be able to:

**Beginner Path (1-5):**
- ✅ Write basic robot programs
- ✅ Use sensors and actuators
- ✅ Understand control flow
- ✅ Debug simple issues

**Hobbyist Path (6-10):**
- ✅ Build complete robots
- ✅ Implement vision-based control
- ✅ Create autonomous behaviors
- ✅ Share projects online

**Student Path (11-17):**
- ✅ Understand robotics theory
- ✅ Implement algorithms
- ✅ Compete in competitions
- ✅ Complete class projects

**Professional Path (18-24):**
- ✅ Deploy production systems
- ✅ Ensure safety and reliability
- ✅ Optimize performance
- ✅ Maintain large codebases

**Researcher Path (25-30):**
- ✅ Implement research papers
- ✅ Develop novel algorithms
- ✅ Publish results
- ✅ Advance the field

---

## 🎉 After Completing Tutorials

### Keep Learning
- Read research papers
- Try challenges
- Attend conferences
- Take advanced courses

### Build Portfolio
- GitHub projects
- Demo videos
- Technical blog
- Open source contributions

### Give Back
- Answer questions
- Write tutorials
- Mentor beginners
- Improve documentation

### Go Pro
- Freelance projects
- Full-time robotics
- Start a company
- Research career

---

<div align="center">

**Ready to start your robotics journey?**

[Begin with Tutorial 1](./tutorials/01-first-robot.md) • [Join Discord](https://discord.gg/maneuver) • [Browse Examples](./examples)

**Questions about learning path?**

[Ask on Forum](https://discuss.maneuver-lang.org) • [Read FAQ](./FAQ.md)

</div>
