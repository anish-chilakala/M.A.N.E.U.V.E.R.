# Getting Started with MANEUVER

> **Your First Robot in 10 Minutes**

[![Difficulty](https://img.shields.io/badge/Difficulty-Beginner-green)]()
[![Time](https://img.shields.io/badge/Time-10_minutes-blue)]()
[![Prerequisites](https://img.shields.io/badge/Prerequisites-None-success)]()

---

## 🎯 What You'll Learn

By the end of this guide, you'll:
- ✅ Install MANEUVER on your system
- ✅ Write your first robot program
- ✅ Understand basic syntax
- ✅ Run code in simulation
- ✅ Deploy to real hardware (optional)

**No prior robotics or programming experience required!**

---

## 📦 Installation

### Step 1: Install MANEUVER

**Linux / macOS:**
```bash
curl -sSf https://maneuver-lang.org/install.sh | sh
```

**Windows:**
```powershell
irm https://maneuver-lang.org/install.ps1 | iex
```

**From Source:**
```bash
git clone https://github.com/maneuver-lang/maneuver
cd maneuver
cargo install --path .
```

### Step 2: Verify Installation

```bash
maneuver --version
# Output: MANEUVER 1.0.0
```

### Step 3: Install Simulator (Optional)

```bash
maneuver install simulator
```

---

## 🤖 Your First Robot Program

### Hello, Robot!

Create a file called `first_robot.mnvr`:

```maneuver
// first_robot.mnvr - Your first MANEUVER program!

robot my_first_bot:
    say "Hello, World!"
    wait 1 second
    say "I am alive!"
```

### Run It!

```bash
maneuver run first_robot.mnvr
```

**Output:**
```
🤖 Starting robot: my_first_bot
Hello, World!
[waiting 1 second...]
I am alive!
✅ Robot my_first_bot finished!
```

**Congratulations! You just wrote your first robot program! 🎉**

---

## 🚶 Making Your Robot Move

### Moving Forward

```maneuver
robot mobile_bot:
    say "Starting to move"
    move forward 50cm
    say "I moved 50 centimeters!"
```

**Run it:**
```bash
maneuver run mobile_bot.mnvr --simulate
```

You'll see a visualization of your robot moving forward!

### Turning

```maneuver
robot mobile_bot:
    move forward 50cm
    turn right 90 degrees
    move forward 30cm
    say "I made an L shape!"
```

### Complete Square

```maneuver
robot mobile_bot:
    say "Drawing a square"
    
    // Repeat 4 times
    repeat 4 times:
        move forward 50cm
        turn right 90 degrees
    
    say "Square complete!"
```

---

## 🎮 Interactive Control

### Using Sensors

```maneuver
robot mobile_bot:
    // Define a sensor
    sensor distance_sensor:
        type: ultrasonic
        location: front
        range: 0cm to 400cm
    
    // Main behavior
    say "Obstacle avoidance mode"
    
    loop:
        if distance_sensor < 20cm:
            say "Obstacle detected!"
            stop
            turn right 90 degrees
            move forward 10cm
        else:
            move forward 5cm
```

**What's happening:**
1. Robot moves forward
2. If something is closer than 20cm, it stops and turns
3. Otherwise, keeps moving
4. Repeats forever

---

## 📐 Understanding the Basics

### 1. Comments

```maneuver
// This is a single-line comment

/* This is a
   multi-line
   comment */
```

### 2. Robot Declaration

```maneuver
robot <name>:
    // Your code here
```

Every program starts with `robot` followed by a name.

### 3. Commands

```maneuver
move forward 50cm          // Movement
turn left 45 degrees       // Rotation
wait 2 seconds            // Pause
say "Hello"               // Output message
stop                      // Stop all motion
```

### 4. Units

**MANEUVER understands physical units!**

```maneuver
// Distance
10cm, 5 meters, 2.5m, 100 millimeters

// Angles
90 degrees, 45°, π/4 radians

// Time
1 second, 500ms, 2.5 seconds

// Speed
0.5 m/s, 10 cm/s
```

**You can't mix incompatible units:**
```maneuver
distance: 10cm + 5cm        // ✓ Works! (15cm)
mistake: 10cm + 5 seconds   // ✗ Compiler error!
```

### 5. Variables

```maneuver
robot mobile_bot:
    // Declare variables with types
    speed: 0.5 m/s
    distance: 100cm
    
    // Use them
    move forward distance at speed
```

### 6. Loops

**Repeat a fixed number of times:**
```maneuver
repeat 5 times:
    move forward 10cm
    wait 0.5 seconds
```

**Loop while a condition is true:**
```maneuver
loop while battery_level > 20%:
    move forward 10cm
    
say "Battery low, stopping"
```

**Loop forever:**
```maneuver
loop:
    // Keep running forever
    blink LED
    wait 1 second
```

### 7. Conditionals

```maneuver
if distance_sensor < 20cm:
    turn right 90 degrees
else if distance_sensor < 50cm:
    move forward slowly
else:
    move forward at full_speed
```

---

## 🎓 Example Programs

### 1. Line Following Robot

```maneuver
robot line_follower:
    sensor left_sensor: infrared at left
    sensor right_sensor: infrared at right
    
    loop:
        if left_sensor detects_line and right_sensor detects_line:
            // On line, go straight
            move forward 5cm
            
        else if left_sensor detects_line:
            // Veering right, turn left
            turn left 5 degrees
            
        else if right_sensor detects_line:
            // Veering left, turn right
            turn right 5 degrees
            
        else:
            // Lost the line, search
            turn right 10 degrees
```

### 2. Patrol Robot

```maneuver
robot patrol_bot:
    // Define patrol points
    waypoints: [
        (x: 0m, y: 0m),
        (x: 2m, y: 0m),
        (x: 2m, y: 2m),
        (x: 0m, y: 2m),
    ]
    
    loop:
        for waypoint in waypoints:
            say "Moving to next waypoint"
            move to waypoint
            wait 2 seconds
            
        say "Patrol complete, restarting"
```

### 3. Robotic Arm Pick and Place

```maneuver
arm robot_arm:
    // Define the arm
    joints: 6
    reach: 80cm
    
task pick_and_place:
    // Object location
    object_pos: (x: 30cm, y: 15cm, z: 5cm)
    target_pos: (x: 0cm, y: 40cm, z: 20cm)
    
    say "Starting pick and place"
    
    // Approach object
    move arm to (object_pos + offset(z: 10cm))
    open gripper
    
    // Pick up
    move arm to object_pos
    close gripper with gentle force
    wait 0.5 seconds
    
    // Lift
    move arm to (object_pos + offset(z: 10cm))
    
    // Move to target
    move arm to (target_pos + offset(z: 10cm))
    move arm to target_pos
    
    // Release
    open gripper
    wait 0.5 seconds
    
    // Return home
    move arm to home_position
    
    say "Pick and place complete!"
```

---

## 🎮 Using the Simulator

### Launch Simulator

```bash
maneuver simulate first_robot.mnvr
```

**You'll see:**
- 3D visualization of your robot
- Real-time sensor data
- Movement traces
- Speed controls

### Simulator Controls

- **Space**: Pause/Resume
- **R**: Reset simulation
- **Arrow Keys**: Manual control
- **+/-**: Speed up/slow down
- **C**: Toggle camera view

---

## 🔧 Hardware Deployment

### Step 1: Configure Hardware

Create `robot_config.toml`:

```toml
[hardware]
platform = "raspberry-pi"

[motors]
left_motor = { pin = 17, type = "DC" }
right_motor = { pin = 18, type = "DC" }

[sensors]
distance_sensor = { pin = 27, type = "ultrasonic" }
```

### Step 2: Compile for Target

```bash
maneuver build --target raspberry-pi first_robot.mnvr
```

### Step 3: Deploy

```bash
# Copy to robot
scp build/first_robot pi@robot.local:/home/pi/

# SSH and run
ssh pi@robot.local
./first_robot
```

---

## 🐛 Debugging

### Common Errors

**Error: "Expected unit, found number"**
```maneuver
move forward 50  // ✗ Wrong - no unit!
move forward 50cm  // ✓ Correct
```

**Error: "Cannot add incompatible units"**
```maneuver
total: 10cm + 5 seconds  // ✗ Can't add distance + time
total: 10cm + 5cm        // ✓ Correct
```

**Error: "Robot not defined"**
```maneuver
move forward 50cm  // ✗ No robot declared!

robot mobile_bot:  // ✓ Correct - declare robot first
    move forward 50cm
```

### Debug Mode

```bash
maneuver run --debug first_robot.mnvr
```

Shows:
- Each command as it executes
- Variable values
- Sensor readings
- Timing information

### Verbose Output

```bash
maneuver run --verbose first_robot.mnvr
```

---

## 📚 Next Steps

### Learn More

1. **Read the Language Guide**
   ```bash
   maneuver docs guide
   ```

2. **Explore Examples**
   ```bash
   maneuver examples list
   maneuver examples run line-follower
   ```

3. **Try Tutorials**
   - [Tutorial 1: Sensors and Input](./tutorials/01-sensors.md)
   - [Tutorial 2: Complex Movement](./tutorials/02-movement.md)
   - [Tutorial 3: Robotic Arms](./tutorials/03-arms.md)
   - [Tutorial 4: Autonomous Navigation](./tutorials/04-navigation.md)

### Join the Community

- 💬 [Discord Server](https://discord.gg/maneuver)
- 📖 [Forum](https://discuss.maneuver-lang.org)
- 🐦 [Twitter](https://twitter.com/maneuverlang)
- 📧 [Mailing List](https://groups.google.com/g/maneuver-lang)

### Build Something!

Share your projects:
- Tag us on social media: `#MANEUVERlang`
- Post in [Show & Tell](https://discuss.maneuver-lang.org/c/show-and-tell)
- Submit to [Awesome MANEUVER](https://github.com/maneuver-lang/awesome-maneuver)

---

## 💡 Quick Reference Card

### Essential Commands

```maneuver
// Movement
move forward <distance>
move backward <distance>
turn left <angle>
turn right <angle>
stop

// Control Flow
if <condition>:
    // code
else:
    // code

loop while <condition>:
    // code

repeat <n> times:
    // code

// Sensors
sensor <name>:
    type: <type>
    location: <location>

// Wait
wait <duration>

// Output
say "message"
```

### Data Types

```maneuver
// Numbers
count: 10
speed: 0.5

// Distances
distance: 50cm
far: 2 meters

// Angles
angle: 90 degrees
radians: π/4

// Time
duration: 2 seconds
fast: 100ms

// Booleans
moving: true
stopped: false

// Strings
message: "Hello"
```

---

## 🎯 Exercises

### Exercise 1: Figure Eight

Write a program that makes the robot draw a figure-eight pattern.

<details>
<summary>Click to see solution</summary>

```maneuver
robot figure_eight:
    repeat 2 times:
        // First loop
        repeat 8 times:
            move forward 10cm
            turn left 45 degrees
        
        // Second loop (opposite direction)
        repeat 8 times:
            move forward 10cm
            turn right 45 degrees
```
</details>

### Exercise 2: Smart Parking

Write a program that makes the robot park itself between two obstacles.

<details>
<summary>Click to see solution</summary>

```maneuver
robot parking_bot:
    sensor front: ultrasonic at front
    sensor left: ultrasonic at left
    sensor right: ultrasonic at right
    
    // Find parking spot
    loop while true:
        move forward 5cm
        
        if left > 50cm and right > 50cm:
            // Found a spot!
            stop
            break
    
    // Center in the spot
    if left > right:
        turn left 10 degrees
        move forward 10cm
    else:
        turn right 10 degrees
        move forward 10cm
    
    say "Parked successfully!"
```
</details>

### Exercise 3: Following a Wall

Make the robot follow a wall on its right side.

<details>
<summary>Click to see solution</summary>

```maneuver
robot wall_follower:
    sensor right: ultrasonic at right
    target_distance: 20cm
    
    loop:
        if right < target_distance - 5cm:
            // Too close to wall
            turn left 10 degrees
        else if right > target_distance + 5cm:
            // Too far from wall
            turn right 10 degrees
        else:
            // Perfect distance
            move forward 5cm
```
</details>

---

## ❓ FAQ

**Q: Do I need to know C++ or Python?**
A: No! MANEUVER is designed for beginners. If you can read English, you can read MANEUVER.

**Q: What hardware does MANEUVER support?**
A: Arduino, Raspberry Pi, NVIDIA Jetson, and many more. See [HARDWARE.md](./HARDWARE.md).

**Q: Can I use MANEUVER for my school project?**
A: Absolutely! It's perfect for learning robotics.

**Q: Is MANEUVER suitable for professional robots?**
A: Yes! It's designed to scale from hobby projects to industrial robots.

**Q: How do I get help?**
A: Join our [Discord](https://discord.gg/maneuver) - friendly community ready to help!

**Q: Can I mix MANEUVER with other languages?**
A: Yes! You can call C/C++ libraries from MANEUVER.

---

## 🎉 You Did It!

You've learned:
- ✅ How to install MANEUVER
- ✅ Basic syntax and commands
- ✅ How to control a robot
- ✅ How to use sensors
- ✅ How to run simulations

**Now go build something amazing! 🚀**

---

<div align="center">

**Questions? Get help:**

[Discord](https://discord.gg/maneuver) • [Forum](https://discuss.maneuver-lang.org) • [Documentation](./docs) • [Examples](./examples)

**Ready for more?**

[Language Guide](./LANGUAGE-GUIDE.md) • [Advanced Tutorials](./tutorials) • [API Reference](./API-REFERENCE.md)

</div>
