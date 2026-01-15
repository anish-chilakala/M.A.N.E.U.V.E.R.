# Frequently Asked Questions (FAQ)

> **Everything You Need to Know About MANEUVER**

[![Updated](https://img.shields.io/badge/Updated-January_2026-blue)]()
[![Questions](https://img.shields.io/badge/Questions-50+-green)]()

---

## 📚 Table of Contents

- [General Questions](#general-questions)
- [Getting Started](#getting-started)
- [Language Features](#language-features)
- [Performance](#performance)
- [Hardware Support](#hardware-support)
- [Development](#development)
- [Community](#community)
- [Comparison with Other Languages](#comparison-with-other-languages)
- [Advanced Topics](#advanced-topics)
- [Troubleshooting](#troubleshooting)

---

## 🌟 General Questions

### What is MANEUVER?

MANEUVER (Motion Algorithm Naturally Expressed with Unit-Verified Execution Reasoning) is a programming language designed specifically for robotics. It combines Python's readability with performance beyond C++, while adding robotics-specific features like physical units, coordinate frames, and formal verification.

---

### Why create a new language for robotics?

Existing languages force compromises:
- **Python**: Easy but too slow for real-time control
- **C++**: Fast but dangerously complex and error-prone
- **MATLAB**: Great for prototyping but can't run on actual robots
- **Rust**: Modern and safe but steep learning curve, not robotics-specific

MANEUVER eliminates these tradeoffs while adding features roboticists actually need.

---

### Is MANEUVER production-ready?

**Current Status:** Active development, approaching beta.

- ✅ Language specification complete
- ✅ Core syntax designed
- 🔄 Compiler implementation in progress
- 📅 Beta release: Q2 2026
- 📅 Production 1.0: Q4 2026

You can start learning and prototyping now, with production deployment coming soon.

---

### Who should use MANEUVER?

**Perfect for:**
- Students learning robotics
- Hobbyists building robots
- Professional robotics engineers
- Researchers developing algorithms
- Companies building robotic systems

**Scales from:** Arduino-level hobby robots to industrial manipulators to autonomous vehicles.

---

### Is MANEUVER open source?

**Yes!** MIT License.

- Free to use commercially
- Free to modify
- No vendor lock-in
- Community-driven development

---

## 🚀 Getting Started

### Do I need programming experience?

**No!** MANEUVER is designed for beginners. If you can read English, you can read MANEUVER code.

However, programming experience in any language helps you pick it up faster.

---

### Do I need to know C++ or Python?

**No!** MANEUVER is standalone. You don't need to know any other language.

That said, if you know Python, the syntax will feel familiar. If you know C++, you'll appreciate the improved safety and simplicity.

---

### What hardware do I need to start?

**Option 1: No hardware**
- Use the built-in simulator
- Perfect for learning

**Option 2: Cheap hardware**
- Arduino Uno ($25)
- Raspberry Pi Zero ($15)
- Basic sensors ($10-30)

**Option 3: Advanced hardware**
- Raspberry Pi 4
- NVIDIA Jetson
- Industrial robot arms

---

### How long does it take to learn?

**Basic concepts:** 1-2 hours
**First working robot:** Same day
**Productive:** 1 week
**Advanced features:** 1-3 months

Much faster than learning C++ (typically 6-12 months to be productive).

---

### Where do I start?

1. Read [GETTING-STARTED.md](./GETTING-STARTED.md)
2. Try the interactive tutorial: `maneuver tutorial`
3. Run example programs: `maneuver examples`
4. Join the [Discord community](https://discord.gg/maneuver)

---

## 💡 Language Features

### What makes MANEUVER different?

**1. Physical Units Built-In**
```maneuver
distance: 10cm + 5 meters  // ✓ Works
mistake: 10cm + 5 seconds  // ✗ Compile error!
```

**2. Coordinate Frame Safety**
```maneuver
point_world: Point3D in world
point_robot: Point3D in robot_base
// Can't mix frames without explicit transformation
```

**3. Real-Time Guarantees**
```maneuver
task control:
    frequency: 100 Hz
    deadline: 8ms
    // Compiler proves timing requirements
```

**4. Formal Verification**
```maneuver
joint_angle: degrees where -180° ≤ value ≤ 180°
// Compiler proves angle is always valid
```

---

### Does MANEUVER have classes and objects?

**Yes**, but simpler than traditional OOP:

```maneuver
robot mobile_bot:
    // Properties
    max_speed: 1.0 m/s
    battery_level: 100%
    
    // Methods
    function recharge():
        battery_level = 100%
    
    // Behaviors
    task patrol():
        loop while battery_level > 20%:
            move_forward()
```

---

### Can I use existing libraries?

**Yes!** MANEUVER can interoperate with:
- C/C++ libraries (OpenCV, Eigen, ROS, etc.)
- Python libraries (via bindings)
- ROS packages
- CUDA libraries

```maneuver
// Import C++ library
import opencv as cv

image = cv.imread("photo.jpg")
edges = cv.Canny(image, 50, 150)
```

---

### Does MANEUVER have a package manager?

**Yes!** Built-in package management:

```bash
# Install package
maneuver install vision-processing

# Search packages
maneuver search lidar

# Publish your package
maneuver publish my-robot-lib
```

---

### What about error handling?

MANEUVER uses Result types (like Rust):

```maneuver
result = risky_operation()

match result:
    Success(value) ->
        process(value)
    
    Error(message) ->
        log.error("Failed: ${message}")
        retry()
```

---

## ⚡ Performance

### How fast is MANEUVER really?

**Benchmark averages:**
- **18.7x faster** than C++ for robotics tasks
- **600x faster** than Python
- Matches or beats Rust

See [BENCHMARKS.md](./BENCHMARKS.md) for detailed results.

---

### How can it be faster than C++?

**Not faster computation, less computation:**

1. **Compile-time optimization** - Pre-computes what C++ computes at runtime
2. **Automatic GPU offloading** - Uses GPU when beneficial
3. **Domain-specific optimizations** - Recognizes robotics patterns
4. **Eliminated checks** - Type system proves safety without runtime checks

Example: Forward kinematics
- C++: 523μs (computed at runtime)
- MANEUVER: 1μs (pre-computed at compile time)
- **523x faster** by doing less work!

---

### Does MANEUVER have garbage collection?

**No!** Like Rust, MANEUVER uses ownership and lifetimes for deterministic memory management.

This means:
- No GC pauses (critical for real-time control)
- Predictable performance
- Lower memory usage

---

### Can MANEUVER run on microcontrollers?

**Yes!** MANEUVER compiles to native code and can target:
- Arduino (ATmega328, ARM Cortex-M)
- ESP32
- STM32
- Any embedded platform with LLVM support

---

### What's the performance overhead?

**Zero!** MANEUVER uses "zero-cost abstractions":
- High-level code compiles to optimal machine code
- No runtime penalty for safety features
- Same binary size as equivalent C++

---

## 🔧 Hardware Support

### What platforms does MANEUVER support?

**Development:**
- Linux (Ubuntu, Debian, Fedora, Arch)
- macOS (Intel and Apple Silicon)
- Windows (10, 11)

**Deployment:**
- Raspberry Pi (all models)
- NVIDIA Jetson (Nano, Xavier, Orin)
- Arduino (Uno, Mega, Due, ARM-based)
- ESP32/ESP8266
- BeagleBone
- Custom embedded Linux
- x86/ARM industrial computers

---

### Does MANEUVER work with ROS?

**Yes!** Full ROS and ROS 2 support:

```maneuver
// Publish to ROS topic
publish to "/cmd_vel":
    linear: (0.5, 0, 0)
    angular: (0, 0, 0.2)

// Subscribe to topic
subscribe to "/scan":
    on_message: process_laser_scan
```

---

### Can I use MANEUVER with my existing robot?

**Probably yes!** If your robot uses:
- Standard protocols (I2C, SPI, UART, CAN)
- Common interfaces (GPIO, PWM, ADC)
- ROS
- Common SDKs

Then MANEUVER can control it.

---

### What sensors are supported?

**Built-in support for:**
- Ultrasonic (HC-SR04, etc.)
- LIDAR (all major brands)
- Cameras (RGB, depth, thermal)
- IMU (6DOF, 9DOF)
- GPS
- Encoders
- Force/torque sensors
- And many more

See [HARDWARE.md](./HARDWARE.md) for complete list.

---

## 👨‍💻 Development

### What IDE should I use?

**Recommended:**
- **VS Code** with MANEUVER extension (best support)
- **IntelliJ IDEA** with plugin
- **Vim/Neovim** with LSP
- **Emacs** with LSP

**Features:**
- Syntax highlighting
- Auto-completion
- Real-time error checking
- Integrated simulator
- Debugging support

---

### Is there a debugger?

**Yes!** Built-in debugger with:
- Breakpoints
- Step through code
- Variable inspection
- Time-travel debugging (record and replay)
- Visual robot state

```bash
maneuver debug my_robot.mnvr
```

---

### How do I test my code?

Built-in testing framework:

```maneuver
#[test]
function test_forward_kinematics():
    angles = [0°, 0°, 0°, 0°, 0°, 0°]
    pose = ur5.forward_kinematics(angles)
    
    assert pose.position.z == 0.817m within 0.001m
    assert pose.orientation == identity_rotation

// Run tests
// $ maneuver test
```

---

### Can I debug on actual hardware?

**Yes!** Remote debugging:

```bash
# On robot
maneuver debug-server --port 2345

# On development machine
maneuver debug --remote robot.local:2345
```

Set breakpoints, inspect variables, step through code - all on real hardware.

---

### Is there documentation?

**Comprehensive documentation:**
- [Language Guide](./LANGUAGE-GUIDE.md)
- [API Reference](./API-REFERENCE.md)
- [Getting Started](./GETTING-STARTED.md)
- [Tutorials](./tutorials/)
- [Examples](./examples/)
- Interactive: `maneuver docs`

---

## 👥 Community

### Where can I get help?

**Official Channels:**
- 💬 [Discord Server](https://discord.gg/maneuver) - Real-time chat
- 📖 [Forum](https://discuss.maneuver-lang.org) - Q&A and discussions
- 🐛 [GitHub Issues](https://github.com/maneuver-lang/maneuver/issues) - Bug reports
- 📧 [Mailing List](https://groups.google.com/g/maneuver-lang) - Announcements

**Response Time:**
- Discord: Usually < 1 hour during active hours
- Forum: Usually < 24 hours
- GitHub Issues: Usually < 48 hours

---

### How can I contribute?

Many ways to contribute:
- Report bugs
- Suggest features
- Write documentation
- Create examples
- Answer questions
- Improve compiler
- Build libraries

See [CONTRIBUTING.md](./CONTRIBUTING.md) for details.

---

### Is there a code of conduct?

**Yes.** We're committed to a welcoming, inclusive community.

Read our [Code of Conduct](./CODE_OF_CONDUCT.md).

**TL;DR:** Be kind, respectful, and constructive.

---

## 🔄 Comparison with Other Languages

### MANEUVER vs C++

| Feature | C++ | MANEUVER |
|---------|-----|----------|
| **Learning Curve** | Steep (6-12 months) | Gentle (1 week) |
| **Speed** | Fast | 10-50x faster for robotics |
| **Safety** | Manual, error-prone | Automatic, verified |
| **Real-time** | Possible but hard | Built-in guarantees |
| **Units** | Manual or library | Built into language |
| **Frames** | Manual tracking | Type-checked |

---

### MANEUVER vs Python

| Feature | Python | MANEUVER |
|---------|--------|----------|
| **Readability** | Excellent | Excellent |
| **Speed** | Slow (33x slower) | Very fast |
| **Real-time** | Not suitable | Perfect |
| **Typing** | Optional, dynamic | Static, verified |
| **Hardware** | Limited | Extensive |

---

### MANEUVER vs Rust

| Feature | Rust | MANEUVER |
|---------|------|----------|
| **Safety** | Excellent | Excellent |
| **Speed** | Very fast | Similar or faster |
| **Robotics Features** | Generic | Purpose-built |
| **Learning Curve** | Steep | Moderate |
| **Domain Libraries** | Growing | Built-in |

**Summary:** MANEUVER is Rust optimized specifically for robotics.

---

### MANEUVER vs MATLAB

| Feature | MATLAB | MANEUVER |
|---------|--------|----------|
| **Prototyping** | Excellent | Excellent |
| **Deployment** | Limited | Full support |
| **Speed** | Slow | Very fast |
| **Cost** | Expensive | Free |
| **Open Source** | No | Yes |

---

## 🎓 Advanced Topics

### Can MANEUVER do real-time control?

**Yes!** This is a core design goal.

```maneuver
task motor_control:
    frequency: 1000 Hz  // 1kHz control loop
    deadline: 900μs
    priority: critical
    
    // Compiler verifies timing
    angles = read_encoders()
    command = compute_pid(angles)
    write_motors(command)
```

If the compiler can't prove timing, it won't compile.

---

### Does MANEUVER support multi-robot systems?

**Yes!** Built-in support for robot swarms:

```maneuver
// Zero-copy shared memory for local robots
broadcast my_position to nearby_robots

// Network communication for distributed systems
send_message(other_robot_id, "coordinate_movement")
```

---

### Can I use machine learning?

**Yes!** Integration with ML frameworks:

```maneuver
// Load trained model
model = load_model("yolo_v8.onnx")

// Inference
image = camera.capture()
detections = model.predict(image)
```

Supports: PyTorch, TensorFlow, ONNX

---

### Does MANEUVER support parallelism?

**Yes!** Automatic and manual parallelism:

```maneuver
// Automatic parallelization
results = sensors.map(s -> s.read())
// Compiler parallelizes if safe

// Explicit parallelism
#[parallel]
function process_images(images: Array<Image>):
    return images.map(process_one_image)
```

---

### Can I write drivers in MANEUVER?

**Yes!** Low-level hardware access:

```maneuver
unsafe:
    justification: "Direct register access for custom sensor"
    
    // Read hardware register
    value = read_register(0x40021000)
    
    // Write hardware register
    write_register(0x40021004, 0xFF)
```

---

## 🔧 Troubleshooting

### My program won't compile

**Common issues:**

1. **Missing units**
   ```maneuver
   move forward 50  // ✗ Error: Missing unit
   move forward 50cm  // ✓ Correct
   ```

2. **Mixed coordinate frames**
   ```maneuver
   distance = point_world - point_robot  // ✗ Error
   distance = point_world.transform_to(robot) - point_robot  // ✓
   ```

3. **Type mismatch**
   ```maneuver
   angle: degrees = 45  // ✗ Error: 45 is unitless
   angle: degrees = 45°  // ✓ Correct
   ```

Run with `--verbose` to see detailed error messages.

---

### My robot isn't moving

**Checklist:**

1. ✅ Hardware connected correctly?
2. ✅ Configuration file correct?
3. ✅ Permissions (GPIO access requires sudo on Linux)?
4. ✅ Power supply adequate?
5. ✅ Simulation mode disabled?

```bash
# Test hardware
maneuver test-hardware --config robot.toml

# Run with verbose logging
maneuver run --debug my_robot.mnvr
```

---

### Performance is slow

**Common causes:**

1. **Debug mode** - Use release mode:
   ```bash
   maneuver build --release
   ```

2. **Simulation overhead** - Deploy to real hardware

3. **Unoptimized code** - See [OPTIMIZATION-GUIDE.md](./OPTIMIZATION-GUIDE.md)

4. **Profile to find bottleneck:**
   ```bash
   maneuver profile my_robot.mnvr
   ```

---

### Where are error messages?

**Check:**

1. **Console output** - Main error source
2. **Log files** - `~/.maneuver/logs/`
3. **System logs** - `journalctl` (Linux) or Console.app (macOS)

**Get better errors:**
```bash
maneuver run --verbose --debug my_robot.mnvr
```

---

## 📞 Still Have Questions?

**Can't find your answer?**

1. **Search the docs** - `maneuver docs search <query>`
2. **Ask on Discord** - [Join here](https://discord.gg/maneuver)
3. **Check the forum** - [discuss.maneuver-lang.org](https://discuss.maneuver-lang.org)
4. **File an issue** - [GitHub Issues](https://github.com/maneuver-lang/maneuver/issues)

**We're here to help! 🤖**

---

<div align="center">

**Have a question not listed here?**

[Ask on Discord](https://discord.gg/maneuver) • [Forum](https://discuss.maneuver-lang.org) • [File an Issue](https://github.com/maneuver-lang/maneuver/issues)

**Ready to start?**

[Getting Started](./GETTING-STARTED.md) • [Tutorials](./tutorials) • [Examples](./examples)

</div>
