# MANEUVER Debugging Guide

**Finding and Fixing Bugs Fast**

---

## Table of Contents

1. [Debugging Philosophy](#debugging-philosophy)
2. [Compiler Error Messages](#compiler-error-messages)
3. [Common Errors](#common-errors)
4. [Debugging Tools](#debugging-tools)
5. [Interactive Debugging](#interactive-debugging)
6. [Performance Debugging](#performance-debugging)
7. [Hardware Debugging](#hardware-debugging)
8. [Advanced Techniques](#advanced-techniques)

---

## Debugging Philosophy

### The MANEUVER Advantage

**Most bugs are caught at compile time:**
- Unit mismatches
- Frame confusion
- Type errors
- Memory safety violations
- Timing constraint violations

**Result:** Far fewer runtime bugs to debug!

### When Bugs Occur

If code compiles, bugs are typically:
1. **Logic errors** (your algorithm is wrong)
2. **Hardware issues** (sensor malfunction, actuator failure)
3. **Environmental factors** (unexpected obstacles, lighting changes)
4. **Incorrect assumptions** (preconditions not met)

---

## Compiler Error Messages

### Reading Error Messages

MANEUVER provides detailed, helpful error messages:

```
Error: Unit mismatch
  ┌─ robot.mnvr:15:20
  │
15│     distance = 5 meters + 3 seconds
  │                           ^^^^^^^^^ 
  │                           Expected: meters
  │                           Found: seconds
  │
  = help: Cannot add quantities with different units
  = note: 5 meters has unit 'length'
  = note: 3 seconds has unit 'time'
  = hint: Did you mean to multiply or divide?
```

### Error Message Structure

```
[Error Type]: [Brief description]
  ┌─ [filename]:[line]:[column]
  │
[line]│ [source code]
      │ [visual pointer to error]
      │ [Expected vs Found]
  │
  = [help message]
  = [additional context]
  = [hints for fixing]
```

---

## Common Errors

### 1. Unit Mismatch

**Error:**
```maneuver
distance = 5 meters + 3 seconds
```

**Message:**
```
Error: Cannot add 'meters' and 'seconds'
  = help: Units must match for addition/subtraction
```

**Fix:**
```maneuver
// Use consistent units
time = 3 seconds
speed = 5 m/s
distance = speed * time  // = 15 meters
```

---

### 2. Frame Confusion

**Error:**
```maneuver
point_world: Point3D in world_frame
point_camera: Point3D in camera_frame
delta = point_world - point_camera  // ✗
```

**Message:**
```
Error: Cannot operate on points in different frames
  ┌─ robot.mnvr:23:9
  │
23│ delta = point_world - point_camera
  │         ^^^^^^^^^^^ in world_frame
  │                       ^^^^^^^^^^^^^ in camera_frame
  │
  = help: Use .transform_to() to convert frames
  = example: point_camera.transform_to(world_frame)
```

**Fix:**
```maneuver
// Transform to same frame
point_camera_world = point_camera.transform_to(world_frame)
delta = point_world - point_camera_world  // ✓
```

---

### 3. Refinement Type Violation

**Error:**
```maneuver
motor_speed: m/s where 0 ≤ value ≤ 2.0 = 5.0 m/s
```

**Message:**
```
Error: Value does not satisfy refinement predicate
  ┌─ robot.mnvr:10:45
  │
10│ motor_speed: m/s where 0 ≤ value ≤ 2.0 = 5.0 m/s
  │                                           ^^^^^^^
  │                                           5.0 > 2.0
  │
  = help: Value must be between 0 and 2.0 m/s
  = note: Constraint: 0 ≤ value ≤ 2.0
  = note: Actual value: 5.0
```

**Fix:**
```maneuver
motor_speed: m/s where 0 ≤ value ≤ 2.0 = 1.5 m/s  // ✓
```

---

### 4. Deadline Violation

**Error:**
```maneuver
task slow_control:
    deadline: 10ms
    
    expensive_operation()  // Takes 50ms
```

**Message:**
```
Error: Task cannot meet deadline
  ┌─ robot.mnvr:5:5
  │
5 │     expensive_operation()
  │     ^^^^^^^^^^^^^^^^^^^^^ WCET: 50ms
  │
  = help: Deadline is 10ms, but worst-case execution time is 50ms
  = note: Consider optimizing expensive_operation()
  = note: Or increase deadline if safe to do so
```

**Fix:**
```maneuver
task slow_control:
    deadline: 100ms  // Increased deadline
    
    expensive_operation()  // ✓

// Or optimize the operation
task fast_control:
    deadline: 10ms
    
    optimized_operation()  // ✓
```

---

### 5. Ownership/Borrowing Errors

**Error:**
```maneuver
data = create_data()
process(data)      // data moved here
use_data(data)     // ✗ data already moved
```

**Message:**
```
Error: Use of moved value
  ┌─ robot.mnvr:12:10
  │
11│ process(data)
  │         ---- value moved here
12│ use_data(data)
  │          ^^^^ value used after move
  │
  = help: data was moved on line 11
  = note: Consider borrowing instead: process(&data)
```

**Fix:**
```maneuver
// Option 1: Borrow instead of move
data = create_data()
process(&data)     // Borrow, don't move
use_data(&data)    // ✓ Still accessible

// Option 2: Clone if needed
data = create_data()
process(data.clone())  // Clone for process
use_data(data)         // ✓ Original still valid
```

---

### 6. Type Inference Failure

**Error:**
```maneuver
values = []  // ✗ Can't infer type of empty array
```

**Message:**
```
Error: Cannot infer type
  ┌─ robot.mnvr:8:1
  │
8 │ values = []
  │ ^^^^^^^^^^^ Cannot determine element type
  │
  = help: Add type annotation
  = example: values: [i32; 0] = []
```

**Fix:**
```maneuver
values: [i32; 0] = []  // ✓ Explicit type
// Or
values = [i32]::new()  // ✓ Type from constructor
```

---

## Debugging Tools

### 1. Print Debugging

```maneuver
// Basic print
print("Debug: sensor value =", sensor_value)

// Debug print (includes file, line, variable name)
debug!(sensor_value)
// Output: [robot.mnvr:42] sensor_value = 3.14 meters

// Conditional debug
if DEBUG:
    print("Entering control loop")
```

### 2. Assertions

```maneuver
// Runtime assertion
assert distance > 0, "Distance must be positive"

// Compile-time assertion
static_assert N > 0, "Array size must be positive"

// Debug assertion (removed in release builds)
debug_assert speed < max_speed
```

### 3. Logging

```maneuver
import std.logging

// Different log levels
log.trace("Fine-grained debug info")
log.debug("Debug information")
log.info("Informational message")
log.warn("Warning: battery low")
log.error("Error: sensor disconnected")
log.fatal("Critical: emergency stop activated")

// Structured logging
log.info("Motor status",
    speed: current_speed,
    position: current_position,
    temperature: motor_temp
)
```

### 4. Interactive Debugger

```bash
# Start debugger
maneuver debug robot.mnvr

# Common commands
(mdb) break robot.mnvr:42    # Set breakpoint
(mdb) run                     # Run program
(mdb) next                    # Step over
(mdb) step                    # Step into
(mdb) continue               # Continue execution
(mdb) print sensor_value     # Print variable
(mdb) watch motor_speed      # Watch variable
(mdb) backtrace              # Show call stack
```

### 5. Time-Travel Debugging

```maneuver
// Record execution
maneuver record robot.mnvr

// Replay and debug
maneuver replay --debug recording.mdb

// In debugger:
(mdb) reverse-step           # Step backward
(mdb) reverse-continue       # Continue backward
(mdb) goto timestamp:100ms   # Jump to specific time
```

---

## Interactive Debugging

### Breakpoints

```maneuver
// Code breakpoint
function control_loop():
    sensor_value = read_sensor()
    breakpoint()  // Execution pauses here
    compute_control(sensor_value)

// Conditional breakpoint
function process(data):
    if data.length > 1000:
        breakpoint()  // Only break for large data
```

### REPL (Read-Eval-Print Loop)

```bash
$ maneuver repl

maneuver> distance = 5 meters
maneuver> time = 2 seconds  
maneuver> speed = distance / time
maneuver> speed
2.5 m/s

maneuver> :type speed
m/s

maneuver> :load robot.mnvr
Loaded robot.mnvr

maneuver> robot.position
Point3D(1m, 2m, 0.5m) in world_frame
```

### Inspecting State

```bash
# In debugger
(mdb) info locals          # Show local variables
(mdb) info globals         # Show global variables
(mdb) info tasks           # Show all tasks
(mdb) info frames          # Show coordinate frames

# Print with format
(mdb) print /x sensor_value     # Hexadecimal
(mdb) print /b sensor_value     # Binary
(mdb) print /t sensor_value     # Show type
```

---

## Performance Debugging

### 1. Profiling

```bash
# CPU profiling
maneuver profile --cpu robot.mnvr

# Output:
Function              | Time   | % Total | Calls
---------------------|--------|---------|-------
process_lidar()      | 45ms   | 30%     | 100
plan_path()          | 38ms   | 25%     | 50
sensor_fusion()      | 30ms   | 20%     | 100
render_visualization()| 25ms   | 17%     | 30
```

### 2. Memory Profiling

```bash
# Memory profiling
maneuver profile --memory robot.mnvr

# Output:
Allocation            | Size    | Count
---------------------|---------|-------
Point3D              | 24 bytes| 10000
PathNode             | 128 bytes| 500
SensorData           | 4KB     | 100
```

### 3. Timing Analysis

```maneuver
// Measure execution time
import std.time

start = time.now()
expensive_operation()
duration = time.now() - start
print("Operation took", duration)

// Automatic timing
#[profile]
function critical_function():
    // Function automatically timed
```

### 4. Performance Assertions

```maneuver
function fast_control():
    deadline: 10ms
    
    start = time.now()
    
    compute_control()
    
    elapsed = time.now() - start
    assert elapsed < deadline, "Missed deadline"
```

---

## Hardware Debugging

### 1. Sensor Diagnostics

```maneuver
// Test sensor connectivity
function test_sensor(sensor: Sensor) -> SensorStatus:
    try:
        value = sensor.read()
        if value in expected_range:
            return SensorStatus.OK
        else:
            return SensorStatus.OutOfRange
    catch HardwareError:
        return SensorStatus.Disconnected

// Automated diagnostics
robot mobile_bot:
    diagnostics:
        on_startup: test_all_sensors()
        periodic: every 60 seconds
```

### 2. Actuator Testing

```maneuver
// Safe actuator test
function test_motor(motor: Motor):
    requires: robot in safe_mode
    
    // Low speed test
    motor.set_speed(0.1 m/s)
    wait 1 second
    motor.stop()
    
    // Verify encoder feedback
    actual_distance = motor.encoder.read()
    expected_distance = 0.1 meters
    
    assert abs(actual_distance - expected_distance) < 0.01 meters
```

### 3. Hardware Logging

```maneuver
// Log all hardware operations
#[log_hardware]
function control_actuators():
    // All hardware access automatically logged
    motor.set_speed(speed)
    gripper.close()

// View hardware log
$ maneuver logs --hardware
[10:23:45.123] GPIO Write: PIN_MOTOR_PWM = 1500μs
[10:23:45.125] I2C Read: ADDR_0x40, REG_0x00 = 0xFF
[10:23:45.130] SPI Write: CS_0, [0x01, 0x42, 0x7F]
```

### 4. Signal Inspection

```maneuver
// Oscilloscope-like capture
import std.hardware.capture

// Capture motor PWM signal
signal = capture.pin(GPIO_MOTOR_PWM, duration: 100ms)

// Analyze signal
print("Frequency:", signal.frequency())
print("Duty cycle:", signal.duty_cycle())
print("Min/Max voltage:", signal.voltage_range())

// Plot signal
signal.plot()
```

---

## Advanced Techniques

### 1. Assertion-Guided Debugging

```maneuver
function complex_algorithm(data):
    // Invariants help narrow down bugs
    assert data.length > 0, "Input must not be empty"
    
    result = process_step1(data)
    assert result.valid(), "Step 1 produced invalid result"
    
    result = process_step2(result)
    assert result.consistent(), "Step 2 broke consistency"
    
    return result
```

### 2. Differential Debugging

```maneuver
// Compare two implementations
function test_algorithm():
    data = generate_test_data()
    
    result1 = old_algorithm(data)
    result2 = new_algorithm(data)
    
    if result1 != result2:
        print("Difference found!")
        print("Old:", result1)
        print("New:", result2)
        print("Input:", data)
        breakpoint()
```

### 3. Fuzzing

```bash
# Fuzz test your code
maneuver fuzz robot.mnvr --function process_input

# Automatically generates random inputs
# Reports crashes and assertion failures
```

### 4. Visualization

```maneuver
import std.visualization

// Visualize robot state
function visualize_state():
    viz = create_visualization()
    
    // Draw robot
    viz.draw_robot(current_position, current_orientation)
    
    // Draw sensors
    for sensor in sensors:
        viz.draw_sensor_cone(sensor)
    
    // Draw obstacles
    for obstacle in detected_obstacles:
        viz.draw_obstacle(obstacle)
    
    // Show
    viz.show()
```

### 5. State Recording

```maneuver
// Record all state for later analysis
#[record_state]
robot autonomous_vehicle:
    // All state automatically recorded
    
    sensors:
        lidar: 10 Hz
        camera: 30 Hz
    
    // Later: replay and analyze
    // $ maneuver replay recording.mdb --visualize
```

---

## Error Recovery

### 1. Graceful Degradation

```maneuver
function sensor_fusion():
    try:
        lidar_data = read_lidar()
        camera_data = read_camera()
        return fuse(lidar_data, camera_data)
    catch LidarError:
        log.warn("LIDAR failed, using camera only")
        return use_camera_only()
    catch CameraError:
        log.warn("Camera failed, using LIDAR only")
        return use_lidar_only()
    catch AllSensorsFailed:
        log.error("All sensors failed!")
        emergency_stop()
```

### 2. Retry Logic

```maneuver
function robust_read_sensor() -> Result<f64, Error>:
    max_retries = 3
    
    for attempt in 1..max_retries:
        try:
            return Ok(sensor.read())
        catch TransientError:
            log.debug("Retry", attempt, "of", max_retries)
            wait 100ms
    
    return Err(Error.SensorFailed)
```

### 3. Watchdog Recovery

```maneuver
safety_system watchdog:
    monitor control_loop every 10ms
    timeout: 50ms
    
    on_timeout:
        log.error("Control loop timeout!")
        
        // Attempt recovery
        try:
            restart_control_loop()
        catch:
            // Recovery failed, emergency stop
            emergency_stop()
            alert_operator()
```

---

## Debugging Checklist

Before asking for help, check:

### Compile-Time Errors
- [ ] Read the full error message
- [ ] Check line/column indicated
- [ ] Read the help/hint text
- [ ] Try the suggested fix

### Runtime Errors
- [ ] Can you reproduce it consistently?
- [ ] What's the minimal example that triggers it?
- [ ] Have you added debug prints?
- [ ] What's the value of relevant variables?

### Performance Issues
- [ ] Have you profiled the code?
- [ ] What's the bottleneck?
- [ ] Are you hitting deadlines?
- [ ] Is it CPU, memory, or I/O bound?

### Hardware Issues
- [ ] Does it work in simulation?
- [ ] Have you tested sensors individually?
- [ ] Are connections secure?
- [ ] Is power supply adequate?

---

## Getting Help

### 1. Minimal Reproducible Example

```maneuver
// Bad: "My robot doesn't work"
// Good: Minimal example

robot simple_bot:
    sensor test_sensor:
        type: ultrasonic
    
    task main:
        value = read test_sensor
        print(value)  // Always prints 0

// Expected: sensor reading 0-400cm
// Actual: always 0
// Hardware: Arduino Uno, HC-SR04 sensor
```

### 2. Include Context

```markdown
## Environment
- MANEUVER version: 1.0.0
- OS: Ubuntu 22.04
- Hardware: Raspberry Pi 4

## Steps to Reproduce
1. Run: maneuver run robot.mnvr
2. Observe output
3. Expected X, got Y

## Error Output
[paste exact error message]
```

### 3. Where to Ask

- **Discord**: Quick questions
- **Forum**: Detailed discussions
- **GitHub Issues**: Bug reports
- **Stack Overflow**: Tagged `maneuver-lang`

---

## Conclusion

**Debugging in MANEUVER is easier because:**

✅ Most bugs caught at compile time  
✅ Clear, helpful error messages  
✅ Powerful debugging tools  
✅ Type system guides you to the fix  

**Remember:** If it compiles, it's probably correct. If it's not, the bug is likely in your logic, not your types.

**Happy debugging!** 🐛🔍
