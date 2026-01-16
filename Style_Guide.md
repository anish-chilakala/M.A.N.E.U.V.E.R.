# MANEUVER Style Guide

**Writing Clean, Idiomatic MANEUVER Code**

---

## Core Principles

1. **Clarity over Cleverness** - Code is read more than written
2. **Types Document Intent** - Use the type system to express constraints
3. **Natural Language Flow** - Code should read like precise English
4. **Safety by Default** - Unsafe code should be rare and well-justified

---

## Naming Conventions

### Variables & Functions

```maneuver
// ✓ snake_case for variables and functions
sensor_value: f64
current_position: Point3D
max_speed: m/s

function calculate_distance(p1: Point3D, p2: Point3D) -> meters:
    return ||p1 - p2||

// ✗ Avoid
SensorValue: f64           // Wrong: PascalCase
currentPosition: Point3D   // Wrong: camelCase
MAX_SPEED: m/s            // Wrong: SCREAMING_SNAKE_CASE (unless constant)
```

### Constants

```maneuver
// ✓ SCREAMING_SNAKE_CASE for constants
const MAX_SPEED: m/s = 2.0 m/s
const PI: f64 = 3.14159265359
const ROBOT_WIDTH: meters = 0.5 meters

// ✗ Avoid
const max_speed = 2.0 m/s  // Wrong: lowercase
```

### Types & Enums

```maneuver
// ✓ PascalCase for types and enum variants
struct Point3D {
    x: meters
    y: meters
    z: meters
}

enum RobotState {
    Idle,
    Moving,
    Stopped,
    Error
}

// ✗ Avoid
struct point_3d { ... }     // Wrong: snake_case
enum RobotState {
    idle,                   // Wrong: lowercase
    MOVING,                 // Wrong: SCREAMING
}
```

### Acronyms

```maneuver
// ✓ Treat as single word
lidar_sensor: LidarSensor
gps_data: GpsData
pid_controller: PidController

// ✗ Avoid
lIDAR_sensor: LIDARSensor   // Wrong: all caps in middle
GPS_Data: GPSData           // Wrong: all caps at start
```

---

## Formatting

### Indentation

```maneuver
// ✓ 4 spaces (not tabs)
function example():
    if condition:
        do_something()
        do_more()
    else:
        do_alternative()
```

### Line Length

```maneuver
// ✓ Maximum 100 characters per line
function long_function_name(
    parameter1: Type1,
    parameter2: Type2,
    parameter3: Type3
) -> ReturnType:
    // Implementation

// ✗ Avoid lines over 100 chars
function long_function_name(parameter1: Type1, parameter2: Type2, parameter3: Type3) -> ReturnType:
```

### Blank Lines

```maneuver
// ✓ Use blank lines to separate logical sections
function complex_calculation():
    // Step 1: Read sensors
    lidar_data = read_lidar()
    camera_data = read_camera()
    
    // Step 2: Process data
    filtered = filter_noise(lidar_data)
    detected = detect_objects(camera_data)
    
    // Step 3: Fuse results
    return sensor_fusion(filtered, detected)
```

### Trailing Commas

```maneuver
// ✓ Use trailing commas in multi-line lists
robot autonomous_vehicle:
    sensors:
        lidar: Velodyne_VLS128,
        camera: ZED_2i,
        imu: BMI088,     // ← trailing comma
```

---

## Type Annotations

### Explicit vs Inferred

```maneuver
// ✓ Explicit for function signatures and struct fields
function calculate_speed(distance: meters, time: seconds) -> m/s:
    return distance / time

struct Robot {
    name: string
    position: Point3D
    speed: m/s
}

// ✓ Inferred for local variables (when obvious)
distance := 10 meters
speed := distance / time

// ✗ Avoid unnecessary annotations for locals
distance: meters = 10 meters  // Redundant
```

### Physical Units

```maneuver
// ✓ Always use explicit units
distance: meters = 5.0 meters
speed: m/s = 2.5 m/s
angle: degrees = 90 degrees

// ✗ Avoid bare numbers for physical quantities
distance: f64 = 5.0  // What unit?
speed = 2.5          // m/s? mph? km/h?
```

### Coordinate Frames

```maneuver
// ✓ Always label frames explicitly
target: Point3D in world_frame
robot_pos: Point3D in robot_base
sensor_reading: Point3D in camera_frame

// ✗ Avoid unlabeled spatial types
target: Point3D  // Which frame?
```

---

## Comments

### When to Comment

```maneuver
// ✓ Explain WHY, not WHAT
// We use 10Hz because sensor fusion needs minimum 100ms intervals
const CONTROL_FREQUENCY: Hz = 10 Hz

// Safety margin added after testing showed 2.0 m/s caused instability
const MAX_SAFE_SPEED: m/s = 1.5 m/s

// ✗ Don't comment the obvious
distance = 5 meters  // Set distance to 5 meters (redundant!)
```

### Documentation Comments

```maneuver
/// Calculate the shortest path between two points
///
/// Uses A* algorithm with euclidean distance heuristic.
/// Guaranteed to find optimal path if one exists.
///
/// # Arguments
/// * `start` - Starting position in world frame
/// * `goal` - Goal position in world frame
///
/// # Returns
/// Path from start to goal, or None if no path exists
///
/// # Example
/// ```
/// start = Point3D(0m, 0m, 0m)
/// goal = Point3D(5m, 3m, 0m)
/// path = plan_path(start, goal)
/// ```
function plan_path(
    start: Point3D in world_frame,
    goal: Point3D in world_frame
) -> Option<Path>:
    // Implementation
```

### TODO Comments

```maneuver
// ✓ Include context and tracking
// TODO(username): Optimize this loop - currently O(n²)
// TODO: Add obstacle avoidance (Issue #42)
// FIXME: Race condition possible here

// ✗ Avoid vague TODOs
// TODO: Fix this
// TODO: Make better
```

---

## Control Flow

### If Statements

```maneuver
// ✓ Use natural language
if distance < threshold:
    stop_robot()
    turn_away()

// ✓ Early returns for guards
function process_data(data: Data) -> Result:
    if not data.valid():
        return Err(InvalidData)
    
    // Main logic here
    return Ok(result)

// ✗ Avoid deep nesting
if condition1:
    if condition2:
        if condition3:
            if condition4:
                do_something()  // Too deep!
```

### Pattern Matching

```maneuver
// ✓ Prefer match over if-else chains
state = get_robot_state()
match state:
    Idle:
        wait_for_command()
    Moving(speed):
        maintain_speed(speed)
    Stopped:
        await_restart()
    Error(msg):
        handle_error(msg)

// ✗ Avoid if-else chains when match is clearer
if state == Idle:
    wait_for_command()
else if state == Moving:
    // ...
```

### Loops

```maneuver
// ✓ Use descriptive loop variables
for point in point_cloud:
    process(point)

for (index, value) in array.enumerate():
    print(index, value)

// ✗ Avoid single-letter names (except i, j for indices)
for x in data:  // What is x?
    process(x)

// ✓ OK for mathematical contexts
for i in 0..N:
    for j in 0..M:
        matrix[i][j] = compute(i, j)
```

---

## Functions

### Function Length

```maneuver
// ✓ Keep functions focused (< 50 lines)
function calculate_speed(distance: meters, time: seconds) -> m/s:
    return distance / time

// ✗ If function is too long, split it
function do_everything():  // 200 lines...
    // Too long! Split into smaller functions
```

### Parameters

```maneuver
// ✓ Named parameters for clarity
move_robot(
    distance: 5 meters,
    speed: 1.5 m/s,
    acceleration: 0.5 m/s²
)

// ✓ Limit parameters (< 5)
function plan_motion(
    start: Point3D,
    goal: Point3D,
    max_speed: m/s,
    obstacles: [Obstacle]
) -> Path:
    // Implementation

// ✗ Too many parameters - use struct
function plan_motion(
    start_x, start_y, start_z,
    goal_x, goal_y, goal_z,
    max_speed, max_accel, max_jerk,
    obstacle1, obstacle2, obstacle3
):  // Too many! Use MotionParams struct
```

### Return Types

```maneuver
// ✓ Use Result for fallible operations
function read_sensor() -> Result<f64, SensorError>:
    try:
        return Ok(hardware.read())
    catch HardwareError as e:
        return Err(SensorError.Disconnected)

// ✓ Use Option for nullable values
function find_nearest_obstacle() -> Option<Obstacle>:
    if obstacles.empty():
        return None
    return Some(obstacles.nearest())

// ✗ Avoid returning magic values
function read_sensor() -> f64:
    return -999.0  // Error indicator - use Result instead!
```

---

## Error Handling

### Explicit Over Implicit

```maneuver
// ✓ Handle errors explicitly
result = try_operation()
match result:
    Ok(value):
        process(value)
    Err(error):
        handle_error(error)

// ✗ Avoid ignoring errors
result = try_operation()  // What if it fails?
value = result.unwrap()   // Can panic!
```

### Error Context

```maneuver
// ✓ Provide context in error messages
if not sensor.connected():
    return Err("LIDAR sensor disconnected - check cable")

// ✗ Vague error messages
if not sensor.connected():
    return Err("Error")  // Not helpful!
```

---

## Safety & Unsafe Code

### Minimize Unsafe

```maneuver
// ✓ Keep unsafe blocks small
function direct_hardware_access():
    // Safe preparation
    validate_input()
    
    // Minimal unsafe block
    unsafe:
        justification: "Direct register access required for DMA"
        write_register(DMA_ADDR, ptr)
    
    // Safe cleanup
    verify_operation()

// ✗ Large unsafe blocks
unsafe:  // Too much unsafe!
    // 100 lines of unsafe code
```

### Document Unsafe

```maneuver
// ✓ Always justify unsafe code
unsafe:
    justification: "Performance-critical: eliminates bounds check in verified safe loop"
    audit_id: "UNSAFE-2026-001"
    reviewer: "security-team"
    
    // Unsafe operation
```

---

## Testing

### Test Organization

```maneuver
// ✓ Organize tests near the code
function calculate_distance(p1: Point3D, p2: Point3D) -> meters:
    return ||p1 - p2||

test calculate_distance_basic:
    p1 = Point3D(0m, 0m, 0m)
    p2 = Point3D(3m, 4m, 0m)
    assert calculate_distance(p1, p2) == 5 meters

test calculate_distance_zero:
    p1 = Point3D(0m, 0m, 0m)
    assert calculate_distance(p1, p1) == 0 meters
```

### Test Naming

```maneuver
// ✓ Descriptive test names
test sensor_returns_valid_range:
    // Test that sensor returns values in expected range

test motor_stops_on_emergency:
    // Test emergency stop functionality

// ✗ Vague names
test test1:
    // What does this test?
```

---

## Performance

### Premature Optimization

```maneuver
// ✓ Write clear code first
function process_data(data: [f64; N]) -> [f64; N]:
    result = []
    for value in data:
        result.push(value * 2.0)
    return result

// ✗ Don't optimize prematurely
function process_data(data: [f64; N]) -> [f64; N]:
    // Unrolled loop, SIMD intrinsics, cache-aligned...
    // Only after profiling shows this is bottleneck!
```

### Compiler Hints

```maneuver
// ✓ Use annotations when performance matters
#[inline]
function critical_inner_loop():
    // Hot path

#[optimize(aggressive)]
function performance_critical():
    // Compiler applies all optimizations
```

---

## Module Organization

### File Structure

```
robot_project/
├── Maneuver.toml
├── src/
│   ├── main.mnvr          # Entry point
│   ├── robot.mnvr         # Robot definition
│   ├── sensors/
│   │   ├── mod.mnvr       # Module declaration
│   │   ├── lidar.mnvr
│   │   └── camera.mnvr
│   ├── control/
│   │   ├── mod.mnvr
│   │   ├── pid.mnvr
│   │   └── mpc.mnvr
│   └── utils/
│       ├── mod.mnvr
│       └── math.mnvr
├── tests/
│   ├── integration_test.mnvr
│   └── hardware_test.mnvr
└── docs/
    └── README.md
```

### Imports

```maneuver
// ✓ Group and order imports
// 1. Standard library
import std.math
import std.collections

// 2. External dependencies
import robotics.kinematics
import vision.opencv

// 3. Local modules
import sensors.lidar
import control.pid

// ✗ Avoid
import control.pid
import std.math
import vision.opencv  // Random order
```

---

## Anti-Patterns

### 1. Stringly Typed

```maneuver
// ✗ Using strings for everything
function process_command(cmd: string):
    if cmd == "move_forward":
        // ...
    else if cmd == "turn_left":
        // ...

// ✓ Use enums
enum Command {
    MoveForward(meters),
    TurnLeft(degrees),
    Stop
}

function process_command(cmd: Command):
    match cmd:
        MoveForward(dist): move(dist)
        TurnLeft(angle): turn(angle)
        Stop: stop()
```

### 2. Magic Numbers

```maneuver
// ✗ Magic numbers
if speed > 2.5:  // What is 2.5?
    slow_down()

// ✓ Named constants
const MAX_SAFE_SPEED: m/s = 2.5 m/s

if speed > MAX_SAFE_SPEED:
    slow_down()
```

### 3. Boolean Parameters

```maneuver
// ✗ Boolean flags
move_robot(5 meters, true, false)  // What do these mean?

// ✓ Named parameters or enums
move_robot(
    distance: 5 meters,
    smooth: true,
    wait_for_completion: false
)
```

---

## Tooling

### Auto-Formatting

```bash
# Format code automatically
maneuver fmt

# Check formatting (CI)
maneuver fmt --check
```

### Linting

```bash
# Run linter
maneuver lint

# Fix auto-fixable issues
maneuver lint --fix
```

### Editor Integration

Configure your editor:
- Auto-format on save
- Show compiler errors inline
- Auto-complete with type information

---

## Summary

**Golden Rules:**

1. ✅ **Be Explicit** - Types, units, frames
2. ✅ **Be Clear** - Natural language over cleverness
3. ✅ **Be Safe** - Use the type system
4. ✅ **Be Consistent** - Follow conventions
5. ✅ **Be Helpful** - Write for future readers

**When in doubt:** Clarity > Brevity

**Remember:** The compiler is your friend. Let it help you write better code.
