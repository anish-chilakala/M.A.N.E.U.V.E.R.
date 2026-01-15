# MANEUVER API Reference

> **Complete Standard Library Documentation**

[![Version](https://img.shields.io/badge/Version-1.0.0-blue)]()
[![Modules](https://img.shields.io/badge/Modules-15-green)]()
[![Functions](https://img.shields.io/badge/Functions-200+-orange)]()

---

## 📚 Table of Contents

1. [Core](#core)
2. [Motion](#motion)
3. [Sensors](#sensors)
4. [Actuators](#actuators)
5. [Math](#math)
6. [Geometry](#geometry)
7. [Kinematics](#kinematics)
8. [Control](#control)
9. [Planning](#planning)
10. [Perception](#perception)
11. [Communication](#communication)
12. [Time](#time)
13. [I/O](#io)
14. [Vision](#vision)
15. [AI](#ai)

---

## 🎯 Core

### Basic Types

#### `Point2D<Unit>`
Represents a 2D point with physical units.

```maneuver
// Declaration
point: Point2D<meters>

// Construction
p1: Point2D<meters> = (x: 1m, y: 2m)
p2 = Point2D(x: 1m, y: 2m)

// Methods
distance = p1.distance_to(p2)  // Returns meters
angle = p1.angle_to(p2)        // Returns radians
```

#### `Point3D<Unit>`
Represents a 3D point with physical units.

```maneuver
// Declaration
point: Point3D<meters> in world_frame

// Construction
p = Point3D(x: 1m, y: 2m, z: 0.5m) in robot_base

// Methods
p.distance_to(other: Point3D) -> meters
p.magnitude() -> meters
p.normalize() -> Point3D<dimensionless>
p.transform_to(frame: Frame) -> Point3D
```

#### `Vector2D<Unit>`
2D vector with direction and magnitude.

```maneuver
v: Vector2D<meters> = (x: 1m, y: 0m)

// Operations
v1 + v2          // Addition
v1 - v2          // Subtraction
v * 2.0          // Scalar multiplication
v1.dot(v2)       // Dot product
v.magnitude()    // Length
v.normalize()    // Unit vector
v.angle()        // Angle from x-axis
```

#### `Vector3D<Unit>`
3D vector with direction and magnitude.

```maneuver
v: Vector3D<meters> = (x: 1m, y: 0m, z: 0.5m)

// Operations
v1.cross(v2)     // Cross product
v.rotate(axis, angle)  // Rotation
```

---

## 🚗 Motion

### Robot Movement

#### `move`
Move the robot in a direction.

```maneuver
// Basic movement
move forward 50cm
move backward 30cm

// With speed
move forward 1m at 0.5 m/s

// To specific point
move to position (x: 1m, y: 2m, z: 0m)

// With constraints
move to target:
    speed: 0.3 m/s
    acceleration: 1 m/s²
    avoid_obstacles: true
```

**Parameters:**
- `direction`: forward, backward, left, right, up, down
- `distance`: Length with unit (cm, m, etc.)
- `speed`: Velocity (m/s, cm/s, etc.)
- `acceleration`: Acceleration (m/s², etc.)

**Returns:** None

**Throws:** 
- `OutOfReachError` if position unreachable
- `CollisionError` if obstacles detected

---

#### `turn`
Rotate the robot.

```maneuver
// Basic rotation
turn left 90 degrees
turn right 45°

// Absolute heading
turn to heading 180°

// With speed
turn left 90° at 30 degrees/second
```

**Parameters:**
- `direction`: left, right, clockwise, counterclockwise
- `angle`: Angle with unit (degrees, radians, etc.)
- `speed`: Angular velocity (deg/s, rad/s)

---

#### `stop`
Stop all motion immediately.

```maneuver
stop              // Emergency stop
stop gently       // Gradual deceleration
stop after 2s     // Stop after time
```

---

### Trajectories

#### `follow_path`
Follow a pre-defined path.

```maneuver
path: Path = [
    (x: 0m, y: 0m),
    (x: 1m, y: 0m),
    (x: 1m, y: 1m),
]

follow_path(path):
    speed: 0.5 m/s
    smoothing: cubic_spline
```

---

#### `plan_path`
Plan a collision-free path.

```maneuver
start: Point3D = current_position
goal: Point3D = (x: 2m, y: 3m, z: 0m)

path = plan_path(start, goal):
    algorithm: a_star  // or rrt, prm
    obstacles: obstacle_map
    max_iterations: 1000

follow_path(path)
```

---

## 📡 Sensors

### Sensor Declaration

```maneuver
sensor <name>:
    type: <sensor_type>
    location: <position>
    range: <min> to <max>
    frequency: <Hz>
```

### Sensor Types

#### Ultrasonic
Measures distance using sound waves.

```maneuver
sensor distance_sensor:
    type: ultrasonic
    location: front
    range: 2cm to 400cm
    frequency: 10 Hz

// Read value
distance = distance_sensor.read()  // Returns meters
```

#### LIDAR
Laser-based distance sensing (3D point cloud).

```maneuver
sensor lidar:
    type: lidar
    location: roof_center
    range: 0.5m to 100m
    frequency: 10 Hz
    resolution: 0.1 degrees

// Read point cloud
cloud: PointCloud = lidar.read()

// Filter and process
filtered = cloud.filter(z > 0 and z < 2)
ground = cloud.segment_ground()
objects = cloud.detect_objects()
```

#### Camera
Visual sensor (RGB or depth).

```maneuver
sensor camera:
    type: camera_rgb
    resolution: (1920, 1080)
    framerate: 30 Hz
    field_of_view: 110 degrees

// Capture image
image: Image = camera.capture()

// Process
grayscale = image.to_grayscale()
edges = image.detect_edges()
objects = image.detect_objects()
```

#### IMU
Inertial Measurement Unit (accelerometer + gyroscope).

```maneuver
sensor imu:
    type: imu_6dof
    location: center_of_mass
    frequency: 400 Hz

// Read data
reading: IMUReading = imu.read()
acceleration: Vector3D<m/s²> = reading.acceleration
angular_velocity: Vector3D<rad/s> = reading.angular_velocity
```

#### GPS
Global positioning system.

```maneuver
sensor gps:
    type: gps_rtk
    frequency: 10 Hz

// Read position
position: Point3D<meters> in WGS84 = gps.read()
accuracy: meters = gps.accuracy()
```

---

## ⚙️ Actuators

### Motors

#### DC Motor
```maneuver
motor left_motor:
    type: dc_motor
    pin: GPIO_17
    max_voltage: 12V

// Control
left_motor.set_speed(0.5)  // 0.0 to 1.0
left_motor.set_voltage(6V)
left_motor.stop()
```

#### Servo Motor
```maneuver
motor shoulder:
    type: servo
    pin: PWM_1
    range: 0° to 180°

// Control
shoulder.set_angle(90°)
shoulder.move_to(45° over 2 seconds)
```

#### Stepper Motor
```maneuver
motor stepper:
    type: stepper
    step_angle: 1.8°
    steps_per_rev: 200

// Control
stepper.step(100)  // Move 100 steps
stepper.rotate(45°)
stepper.set_speed(200 steps/second)
```

### Grippers

```maneuver
gripper hand:
    type: parallel
    max_force: 50N
    max_opening: 10cm

// Control
hand.open()
hand.close()
hand.set_opening(5cm)
hand.close_with_force(10N)
```

---

## 🔢 Math

### Constants

```maneuver
π = 3.14159265359
e = 2.71828182846
φ = 1.61803398875  // Golden ratio
c = 299792458 m/s  // Speed of light
g = 9.81 m/s²      // Gravity
```

### Basic Functions

```maneuver
// Trigonometry
sin(45 degrees)    -> float
cos(π/4)          -> float
tan(angle)        -> float
asin(x)           -> radians
acos(x)           -> radians
atan2(y, x)       -> radians

// Power & Exponential
sqrt(16)          -> 4.0
pow(2, 3)         -> 8.0
exp(1)            -> e
log(e)            -> 1.0
log10(100)        -> 2.0

// Rounding
abs(-5)           -> 5
ceil(3.2)         -> 4
floor(3.8)        -> 3
round(3.5)        -> 4

// Min/Max
min(1, 2, 3)      -> 1
max(1, 2, 3)      -> 3
clamp(x, 0, 10)   -> x bounded to [0, 10]
```

### Linear Algebra

```maneuver
// Vectors
v1: Vector3D = (1, 2, 3)
v2: Vector3D = (4, 5, 6)

v1 + v2           // Addition
v1 - v2           // Subtraction
v1 * 2.0          // Scalar multiply
v1.dot(v2)        // Dot product: 32
v1.cross(v2)      // Cross product
v1.magnitude()    // Length: 3.74
v1.normalize()    // Unit vector

// Matrices
m: Matrix3x3 = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
]

m * v             // Matrix-vector multiply
m1 * m2           // Matrix multiply
m.transpose()     // Transpose
m.inverse()       // Inverse
m.determinant()   // Determinant
```

---

## 📐 Geometry

### Transforms

#### Translation
```maneuver
// Create translation
t: Transform = Translation(x: 1m, y: 2m, z: 0m)

// Apply to point
p: Point3D = (0m, 0m, 0m)
p_transformed = t * p  // (1m, 2m, 0m)
```

#### Rotation
```maneuver
// Rotation around axis
r: Rotation = Rotation.from_axis_angle(
    axis: (0, 0, 1),
    angle: 90°
)

// Rotation from Euler angles
r: Rotation = Rotation.from_euler(
    roll: 0°,
    pitch: 0°,
    yaw: 90°
)

// Rotation from quaternion
q: Quaternion = (w: 1, x: 0, y: 0, z: 0)
r: Rotation = Rotation.from_quaternion(q)
```

#### Complete Transform
```maneuver
// Combine rotation and translation
transform: Transform = Transform(
    rotation: Rotation.from_euler(0°, 0°, 90°),
    translation: (1m, 0m, 0m)
)

// Apply
point_transformed = transform * point

// Compose transforms
t_combined = t1 * t2 * t3

// Inverse
t_inv = transform.inverse()
```

### Coordinate Frames

```maneuver
// Declare frames
frame world
frame robot_base
frame camera

// Define relationships
robot_base.set_parent(world):
    transform: Transform(...)

camera.set_parent(robot_base):
    transform: Transform(...)

// Convert between frames
point_world: Point3D in world = (1m, 2m, 0m)
point_camera = point_world.transform_to(camera)
```

---

## 🦾 Kinematics

### Forward Kinematics

```maneuver
// Define robot
robot ur5:
    type: serial_manipulator
    joints: 6
    dh_parameters: [
        {a: 0, alpha: π/2, d: 0.089159, theta: θ1},
        {a: -0.425, alpha: 0, d: 0, theta: θ2},
        {a: -0.39225, alpha: 0, d: 0, theta: θ3},
        {a: 0, alpha: π/2, d: 0.10915, theta: θ4},
        {a: 0, alpha: -π/2, d: 0.09465, theta: θ5},
        {a: 0, alpha: 0, d: 0.0823, theta: θ6},
    ]

// Compute forward kinematics
joint_angles: Array<degrees> = [0°, -90°, 90°, 0°, 0°, 0°]
end_effector_pose: Pose = ur5.forward_kinematics(joint_angles)
```

### Inverse Kinematics

```maneuver
// Desired end-effector pose
target: Pose = Pose(
    position: (0.4m, 0.2m, 0.3m),
    orientation: Rotation.from_euler(0°, 90°, 0°)
)

// Compute inverse kinematics
result = ur5.inverse_kinematics(target)

match result:
    Success(joint_angles) ->
        move_joints_to(joint_angles)
    
    NoSolution ->
        say "Target unreachable"
    
    MultipleSolutions(solutions) ->
        // Pick closest to current configuration
        best = solutions.closest_to(current_angles)
        move_joints_to(best)
```

### Jacobian

```maneuver
// Compute Jacobian matrix
J: Matrix<6, 6> = ur5.jacobian(current_joint_angles)

// Use for velocity control
joint_velocities = J.pseudoinverse() * desired_end_effector_velocity
```

---

## 🎮 Control

### PID Controller

```maneuver
// Create controller
pid: PIDController = PIDController(
    kp: 1.0,
    ki: 0.1,
    kd: 0.05,
    output_limits: (-100, 100)
)

// Control loop
loop:
    error = setpoint - current_position
    output = pid.compute(error, dt)
    apply_control(output)
    wait dt
```

### Model Predictive Control

```maneuver
// Create MPC controller
mpc: MPCController = MPCController(
    model: robot_dynamics,
    horizon: 10,
    dt: 0.1,
    cost_function: quadratic_cost
)

// Optimize
optimal_control = mpc.solve(
    current_state,
    desired_trajectory
)
```

### Kalman Filter

```maneuver
// Create Extended Kalman Filter
ekf: KalmanFilter = KalmanFilter(
    state: [position, velocity],
    measurement: [gps_position],
    process_noise: 0.01,
    measurement_noise: 0.1
)

// Predict step
ekf.predict(dt)

// Update step
measurement = gps.read()
ekf.update(measurement)

// Get estimate
estimated_state = ekf.state
uncertainty = ekf.covariance
```

---

## 🗺️ Planning

### A* Path Planning

```maneuver
// Create grid map
map: GridMap = GridMap(
    width: 100,
    height: 100,
    resolution: 0.1m
)

// Add obstacles
map.add_obstacle(rect(x: 2m, y: 3m, width: 1m, height: 1m))

// Plan path
path = a_star(
    start: (0m, 0m),
    goal: (10m, 10m),
    map: map,
    heuristic: euclidean_distance
)
```

### RRT Planning

```maneuver
// Rapidly-Exploring Random Tree
path = rrt(
    start: current_position,
    goal: target_position,
    obstacles: obstacle_list,
    max_iterations: 10000,
    step_size: 0.1m
)
```

### Trajectory Optimization

```maneuver
// Generate smooth trajectory
trajectory = optimize_trajectory(
    waypoints: [p1, p2, p3, p4],
    constraints: {
        max_velocity: 1.0 m/s,
        max_acceleration: 2.0 m/s²,
        max_jerk: 5.0 m/s³
    },
    objective: minimize_time
)

// Follow trajectory
follow_trajectory(trajectory)
```

---

## 👁️ Perception

### Object Detection

```maneuver
// Detect objects in image
objects = image.detect_objects():
    classes: [person, car, dog, cat]
    confidence_threshold: 0.7
    nms_threshold: 0.5

// Process results
for obj in objects:
    say "Found ${obj.class} at ${obj.bounding_box}"
```

### Point Cloud Processing

```maneuver
// Load point cloud
cloud: PointCloud = lidar.read()

// Preprocess
cloud = cloud
    .remove_outliers()
    .downsample(voxel_size: 0.05m)
    .remove_ground_plane()

// Segment into clusters
clusters = cloud.cluster():
    method: euclidean
    tolerance: 0.1m
    min_points: 100

// Classify
for cluster in clusters:
    if cluster.size > 1000:
        say "Large object detected"
```

### SLAM (Simultaneous Localization and Mapping)

```maneuver
// Initialize SLAM
slam: SLAM = SLAM(
    method: orb_slam3,
    sensors: [camera, imu]
)

// Update loop
loop:
    image = camera.capture()
    imu_data = imu.read()
    
    slam.update(image, imu_data)
    
    current_pose = slam.get_pose()
    map = slam.get_map()
```

---

## 📞 Communication

### Inter-Robot Communication

```maneuver
// Create message channel
channel robot_network:
    protocol: wifi
    port: 5000

// Broadcast message
broadcast to robot_network:
    type: "position_update"
    position: current_position
    timestamp: now()

// Receive messages
on message from robot_network:
    match message.type:
        "position_update" ->
            other_position = message.position
            update_map(other_position)
```

### ROS Integration

```maneuver
// Publish to ROS topic
publish to "/cmd_vel":
    linear: (x: 0.5, y: 0, z: 0)
    angular: (x: 0, y: 0, z: 0.1)

// Subscribe to ROS topic
subscribe to "/scan":
    on_message: (scan_data) ->
        process_laser_scan(scan_data)
```

---

## ⏱️ Time

### Time Functions

```maneuver
// Current time
now() -> Timestamp

// Wait
wait 1 second
wait 500ms

// Timer
start_time = now()
// ... do work ...
elapsed = now() - start_time

// Rate limiting
rate: Rate = Rate(10 Hz)  // 10 times per second

loop:
    do_work()
    rate.sleep()  // Maintains 10Hz
```

### Scheduling

```maneuver
// Schedule periodic task
task sensor_read:
    frequency: 100 Hz
    deadline: 9ms
    
    loop:
        data = sensor.read()
        process(data)
```

---

## 📁 I/O

### File Operations

```maneuver
// Read file
data = read_file("config.yaml")

// Write file
write_file("log.txt", log_data)

// CSV
csv_data = read_csv("sensors.csv")

// JSON
config = read_json("robot_config.json")
```

### Logging

```maneuver
// Log levels
log.debug("Debug message")
log.info("Info message")
log.warn("Warning message")
log.error("Error message")

// With context
log.info("Position: ${current_position}")
```

---

## 🎥 Vision

### Image Processing

```maneuver
// Load image
img: Image = load_image("photo.jpg")

// Basic operations
gray = img.to_grayscale()
resized = img.resize(width: 640, height: 480)
cropped = img.crop(x: 100, y: 100, width: 200, height: 200)

// Filters
blurred = img.gaussian_blur(sigma: 5)
edges = img.canny_edges(low: 50, high: 150)
sharpened = img.sharpen()

// Color space
hsv = img.to_hsv()
lab = img.to_lab()

// Thresholding
binary = img.threshold(value: 128)
adaptive = img.adaptive_threshold()
```

### Feature Detection

```maneuver
// Detect features
keypoints = img.detect_keypoints():
    method: orb
    num_features: 1000

// Match features
matches = match_features(keypoints1, keypoints2)

// Estimate pose
pose = estimate_pose_from_features(matches)
```

---

## 🤖 AI

### Neural Networks

```maneuver
// Load pre-trained model
model = load_model("yolov8.onnx")

// Inference
image = camera.capture()
predictions = model.predict(image)

// Process results
for pred in predictions:
    if pred.confidence > 0.8:
        say "Detected ${pred.class}"
```

### Reinforcement Learning

```maneuver
// Load RL policy
policy = load_policy("trained_policy.pt")

// Use in control loop
loop:
    state = get_state()
    action = policy.select_action(state)
    execute_action(action)
```

---

## 🔧 Utility Functions

### Arrays

```maneuver
arr: Array<int> = [1, 2, 3, 4, 5]

// Operations
arr.length        // 5
arr[0]           // 1
arr.append(6)
arr.pop()
arr.map(x -> x * 2)
arr.filter(x -> x > 2)
arr.reduce((a, b) -> a + b)
arr.sort()
arr.reverse()
```

### Strings

```maneuver
s: string = "Hello, World!"

s.length         // 13
s.to_upper()     // "HELLO, WORLD!"
s.to_lower()     // "hello, world!"
s.contains("World")  // true
s.split(",")     // ["Hello", " World!"]
s.replace("World", "MANEUVER")
```

---

<div align="center">

**Complete API documentation with examples for every function**

[Getting Started](./GETTING-STARTED.md) • [Tutorials](./tutorials) • [Examples](./examples)

</div>
