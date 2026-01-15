# MANEUVER Optimization Guide

> **How to Write Fast Robot Code: Best Practices and Performance Tips**

[![Performance](https://img.shields.io/badge/Target-10--50x_faster-brightgreen)]()
[![Level](https://img.shields.io/badge/Level-Beginner_to_Expert-blue)]()

---

## 🎯 Philosophy

**MANEUVER is fast by default**, but understanding how the compiler works helps you write even faster code.

**Core Principles:**
1. **Write clear code first** - Compiler optimizes for you
2. **Trust the compiler** - Don't write "clever" code
3. **Measure, don't guess** - Profile before optimizing
4. **Let the type system help** - More constraints = better optimization

---

## 📊 Performance Levels

### Level 0: Default (Fast Already)
```maneuver
// This is already optimized!
robot arm:
    move to position(x: 10cm, y: 20cm, z: 15cm)
```
**No manual optimization needed. Compiler handles it.**

### Level 1: Type Annotations (Faster)
```maneuver
// Give compiler more information
robot arm:
    target: Point3D<cm> in world_frame where in_workspace
    move to target
```
**Compiler can eliminate runtime checks.**

### Level 2: Compile-Time Computation (Fastest)
```maneuver
// Pre-compute at compile time
const lookup_table = compile_time:
    [compute_trajectory(i) for i in 0..1000]

// Runtime: O(1) lookup instead of O(n) computation
trajectory = lookup_table[index]
```
**Zero runtime cost.**

---

## 🚀 Optimization Techniques

### 1. Let Compiler Do the Work

#### ❌ Bad: Manual Optimization
```maneuver
// Don't do this!
function process_points(points: Array<Point3D>) -> Array<Point3D>:
    result: Array<Point3D> = []
    
    // Manually trying to optimize
    if gpu_available():
        result = process_on_gpu(points)
    else:
        for point in points:
            result.append(transform(point))
    
    return result
```

#### ✅ Good: Let Compiler Decide
```maneuver
// Compiler automatically uses GPU when beneficial
function process_points(points: Array<Point3D>) -> Array<Point3D>:
    return points.map(transform)
    
// Compiler generates:
// - GPU kernel if data size > threshold
// - SIMD vectorized CPU code otherwise
// - Parallel CPU threads if beneficial
```

**Speedup: Same or better, with less code**

---

### 2. Use Specific Types

#### ❌ Bad: Generic Types
```maneuver
function calculate_distance(p1, p2):
    // Compiler doesn't know types
    // Must insert runtime checks
    return sqrt((p2.x - p1.x)^2 + (p2.y - p1.y)^2)
```

#### ✅ Good: Specific Types with Units
```maneuver
function calculate_distance(
    p1: Point2D<meters> in robot_base,
    p2: Point2D<meters> in robot_base
) -> meters:
    // Compiler knows:
    // - Both points in same frame (no transform needed)
    // - Result is meters (compile-time dimensional analysis)
    // - Can inline and vectorize
    return sqrt((p2.x - p1.x)^2 + (p2.y - p1.y)^2)
```

**Speedup: 3-5x** (eliminates checks, enables inlining)

---

### 3. Constrain Values

#### ❌ Bad: Unconstrained
```maneuver
function set_joint_angle(angle: degrees):
    // Runtime check needed
    if angle < -180 or angle > 180:
        error("Invalid angle")
    
    motor.set(angle)
```

#### ✅ Good: Type-Level Constraints
```maneuver
function set_joint_angle(angle: degrees where -180° ≤ value ≤ 180°):
    // No runtime check! Compiler proves angle is valid
    motor.set(angle)
```

**Speedup: 2x** in tight loops (eliminates checks)

---

### 4. Use Compile-Time Computation

#### ❌ Bad: Runtime Computation
```maneuver
function init():
    // Computed every time program starts
    sin_table: Array<float> = []
    for i in 0..360:
        sin_table.append(sin(i * degrees))
    
    return sin_table
```

#### ✅ Good: Compile-Time Computation
```maneuver
// Computed once during compilation
const sin_table = compile_time:
    [sin(i * degrees) for i in 0..360]

// Baked into binary, zero runtime cost
function lookup_sin(angle: int) -> float:
    return sin_table[angle]
```

**Speedup: ∞** (infinite - zero runtime cost vs milliseconds)

---

### 5. Leverage Pattern Matching

#### ❌ Bad: If-Else Chains
```maneuver
function handle_sensor_data(data):
    if data.type == "lidar":
        process_lidar(data)
    else if data.type == "camera":
        process_camera(data)
    else if data.type == "imu":
        process_imu(data)
    else:
        error("Unknown sensor")
```

#### ✅ Good: Pattern Matching
```maneuver
function handle_sensor_data(data: SensorData):
    match data:
        Lidar(points) -> process_lidar(points)
        Camera(image) -> process_camera(image)
        IMU(reading) -> process_imu(reading)

// Compiler generates jump table or inline code
// No string comparisons at runtime
```

**Speedup: 10x** (jump table vs string comparison)

---

### 6. Batch Operations

#### ❌ Bad: One-at-a-Time
```maneuver
for point in point_cloud:
    transformed = transform_point(point, T)
    if transformed.z > 0:
        filtered.append(transformed)

// CPU processes one point at a time
// Many function calls
// Poor cache usage
```

#### ✅ Good: Vectorized Operations
```maneuver
// Transform all points at once
transformed = point_cloud.transform(T)

// Filter in one operation
filtered = transformed.filter(z > 0)

// Compiler generates:
// - SIMD vectorized code (8-16 points at once)
// - GPU kernel if data is large
// - Cache-friendly access patterns
```

**Speedup: 10-50x** (SIMD + GPU)

---

### 7. Avoid Unnecessary Copies

#### ❌ Bad: Copying Data
```maneuver
function process_image(image: Image) -> Image:
    // Creates copy
    result: Image = Image.new(image.width, image.height)
    
    for pixel in image.pixels:
        result.pixels[i] = process_pixel(pixel)
    
    return result
```

#### ✅ Good: In-Place Modification
```maneuver
function process_image(image: mut Image):
    // Modifies in-place, no copy
    for pixel in image.pixels:
        pixel.brightness *= 1.2

// Or use immutable with compiler optimization:
function process_image(image: Image) -> Image:
    return image.map_pixels(p -> p * 1.2)
    // Compiler may optimize to in-place if possible
```

**Speedup: 3-10x** (no allocation/copying)

---

### 8. Use Appropriate Data Structures

#### ❌ Bad: Wrong Structure
```maneuver
// Finding nearby points
points: Array<Point3D>

function find_nearby(query: Point3D, radius: meters) -> Array<Point3D>:
    nearby: Array<Point3D> = []
    for point in points:  // O(n) - slow!
        if distance(query, point) < radius:
            nearby.append(point)
    return nearby
```

#### ✅ Good: Spatial Structure
```maneuver
// Use built-in spatial index
points: Octree<Point3D>

function find_nearby(query: Point3D, radius: meters) -> Array<Point3D>:
    return points.within_radius(query, radius)
    // O(log n) - fast!
```

**Speedup: 100-1000x** for large point clouds

---

### 9. Minimize Dynamic Dispatch

#### ❌ Bad: Virtual Methods
```maneuver
trait Sensor:
    function read() -> float  // Dynamic dispatch

function process_all(sensors: Array<Sensor>):
    for sensor in sensors:
        value = sensor.read()  // Vtable lookup each iteration
        process(value)
```

#### ✅ Good: Static Dispatch
```maneuver
// Compiler generates specialized version for each sensor type
function process_all<S: Sensor>(sensors: Array<S>):
    for sensor in sensors:
        value = sensor.read()  // Direct call, can inline
        process(value)

// Or use enum for closed set
enum SensorType:
    Lidar(LidarData)
    Camera(CameraData)
    IMU(IMUData)

function process_all(sensors: Array<SensorType>):
    for sensor in sensors:
        match sensor:
            Lidar(data) -> process_lidar(data)  // Direct call
            Camera(data) -> process_camera(data)
            IMU(data) -> process_imu(data)
```

**Speedup: 2-5x** (eliminates vtable lookups)

---

### 10. Parallelize When Possible

#### ❌ Bad: Sequential
```maneuver
function process_cameras(cameras: Array<Camera>) -> Array<Image>:
    results: Array<Image> = []
    for camera in cameras:
        image = camera.capture()
        processed = process_image(image)
        results.append(processed)
    return results
```

#### ✅ Good: Automatic Parallelization
```maneuver
function process_cameras(cameras: Array<Camera>) -> Array<Image>:
    // Compiler detects no dependencies between iterations
    // Automatically parallelizes across CPU cores
    return cameras.map(camera -> camera.capture().process())

// Or explicit parallel:
#[parallel]
function process_cameras(cameras: Array<Camera>) -> Array<Image>:
    return cameras.map(camera -> camera.capture().process())
```

**Speedup: 4-16x** (matches CPU core count)

---

## 🎓 Advanced Techniques

### 1. Manual GPU Offloading

```maneuver
#[gpu]
function process_large_array(data: Array<float, 1000000>) -> Array<float>:
    // Forces GPU execution
    return data.map(x -> x * 2.0 + 1.0)

// Compiler generates CUDA kernel:
// __global__ void process_kernel(float* input, float* output, int n) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx < n) output[idx] = input[idx] * 2.0f + 1.0f;
// }
```

---

### 2. Memory Layout Control

```maneuver
// Array of Structures (AoS) - cache unfriendly
#[layout(aos)]
particles_aos: Array<Particle>

// Structure of Arrays (SoA) - cache friendly
#[layout(soa)]
particles_soa: Array<Particle>

// Compiler transforms:
// AoS: [p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, ...]
// SoA: [p1.x, p2.x, p3.x, ...], [p1.y, p2.y, p3.y, ...], [p1.z, p2.z, p3.z, ...]
```

**When to use SoA:**
- Processing one field at a time
- SIMD operations
- GPU kernels

**Speedup: 5-8x** (better cache utilization)

---

### 3. Profile-Guided Optimization

```bash
# Step 1: Compile with profiling
maneuver build --profile

# Step 2: Run program to collect data
./my_robot --collect-profile

# Step 3: Recompile with profile data
maneuver build --use-profile profile_data.prof

# Compiler uses actual runtime data to:
# - Inline hot functions
# - Optimize branch predictions
# - Reorder code for cache
```

**Speedup: 10-30%** on top of normal optimizations

---

### 4. Explicit Inlining

```maneuver
// Force inline for hot path
#[inline(always)]
function fast_transform(point: Point3D, T: Matrix4x4) -> Point3D:
    return T * point

// Never inline (saves code size)
#[inline(never)]
function rare_error_handler():
    // ...
```

---

### 5. Cache-Friendly Algorithms

```maneuver
// Bad: Column-major access (cache misses)
for j in 0..width:
    for i in 0..height:
        process(image[i][j])  // Jumps around memory

// Good: Row-major access (cache friendly)
for i in 0..height:
    for j in 0..width:
        process(image[i][j])  // Sequential memory access
```

**Speedup: 5-10x** (cache hits vs misses)

---

## 🔬 Measuring Performance

### 1. Built-In Profiler

```maneuver
#[profile]
function expensive_computation():
    // Compiler instruments this function
    // Reports execution time
```

```bash
$ maneuver run --profile
Function                    | Calls | Total Time | Avg Time
expensive_computation       | 1000  | 523ms      | 0.52ms
transform_points           | 1000  | 89ms       | 0.09ms
```

---

### 2. Benchmarking

```maneuver
#[benchmark]
function bench_transform():
    points = generate_test_points(10000)
    
    start = timer()
    transformed = points.transform(T)
    end = timer()
    
    return end - start

// Run: maneuver benchmark bench_transform
// Output: Average: 2.3ms, Std Dev: 0.1ms
```

---

### 3. Compiler Optimization Reports

```bash
$ maneuver build --optimization-report

Optimizations Applied:
✓ transform_chain_fusion: 3 transforms combined
✓ gpu_offload: process_image moved to GPU (20x speedup)
✓ simd_vectorization: loop in update_particles (8x)
✓ constant_folding: 15 expressions computed at compile-time
✓ inlining: 42 functions inlined

Optimization Opportunities:
! Function 'complex_calculation' not vectorized (complex control flow)
! Array 'large_buffer' allocated on heap (consider stack allocation)
```

---

## 📋 Performance Checklist

### Before Optimizing

- [ ] Profile to find actual bottlenecks
- [ ] Measure baseline performance
- [ ] Set performance targets

### Code Level

- [ ] Use specific types with units
- [ ] Add value constraints to types
- [ ] Use compile-time computation
- [ ] Batch operations when possible
- [ ] Choose appropriate data structures
- [ ] Avoid unnecessary copies

### Compiler Level

- [ ] Enable optimization flags
- [ ] Use profile-guided optimization
- [ ] Check compiler optimization reports
- [ ] Verify GPU offloading when expected

### Verification

- [ ] Measure after optimization
- [ ] Compare with baseline
- [ ] Verify correctness maintained
- [ ] Document optimization choices

---

## 🎯 Common Pitfalls

### ❌ Pitfall 1: Premature Optimization

```maneuver
// Don't do this first!
#[inline(always)]
#[gpu]
#[simd]
function maybe_fast_but_unreadable():
    // Obscure, hand-optimized code
```

**Instead:** Write clear code, measure, then optimize if needed.

---

### ❌ Pitfall 2: Fighting the Compiler

```maneuver
// Don't try to outsmart the compiler
for i in 0..1000:
    // Manual loop unrolling
    result[i] = data[i] * 2
    result[i+1] = data[i+1] * 2
    result[i+2] = data[i+2] * 2
    result[i+3] = data[i+3] * 2
    i += 3
```

**Instead:** Let compiler unroll loops automatically.

---

### ❌ Pitfall 3: Ignoring Memory Patterns

```maneuver
// Bad: Random memory access
for index in random_indices:
    process(large_array[index])  // Cache misses!
```

**Instead:** Sort indices or use data structures with better locality.

---

## 💡 Quick Wins

### 1. Add Type Annotations
**Effort:** 5 minutes  
**Speedup:** 2-3x  
**Reason:** Eliminates runtime checks

### 2. Use Built-In Spatial Structures
**Effort:** 10 minutes  
**Speedup:** 100-1000x  
**Reason:** O(log n) vs O(n) search

### 3. Mark Constants as `const`
**Effort:** 1 minute  
**Speedup:** Variable  
**Reason:** Enables compile-time computation

### 4. Enable PGO
**Effort:** 15 minutes  
**Speedup:** 10-30%  
**Reason:** Better branch prediction and inlining

### 5. Check Compiler Reports
**Effort:** 5 minutes  
**Speedup:** Identifies opportunities  
**Reason:** Know what compiler is doing

---

## 📊 Performance Examples

### Example 1: Sensor Fusion

**Before:**
```maneuver
function sensor_fusion(lidar, camera, imu):
    l = read_lidar()
    c = read_camera()
    i = read_imu()
    return combine(l, c, i)
// 100ms - sequential
```

**After:**
```maneuver
#[parallel]
function sensor_fusion(lidar, camera, imu):
    return combine(
        lidar.read(),
        camera.read(),
        imu.read()
    )
// 35ms - parallel
```

**Speedup: 2.9x**

---

### Example 2: Point Cloud Processing

**Before:**
```maneuver
filtered = []
for point in cloud:
    if point.z > 0 and point.z < 2:
        filtered.append(point)
// 50ms - CPU, sequential
```

**After:**
```maneuver
filtered = cloud.filter(0m < z < 2m)
// 1ms - GPU, vectorized
```

**Speedup: 50x**

---

## 🎓 Further Learning

### Resources
- [PERFORMANCE.md](./PERFORMANCE.md) - Detailed performance architecture
- [ARCHITECTURE.md](./ARCHITECTURE.md) - Compiler internals
- [Example optimized projects](./examples/optimized/)

### Profiling Tools
- `maneuver profile` - Built-in profiler
- `maneuver benchmark` - Benchmarking suite
- `perf` - Linux performance analyzer
- `Instruments` - macOS profiler

---

## 🤝 Contributing

Found a performance trick? Share it!
- Add to this guide via PR
- Share in discussions
- Write blog post

---

<div align="center">

**Remember: Clear code first, optimize second, measure always.**

[Back to Main README](./README.md) • [Performance Architecture](./PERFORMANCE.md) • [Compiler Design](./ARCHITECTURE.md)

</div>
