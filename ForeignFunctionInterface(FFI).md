# 🔗 Foreign Function Interface (FFI)

MANEUVER is designed to interoperate seamlessly with **C, C++, Python, and Rust**, enabling reuse of existing libraries while preserving safety and performance.

---

## Design Principles

* **Zero-cost abstractions** where possible
* Explicit unsafe boundaries
* Verified data layouts
* Deterministic ownership and lifetimes

---

## C / C++ Interoperability

### Calling C/C++ from MANEUVER

```maneuver
extern "C" function fast_ik(
    joints: Array<float64>,
    target: Pose
) -> Array<float64>
```

* Headers parsed at compile time
* ABI compatibility verified
* No hidden allocations

### Using Existing Libraries

```maneuver
use library "libopencv"

image = cv.imread("frame.png")
```

---

## Exposing MANEUVER to C/C++

```maneuver
export function plan_path(start: Pose, goal: Pose) -> Trajectory
```

Generates:

* C header (`.h`)
* C++ wrapper (`.hpp`)
* Stable ABI

---

## Python Bindings

### Automatic Binding Generation

```bash
maneuver bind --python my_module
```

Produces:

* `pip`-installable package
* NumPy-compatible arrays
* Zero-copy buffers when possible

### Example Usage

```python
import maneuver
traj = maneuver.plan_path(start, goal)
```

---

## Rust FFI

```maneuver
#[rust_ffi]
export function compute_control(state: VehicleState) -> Control
```

Generates:

* `extern "C"` bindings
* Idiomatic Rust wrappers
* Ownership-safe APIs

---

## Memory & Ownership Rules

| Scenario        | Strategy              |
| --------------- | --------------------- |
| Small structs   | Stack-passed          |
| Large arrays    | Borrowed slices       |
| Long-lived data | Reference-counted     |
| Real-time loops | No allocation allowed |

Violations fail at compile time.

---

## Unsafe Blocks

```maneuver
unsafe:
    justification: "Vendor SDK access"
    duration: 5ms
    call vendor_sdk_function()
```

* All unsafe usage is audited
* Cannot cross task boundaries

---

## Performance Characteristics

* No marshaling for POD types
* SIMD preserved across FFI
* Predictable latency

---

## Use Cases

* Use CUDA libraries directly
* Integrate proprietary robot SDKs
* Wrap legacy C++ motion planners
* Expose MANEUVER logic to Python ML stacks

---

**FFI in MANEUVER lets you keep your ecosystem—without giving up safety.**
