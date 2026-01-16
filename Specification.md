# MANEUVER Language Specification

**Version:** 1.0.0  
**Status:** Draft  
**Last Updated:** January 2026

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Lexical Structure](#2-lexical-structure)
3. [Types](#3-types)
4. [Expressions](#4-expressions)
5. [Statements](#5-statements)
6. [Declarations](#6-declarations)
7. [Modules](#7-modules)
8. [Coordinate Frames](#8-coordinate-frames)
9. [Physical Units](#9-physical-units)
10. [Effect System](#10-effect-system)
11. [Real-Time Semantics](#11-real-time-semantics)
12. [Memory Model](#12-memory-model)
13. [Formal Semantics](#13-formal-semantics)

---

## 1. Introduction

### 1.1 Purpose

This document defines the syntax and semantics of the MANEUVER programming language. It serves as the authoritative reference for:
- Language implementers
- Tool developers
- Advanced users
- Standards bodies

### 1.2 Notation

This specification uses the following notation:

- **EBNF Grammar**: Extended Backus-Naur Form
- `keyword`: Language keywords in fixed-width font
- `<non-terminal>`: Grammar non-terminals in angle brackets
- `[optional]`: Optional elements
- `{repeated}`: Zero or more repetitions
- `(choice | alternative)`: Alternatives

### 1.3 Design Philosophy

MANEUVER is designed around three core principles:

1. **Natural Expression**: Code reads like precise English
2. **Mathematical Rigor**: Type system prevents entire classes of errors
3. **Zero-Cost Safety**: Abstractions compile to optimal machine code

---

## 2. Lexical Structure

### 2.1 Character Set

MANEUVER source files are UTF-8 encoded.

### 2.2 Comments

```ebnf
Comment ::= LineComment | BlockComment
LineComment ::= "//" {any character except newline} newline
BlockComment ::= "/*" {any character} "*/"
```

**Examples:**
```maneuver
// This is a line comment

/* This is a
   block comment */
```

### 2.3 Keywords

Reserved keywords:
```
robot       task        function    type        struct
enum        trait       impl        where       requires
ensures     maintains   deadline    frequency   priority
sensor      actuator    if          else        loop
while       for         in          break       continue
return      move        turn        stop        say
read        write       async       await       unsafe
import      export      module      public      private
let         const       var         true        false
and         or          not         xor         is
as          at          to          from        with
```

### 2.4 Identifiers

```ebnf
Identifier ::= Letter {Letter | Digit | "_"}
Letter ::= "a".."z" | "A".."Z"
Digit ::= "0".."9"
```

**Rules:**
- Must start with a letter
- Case-sensitive
- Cannot be a keyword
- Snake_case recommended for variables/functions
- PascalCase recommended for types

### 2.5 Literals

#### Integer Literals
```ebnf
IntLiteral ::= DecimalLit | HexLit | BinaryLit | OctalLit
DecimalLit ::= Digit {Digit | "_"}
HexLit ::= "0x" HexDigit {HexDigit | "_"}
BinaryLit ::= "0b" BinDigit {BinDigit | "_"}
OctalLit ::= "0o" OctDigit {OctDigit | "_"}
```

**Examples:**
```maneuver
42
1_000_000
0xFF
0b1010
0o755
```

#### Float Literals
```ebnf
FloatLiteral ::= Digit+ "." Digit+ [Exponent]
Exponent ::= ("e" | "E") ["+" | "-"] Digit+
```

**Examples:**
```maneuver
3.14
1.5e-10
6.022e23
```

#### String Literals
```ebnf
StringLiteral ::= '"' {Character | EscapeSeq} '"'
EscapeSeq ::= "\\" ("n" | "t" | "r" | "\\" | '"')
```

**Examples:**
```maneuver
"Hello, robot!"
"Line 1\nLine 2"
"Path: C:\\robots\\config"
```

#### Boolean Literals
```maneuver
true
false
```

### 2.6 Physical Unit Literals

```ebnf
UnitLiteral ::= Number Unit
Unit ::= BaseUnit | DerivedUnit | CompoundUnit
BaseUnit ::= "m" | "kg" | "s" | "A" | "K" | "mol" | "cd"
DerivedUnit ::= "meters" | "kilograms" | "seconds" | "degrees" | "radians" | ...
CompoundUnit ::= Unit ["*" | "/" | "^"] Unit
```

**Examples:**
```maneuver
5 meters
10.5 kg
90 degrees
2.5 m/s
9.8 m/s²
```

---

## 3. Types

### 3.1 Type System Overview

MANEUVER features a sophisticated type system with:
- **Primitive types**: integers, floats, booleans
- **Physical types**: quantities with units
- **Refinement types**: constrained values
- **Coordinate frame types**: spatial reference frames
- **Effect types**: computational effects
- **Dependent types**: types that depend on values

### 3.2 Primitive Types

```ebnf
PrimitiveType ::= IntType | FloatType | BoolType | StringType
IntType ::= "i8" | "i16" | "i32" | "i64" | "u8" | "u16" | "u32" | "u64"
FloatType ::= "f32" | "f64"
BoolType ::= "bool"
StringType ::= "string"
```

**Semantics:**
- `i8..i64`: Signed integers (8-64 bits)
- `u8..u64`: Unsigned integers (8-64 bits)
- `f32, f64`: IEEE 754 floating point
- `bool`: true or false
- `string`: UTF-8 encoded text

### 3.3 Physical Types

```ebnf
PhysicalType ::= Quantity "<" Unit ">"
Quantity ::= "length" | "mass" | "time" | "velocity" | "force" | ...
```

**Examples:**
```maneuver
distance: length<meters>
speed: velocity<m/s>
weight: mass<kilograms>
angle: angle<degrees>
```

**Type Rules:**
- Addition/subtraction: Same units required
- Multiplication: Units multiply
- Division: Units divide
- Conversion: Automatic within same dimension

### 3.4 Refinement Types

```ebnf
RefinementType ::= Type "where" Predicate
Predicate ::= Expression
```

**Examples:**
```maneuver
joint_angle: degrees where -180° ≤ value ≤ 180°
motor_speed: m/s where 0 ≤ value ≤ 2.0
battery_level: percent where 0 ≤ value ≤ 100
positive_int: i32 where value > 0
```

**Semantics:**
- Compiler verifies predicate at compile time when possible
- Runtime check inserted when compile-time verification impossible
- Assignment to refinement type requires proof of satisfaction

### 3.5 Coordinate Frame Types

```ebnf
FrameType ::= Type "in" Frame
Frame ::= Identifier
```

**Examples:**
```maneuver
point_world: Point3D in world_frame
velocity_robot: Vector3D in robot_frame
pose_camera: Pose in camera_frame
```

**Type Rules:**
- Operations require same frame
- Explicit transform required to change frames
- Compiler tracks frame at type level

### 3.6 Function Types

```ebnf
FunctionType ::= "(" [ParamType {"," ParamType}] ")" "->" ReturnType [EffectSpec]
ParamType ::= Identifier ":" Type
ReturnType ::= Type | "void"
EffectSpec ::= "!" "{" Effect {"," Effect} "}"
```

**Examples:**
```maneuver
// Pure function (no effects)
add: (x: i32, y: i32) -> i32

// Function with I/O effect
print: (msg: string) -> void !{IO}

// Function with hardware access
read_sensor: () -> f64 !{Hardware}
```

### 3.7 Struct Types

```ebnf
StructDef ::= "struct" Identifier "{" {Field} "}"
Field ::= Identifier ":" Type
```

**Example:**
```maneuver
struct Point3D {
    x: meters
    y: meters
    z: meters
}

struct Robot {
    name: string
    position: Point3D
    velocity: velocity<m/s>
}
```

### 3.8 Enum Types

```ebnf
EnumDef ::= "enum" Identifier "{" Variant {"," Variant} "}"
Variant ::= Identifier ["(" Type {"," Type} ")"]
```

**Example:**
```maneuver
enum RobotState {
    Idle,
    Moving(velocity<m/s>),
    Turning(degrees),
    Error(string)
}
```

### 3.9 Array Types

```ebnf
ArrayType ::= "[" Type ";" Size "]"
Size ::= IntLiteral | Identifier
```

**Examples:**
```maneuver
positions: [Point3D; 100]
sensor_readings: [f64; N]  // N is a compile-time constant
```

### 3.10 Type Inference

MANEUVER supports bidirectional type inference:

```maneuver
// Explicit type
distance: meters = 5.0 meters

// Inferred type
distance := 5.0 meters  // Type inferred as meters

// Inferred from usage
x := 10
y := x + 5  // y inferred as i32
```

---

## 4. Expressions

### 4.1 Literal Expressions

```ebnf
LiteralExpr ::= IntLiteral | FloatLiteral | StringLiteral | BoolLiteral | UnitLiteral
```

### 4.2 Binary Expressions

```ebnf
BinaryExpr ::= Expr BinOp Expr
BinOp ::= "+" | "-" | "*" | "/" | "%" | "^"
        | "==" | "!=" | "<" | ">" | "<=" | ">="
        | "and" | "or" | "xor"
```

**Type Rules:**

Arithmetic operators:
```
meters + meters -> meters        ✓
meters + seconds -> ERROR        ✗
meters * 2 -> meters            ✓
meters / seconds -> m/s         ✓
```

Comparison operators:
```
meters == meters -> bool        ✓
meters < meters -> bool         ✓
meters == seconds -> ERROR      ✗
```

### 4.3 Unary Expressions

```ebnf
UnaryExpr ::= UnaryOp Expr
UnaryOp ::= "-" | "not"
```

### 4.4 Call Expressions

```ebnf
CallExpr ::= Identifier "(" [Argument {"," Argument}] ")"
Argument ::= [Identifier ":"] Expr
```

**Examples:**
```maneuver
sqrt(16)
move_robot(distance: 5 meters, speed: 0.5 m/s)
```

### 4.5 Field Access

```ebnf
FieldAccess ::= Expr "." Identifier
```

**Example:**
```maneuver
robot.position.x
point.transform_to(world_frame)
```

### 4.6 Method Calls

```ebnf
MethodCall ::= Expr "." Identifier "(" [Arguments] ")"
```

**Example:**
```maneuver
point.transform_to(world_frame)
array.filter(x => x > 0)
```

### 4.7 Lambda Expressions

```ebnf
LambdaExpr ::= ["|" Params "|"] "=>" Expr
Params ::= Identifier {"," Identifier}
```

**Example:**
```maneuver
map(array, |x| x * 2)
filter(points, |p| p.z > 0)
```

---

## 5. Statements

### 5.1 Expression Statements

```ebnf
ExprStmt ::= Expr
```

### 5.2 Variable Declarations

```ebnf
VarDecl ::= ("let" | "const" | "var") Pattern [":" Type] "=" Expr
Pattern ::= Identifier | StructPattern | ArrayPattern
```

**Examples:**
```maneuver
let x = 10
const PI = 3.14159
var position: Point3D = Point3D(0, 0, 0)
```

**Semantics:**
- `let`: Immutable binding
- `const`: Compile-time constant
- `var`: Mutable binding

### 5.3 Assignment

```ebnf
Assignment ::= LValue "=" Expr
LValue ::= Identifier | FieldAccess | IndexAccess
```

**Example:**
```maneuver
x = 10
robot.position.x = 5 meters
array[0] = value
```

### 5.4 If Statements

```ebnf
IfStmt ::= "if" Expr ":" Block ["else" ":" Block]
Block ::= Statement | (Indent {Statement} Dedent)
```

**Example:**
```maneuver
if distance < 20cm:
    stop robot
    turn left 90 degrees
else:
    move forward
```

### 5.5 Loop Statements

```ebnf
LoopStmt ::= WhileLoop | ForLoop | InfiniteLoop
WhileLoop ::= "while" Expr ":" Block
ForLoop ::= "for" Pattern "in" Expr ":" Block
InfiniteLoop ::= "loop" ":" Block
```

**Examples:**
```maneuver
while sensor_value > threshold:
    adjust_position()

for point in point_cloud:
    process(point)

loop:
    read_sensors()
    update_control()
    if should_stop:
        break
```

### 5.6 Return Statement

```ebnf
ReturnStmt ::= "return" [Expr]
```

---

## 6. Declarations

### 6.1 Function Declarations

```ebnf
FunctionDecl ::= "function" Identifier "(" [Params] ")" ["->" Type] [Spec] ":" Block
Params ::= Param {"," Param}
Param ::= Identifier ":" Type
Spec ::= RequiresSpec | EnsuresSpec | DeadlineSpec
RequiresSpec ::= "requires" ":" Predicate
EnsuresSpec ::= "ensures" ":" Predicate
DeadlineSpec ::= "deadline" ":" Duration
```

**Example:**
```maneuver
function safe_move(target: Point3D) -> bool:
    requires: target in workspace
    ensures: collision_free
    deadline: 50ms
    
    path = plan_path(current_position, target)
    return execute_path(path)
```

### 6.2 Robot Declarations

```ebnf
RobotDecl ::= "robot" Identifier ":" RobotSpec
RobotSpec ::= {SensorDecl | ActuatorDecl | TaskDecl}
```

**Example:**
```maneuver
robot mobile_bot:
    sensor distance_sensor:
        type: ultrasonic
        location: front
        range: 0cm to 400cm
    
    actuator motors:
        type: differential_drive
        max_speed: 2 m/s
    
    task navigate:
        frequency: 10 Hz
        // Task implementation
```

### 6.3 Task Declarations

```ebnf
TaskDecl ::= "task" Identifier ":" TaskSpec
TaskSpec ::= [FrequencySpec] [DeadlineSpec] [PrioritySpec] Block
FrequencySpec ::= "frequency" ":" Frequency
DeadlineSpec ::= "deadline" ":" Duration
PrioritySpec ::= "priority" ":" Priority
```

**Example:**
```maneuver
task motor_control:
    frequency: 100 Hz
    deadline: 8ms
    priority: critical
    
    position = read encoder
    error = target - position
    control = PID(error)
    send motor_command(control)
```

---

## 7. Modules

### 7.1 Module System

```ebnf
Module ::= {Import} {Declaration}
Import ::= "import" ModulePath ["as" Identifier]
ModulePath ::= Identifier {"." Identifier}
```

**Example:**
```maneuver
import std.math
import robotics.kinematics as kin
import sensors.lidar

// Use imported modules
distance = math.sqrt(x^2 + y^2)
pose = kin.forward_kinematics(angles)
```

### 7.2 Visibility

```ebnf
VisibilitySpec ::= ["public" | "private"]
```

**Default:** Private within module, public if exported

---

## 8. Coordinate Frames

### 8.1 Frame Declarations

```ebnf
FrameDecl ::= "frame" Identifier [":" FrameSpec]
FrameSpec ::= "parent" ":" Identifier ["," "transform" ":" Transform]
```

**Example:**
```maneuver
frame world_frame

frame robot_base:
    parent: world_frame
    transform: position(1m, 2m, 0m), rotation(0, 0, π/2)

frame camera_frame:
    parent: robot_base
    transform: position(0.2m, 0m, 0.3m)
```

### 8.2 Frame Transformations

```ebnf
Transform ::= Expr "." "transform_to" "(" Frame ")"
```

**Example:**
```maneuver
point_world: Point3D in world_frame = point_camera.transform_to(world_frame)
```

**Type Rule:**
```
Point3D in frame_a -> Point3D in frame_b
Requires: Valid transformation exists from frame_a to frame_b
```

---

## 9. Physical Units

### 9.1 Unit System

MANEUVER uses SI units as base units:
- Length: meter (m)
- Mass: kilogram (kg)
- Time: second (s)
- Electric current: ampere (A)
- Temperature: kelvin (K)
- Amount of substance: mole (mol)
- Luminous intensity: candela (cd)

### 9.2 Derived Units

Common derived units:
- Velocity: m/s
- Acceleration: m/s²
- Force: newton (N = kg⋅m/s²)
- Energy: joule (J = N⋅m)
- Power: watt (W = J/s)
- Angle: radian (rad), degree (°)

### 9.3 Unit Conversion

**Automatic conversion within same dimension:**
```maneuver
distance: meters = 5.5 meters
distance_cm: centimeters = distance  // Automatically converts to 550 cm
```

**Explicit conversion:**
```maneuver
angle_rad: radians = (90 degrees).to_radians()
```

### 9.4 Unit Arithmetic

```
meters * meters = meters²
meters / seconds = m/s
(m/s) * seconds = meters
newtons / kilograms = m/s² (acceleration)
```

---

## 10. Effect System

### 10.1 Effect Types

MANEUVER tracks computational effects:

- `Pure`: No side effects
- `IO`: Input/output operations
- `Hardware`: Hardware access
- `Unsafe`: Unsafe operations
- `Diverge`: May not terminate
- `Panic`: May panic/crash

### 10.2 Effect Annotations

```ebnf
EffectAnnotation ::= "!" "{" Effect {"," Effect} "}"
```

**Example:**
```maneuver
function read_sensor() -> f64 !{Hardware}:
    // Hardware access

function print_debug(msg: string) -> void !{IO}:
    // I/O operation

function unsafe_pointer_access() -> void !{Unsafe}:
    // Unsafe operation
```

### 10.3 Effect Polymorphism

Functions can be polymorphic over effects:

```maneuver
function map<E>(array: [T; N], f: T -> U !E) -> [U; N] !E:
    // map inherits the effects of f
```

---

## 11. Real-Time Semantics

### 11.1 Timing Annotations

```maneuver
task control_loop:
    frequency: 100 Hz      // Must run exactly 100 times per second
    deadline: 8ms          // Must complete within 8ms
    priority: critical     // Highest priority
```

### 11.2 WCET Analysis

The compiler performs Worst-Case Execution Time analysis:

1. **Static analysis** of all code paths
2. **Proof** that deadline will be met
3. **Compile error** if deadline cannot be guaranteed

### 11.3 Scheduling

MANEUVER uses Rate Monotonic Scheduling:
- Tasks with shorter periods get higher priority
- Provably optimal for fixed-priority scheduling
- Compile-time schedulability analysis

---

## 12. Memory Model

### 12.1 Memory Safety

MANEUVER guarantees memory safety through:

1. **No null pointers**: All references are valid
2. **No dangling pointers**: Linear types prevent use-after-free
3. **No buffer overflows**: Bounds checking (compile-time when possible)
4. **No data races**: Effect system prevents unsafe concurrency

### 12.2 Ownership

```maneuver
// Ownership transfer
let data = allocate_buffer(1024)
process(data)  // data moved to process
// data no longer accessible here
```

### 12.3 Borrowing

```maneuver
// Immutable borrow
let data = [1, 2, 3, 4, 5]
print(data)  // Borrows data immutably
sum(data)    // Another immutable borrow (allowed)

// Mutable borrow
var data = [1, 2, 3, 4, 5]
modify(data)  // Borrows data mutably (exclusive)
```

---

## 13. Formal Semantics

### 13.1 Operational Semantics

**Small-step semantics:**

```
E-Add:
  n1 + n2 → n    where n = [[n1]] + [[n2]]

E-If-True:
  if true: S1 else: S2 → S1

E-If-False:
  if false: S1 else: S2 → S2

E-Transform:
  (point in frame_a).transform_to(frame_b) → point'
  where point' = T_{a→b} ⊗ point
```

### 13.2 Type System Formalization

**Typing judgments:**

```
Γ ⊢ e : T    (Expression e has type T in context Γ)

T-Int:
  ─────────
  Γ ⊢ n : i32

T-Add-Units:
  Γ ⊢ e1 : length<m>    Γ ⊢ e2 : length<m>
  ──────────────────────────────────────
  Γ ⊢ e1 + e2 : length<m>

T-Frame:
  Γ ⊢ e : Point3D in frame_a    frame_a → frame_b ∈ Γ
  ─────────────────────────────────────────────────
  Γ ⊢ e.transform_to(frame_b) : Point3D in frame_b
```

### 13.3 Safety Theorems

**Theorem (Type Safety):**
If `⊢ e : T`, then either:
1. `e` is a value, or
2. `e → e'` for some `e'` where `⊢ e' : T`

**Theorem (Memory Safety):**
Well-typed programs do not:
- Access invalid memory
- Use dangling pointers
- Overflow buffers

**Theorem (Frame Safety):**
Well-typed programs do not:
- Mix incompatible coordinate frames
- Perform invalid transformations

---

## Appendix A: Grammar Summary

Complete EBNF grammar available in `grammar.ebnf`

## Appendix B: Standard Library

Standard library documentation available in `stdlib.md`

## Appendix C: Reserved Words

Complete list of reserved keywords for future use.

---

**This specification is a living document and will be updated as the language evolves.**
