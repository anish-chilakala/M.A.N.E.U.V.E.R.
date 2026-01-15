# MANEUVER Compiler Architecture

> **Technical Deep Dive: How MANEUVER Transforms Clean Code into Blazing Fast Binaries**

[![Compiler](https://img.shields.io/badge/Backend-LLVM-orange)]()
[![Language](https://img.shields.io/badge/Implementation-Rust-red)]()
[![Status](https://img.shields.io/badge/Status-In_Development-yellow)]()

---

## 📐 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     MANEUVER SOURCE CODE                         │
│         robot arm: move forward 50cm                             │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        LEXER / TOKENIZER                         │
│     Converts text → tokens [ROBOT, ARM, MOVE, FORWARD, 50, CM]  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                            PARSER                                │
│           Tokens → Abstract Syntax Tree (AST)                    │
│               Robot { Move { Forward, 50cm } }                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      SEMANTIC ANALYZER                           │
│    • Type checking (units, frames, bounds)                       │
│    • Symbol resolution                                           │
│    • Scope analysis                                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    HIGH-LEVEL IR (HIR)                           │
│          Platform-independent representation                     │
│          • Physical units attached to types                      │
│          • Coordinate frames tracked                             │
│          • Effect system annotated                               │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    MID-LEVEL IR (MIR)                            │
│          • Borrow checking                                       │
│          • Lifetime analysis                                     │
│          • Optimization passes                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  ROBOTICS OPTIMIZATION LAYER                     │
│    • Domain-specific pattern recognition                         │
│    • GPU kernel generation                                       │
│    • Compile-time computation                                    │
│    • SIMD vectorization                                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         LLVM IR                                  │
│           Low-level, platform-independent IR                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    LLVM OPTIMIZATION PASSES                      │
│    • Constant folding                                            │
│    • Dead code elimination                                       │
│    • Loop unrolling                                              │
│    • Inlining                                                    │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      CODE GENERATION                             │
│    Machine code for target platform:                             │
│    x86-64, ARM, RISC-V, CUDA, etc.                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    EXECUTABLE BINARY                             │
│              Ready to run on robot hardware                      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 Component Details

### 1. Lexer / Tokenizer

**Technology:** Hand-written or pest (parser generator)

**Responsibilities:**
- Convert source code text into tokens
- Handle keywords, identifiers, literals, operators
- Track source locations for error messages
- Handle comments and whitespace

**Key Features:**
- Context-sensitive lexing for units (cm, degrees, etc.)
- Support for physical constants (π, e, c)
- Unicode support for mathematical symbols

**Implementation (Rust):**
```rust
pub enum Token {
    // Keywords
    Robot, Move, Turn, Wait,
    
    // Literals
    Number(f64),
    Unit(String),      // cm, m, degrees
    Identifier(String),
    
    // Operators
    Plus, Minus, Star, Slash,
    
    // Special
    Colon, Arrow, Newline, Eof,
}

pub struct Lexer {
    input: Vec<char>,
    position: usize,
    line: usize,
    column: usize,
}

impl Lexer {
    pub fn next_token(&mut self) -> Token { /* ... */ }
}
```

---

### 2. Parser

**Technology:** Recursive descent or LALR parser (lalrpop)

**Responsibilities:**
- Build Abstract Syntax Tree (AST) from tokens
- Enforce grammar rules
- Generate parse errors with helpful messages

**Grammar (Simplified EBNF):**
```ebnf
program        ::= declaration*
declaration    ::= robot_decl | function_decl | task_decl

robot_decl     ::= "robot" identifier ":" statement*
statement      ::= move_stmt | turn_stmt | wait_stmt | expr_stmt

move_stmt      ::= "move" direction distance unit
turn_stmt      ::= "turn" direction angle unit
wait_stmt      ::= "wait" duration unit

expression     ::= literal | identifier | binary_op | call
binary_op      ::= expression operator expression
```

**AST Structure (Rust):**
```rust
pub enum ASTNode {
    Robot {
        name: String,
        body: Vec<Statement>,
        span: Span,
    },
    
    Move {
        direction: Direction,
        distance: Expression,
        unit: Unit,
        span: Span,
    },
    
    BinaryOp {
        left: Box<Expression>,
        op: Operator,
        right: Box<Expression>,
        span: Span,
    },
    
    // ... more variants
}
```

---

### 3. Semantic Analyzer

**Responsibilities:**
- Type checking with physical dimensions
- Symbol table management
- Scope resolution
- Coordinate frame verification
- Effect system checking

**Type System:**
```rust
pub struct Type {
    base: BaseType,
    dimensions: PhysicalDimensions,
    frame: Option<CoordinateFrame>,
    bounds: Option<ValueBounds>,
    effects: EffectSet,
}

pub struct PhysicalDimensions {
    length: i8,    // meters^1
    time: i8,      // seconds^1
    mass: i8,      // kilograms^1
    // ... more SI dimensions
}

pub struct CoordinateFrame {
    name: String,
    parent: Option<Box<CoordinateFrame>>,
}

pub enum Effect {
    Pure,           // No side effects
    IO,             // Can read/write files
    Hardware,       // Accesses hardware
    Unsafe,         // Potentially dangerous
    Timed(Duration),// Has timing constraint
}
```

**Unit Checking Algorithm:**
```rust
fn check_addition(left: &Type, right: &Type) -> Result<Type, TypeError> {
    // Can only add quantities with same dimensions
    if left.dimensions != right.dimensions {
        return Err(TypeError::IncompatibleDimensions {
            left: left.dimensions.clone(),
            right: right.dimensions.clone(),
        });
    }
    
    // Result has same dimensions
    Ok(Type {
        base: left.base.clone(),
        dimensions: left.dimensions.clone(),
        frame: unify_frames(left.frame, right.frame)?,
        bounds: merge_bounds(left.bounds, right.bounds),
        effects: left.effects.union(&right.effects),
    })
}
```

---

### 4. High-Level IR (HIR)

**Purpose:** Platform-independent representation with high-level semantics

**Features:**
- Physical units preserved
- Coordinate frames explicit
- Control flow graphs
- Data flow analysis

**Structure:**
```rust
pub struct HIR {
    functions: Vec<Function>,
    types: Vec<TypeDefinition>,
    constants: Vec<Constant>,
}

pub struct Function {
    name: String,
    params: Vec<Parameter>,
    return_type: Type,
    body: BasicBlock,
    effects: EffectSet,
    timing_constraint: Option<TimingConstraint>,
}

pub struct BasicBlock {
    instructions: Vec<Instruction>,
    terminator: Terminator,
}

pub enum Instruction {
    // Robotics-specific
    Transform {
        point: Value,
        from_frame: Frame,
        to_frame: Frame,
        result: Register,
    },
    
    ReadSensor {
        sensor: SensorId,
        result: Register,
    },
    
    // General computation
    BinaryOp {
        op: BinaryOperator,
        left: Value,
        right: Value,
        result: Register,
    },
    
    Call {
        function: FunctionId,
        args: Vec<Value>,
        result: Register,
    },
}
```

---

### 5. Robotics Optimization Layer

**This is where the magic happens!**

#### 5.1 Pattern Recognition

**Recognize common robotics patterns and optimize them:**

```rust
pub trait OptimizationPass {
    fn name(&self) -> &str;
    fn run(&mut self, hir: &mut HIR) -> OptimizationResult;
}

pub struct TransformChainOptimizer;

impl OptimizationPass for TransformChainOptimizer {
    fn run(&mut self, hir: &mut HIR) -> OptimizationResult {
        // Find chains of transformations
        // T1 * T2 * T3 * point
        
        // If T1, T2, T3 are compile-time constants:
        // Pre-compute T_combined = T1 * T2 * T3
        // Replace with: T_combined * point
        
        // Speedup: 3x fewer matrix multiplications
    }
}
```

**Optimizations Applied:**

1. **Transform Chain Fusion**
   - Combines multiple coordinate transformations
   - Pre-computes constant transformations

2. **Inverse Kinematics Recognition**
   - Detects IK patterns
   - Replaces with analytical solutions when available
   - Falls back to optimized numerical solver

3. **Kalman Filter Specialization**
   - Recognizes Kalman filter usage
   - Generates specialized code for specific dimensions
   - Pre-computes matrix inverses

4. **Trajectory Pattern Caching**
   - Identifies repeated trajectory patterns
   - Builds lookup table at compile time
   - Enables O(1) trajectory retrieval

#### 5.2 GPU Kernel Generation

**Automatically generate CUDA/OpenCL kernels:**

```rust
pub struct GPUKernelGenerator;

impl GPUKernelGenerator {
    pub fn generate(&self, operations: &[Operation]) -> Option<GPUKernel> {
        // 1. Analyze operations for GPU suitability
        if !self.is_gpu_suitable(operations) {
            return None;
        }
        
        // 2. Fuse compatible operations
        let fused_ops = self.fuse_operations(operations);
        
        // 3. Generate CUDA kernel
        let kernel = self.codegen_cuda(fused_ops);
        
        // 4. Add memory transfer logic
        kernel.add_memory_management();
        
        Some(kernel)
    }
    
    fn is_gpu_suitable(&self, ops: &[Operation]) -> bool {
        // Check for:
        // - Data parallelism
        // - Sufficient work (> 10000 elements)
        // - No complex control flow
        // - No I/O operations
    }
}
```

**GPU Kernel Fusion Example:**

```
Original:
  img → grayscale → blur → edges

Generated CUDA:
__global__ void fused_pipeline(uint8_t* input, uint8_t* output) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    // Grayscale inline
    float gray = 0.299*input[idx*3] + 0.587*input[idx*3+1] + 0.114*input[idx*3+2];
    
    // Blur inline (using shared memory)
    __shared__ float tile[TILE_SIZE];
    tile[threadIdx.x] = blur_kernel(gray);
    __syncthreads();
    
    // Edge detection inline
    output[idx] = sobel_operator(tile);
}
```

#### 5.3 Compile-Time Computation

**Evaluate as much as possible at compile time:**

```rust
pub struct ConstEvaluator;

impl ConstEvaluator {
    pub fn evaluate(&self, expr: &Expression) -> Option<Value> {
        match expr {
            Expression::Literal(val) => Some(val.clone()),
            
            Expression::BinaryOp { op, left, right } => {
                let l = self.evaluate(left)?;
                let r = self.evaluate(right)?;
                self.apply_op(op, l, r)
            }
            
            Expression::Call { func, args } if func.is_const => {
                // Execute function at compile time
                self.const_call(func, args)
            }
            
            _ => None,
        }
    }
    
    fn const_call(&self, func: &Function, args: &[Value]) -> Option<Value> {
        // Interpret function in compile-time VM
        let mut vm = ConstVM::new();
        vm.execute(func, args)
    }
}
```

**Example:**

```maneuver
// User writes:
const sin_table = [sin(i * degrees) for i in 0..360]

// Compiler executes this loop at compile time
// Result: 360 pre-computed values baked into binary
```

---

### 6. LLVM IR Generation

**Convert HIR to LLVM IR:**

```rust
pub struct LLVMCodegen {
    context: LLVMContext,
    module: LLVMModule,
    builder: LLVMBuilder,
}

impl LLVMCodegen {
    pub fn codegen_function(&mut self, func: &Function) -> LLVMValue {
        // Create function signature
        let fn_type = self.convert_function_type(func);
        let llvm_func = self.module.add_function(&func.name, fn_type);
        
        // Generate body
        for block in &func.body {
            let llvm_block = llvm_func.append_basic_block(&block.name);
            self.builder.position_at_end(llvm_block);
            
            for inst in &block.instructions {
                self.codegen_instruction(inst);
            }
        }
        
        llvm_func
    }
    
    fn codegen_instruction(&mut self, inst: &Instruction) {
        match inst {
            Instruction::BinaryOp { op, left, right, result } => {
                let l = self.get_value(left);
                let r = self.get_value(right);
                
                let val = match op {
                    BinaryOperator::Add => self.builder.build_fadd(l, r),
                    BinaryOperator::Mul => self.builder.build_fmul(l, r),
                    // ...
                };
                
                self.set_value(result, val);
            }
            
            Instruction::Transform { point, from, to, result } => {
                // Generate optimized transformation code
                self.codegen_transform(point, from, to, result);
            }
            
            // ... more instructions
        }
    }
}
```

---

### 7. Verification Engine

**Formal verification integration:**

```rust
pub struct VerificationEngine {
    smt_solver: Z3Solver,
}

impl VerificationEngine {
    pub fn verify_bounds(&self, func: &Function) -> VerificationResult {
        // Convert function to SMT constraints
        let constraints = self.function_to_smt(func);
        
        // Check if bounds are always satisfied
        for bound in &func.bounds {
            let negation = self.negate_bound(bound);
            
            if self.smt_solver.check_sat(&constraints, &negation) == Satisfiable {
                return VerificationResult::Failed {
                    reason: "Bound can be violated",
                    counterexample: self.smt_solver.get_model(),
                };
            }
        }
        
        VerificationResult::Verified
    }
    
    pub fn verify_timing(&self, func: &Function) -> VerificationResult {
        // Worst-case execution time analysis
        let wcet = self.compute_wcet(func);
        
        if let Some(deadline) = func.timing_constraint {
            if wcet > deadline {
                return VerificationResult::Failed {
                    reason: format!("WCET {} exceeds deadline {}", wcet, deadline),
                };
            }
        }
        
        VerificationResult::Verified
    }
}
```

---

## 🎯 Key Design Decisions

### 1. Why Rust for Compiler Implementation?

**Pros:**
- Memory safety prevents compiler bugs
- Excellent error handling (Result types)
- Pattern matching perfect for AST traversal
- Fast compilation
- Great LLVM bindings

**Cons:**
- Steeper learning curve
- Longer initial development

**Decision:** Rust is worth it for long-term compiler reliability.

---

### 2. Why LLVM Backend?

**Pros:**
- Mature, battle-tested
- Excellent optimization passes
- Supports many target architectures
- Used by Rust, Swift, Clang

**Cons:**
- Large dependency
- Slower compilation than custom backend

**Decision:** LLVM's benefits far outweigh the costs.

---

### 3. Multiple IR Levels (HIR, MIR, LLVM IR)

**Why not just one IR?**

- **HIR:** Preserves high-level robotics semantics (units, frames)
- **MIR:** Good for borrow checking and lifetime analysis
- **LLVM IR:** Low-level, close to machine code

Each level enables different optimizations.

---

## 📊 Compilation Pipeline Performance

### Typical Compilation Times

| Phase | Time (small program) | Time (large program) |
|-------|---------------------|---------------------|
| Lexing | 5ms | 50ms |
| Parsing | 10ms | 150ms |
| Semantic Analysis | 20ms | 300ms |
| HIR Generation | 15ms | 200ms |
| Robotics Optimization | 50ms | 1000ms |
| LLVM Codegen | 100ms | 2000ms |
| **Total** | **200ms** | **3.7s** |

**Target:** Keep compilation under 5 seconds for large projects.

---

## 🔬 Advanced Features

### 1. Incremental Compilation

**Problem:** Recompiling entire project is slow

**Solution:** Only recompile changed modules

```rust
pub struct IncrementalCompiler {
    cache: CompilationCache,
    dependency_graph: DependencyGraph,
}

impl IncrementalCompiler {
    pub fn compile(&mut self, changed_files: &[PathBuf]) -> Result<Binary> {
        // 1. Find affected modules
        let affected = self.dependency_graph.get_affected(changed_files);
        
        // 2. Reuse cached results for unchanged modules
        let mut compiled = HashMap::new();
        for module in self.all_modules() {
            if affected.contains(module) {
                compiled.insert(module, self.compile_module(module)?);
            } else {
                compiled.insert(module, self.cache.get(module)?);
            }
        }
        
        // 3. Link
        self.link(compiled)
    }
}
```

---

### 2. Cross-Compilation

**Support multiple target platforms:**

```rust
pub enum Target {
    X86_64_Linux,
    ARM_Linux,
    ARM_Bare_Metal,
    RISC_V,
    CUDA_PTX,
}

impl Compiler {
    pub fn compile_for_target(&self, target: Target) -> Result<Binary> {
        // Configure LLVM for target
        self.llvm.set_target_triple(target.triple());
        self.llvm.set_data_layout(target.layout());
        
        // Generate code
        let binary = self.compile()?;
        
        // Target-specific post-processing
        target.post_process(binary)
    }
}
```

---

### 3. Link-Time Optimization (LTO)

**Optimize across module boundaries:**

```rust
pub struct LTOPass;

impl OptimizationPass for LTOPass {
    fn run(&mut self, modules: &mut [Module]) -> OptimizationResult {
        // 1. Inline functions across modules
        self.cross_module_inlining(modules);
        
        // 2. Dead code elimination
        self.remove_unused_functions(modules);
        
        // 3. Constant propagation across modules
        self.cross_module_const_prop(modules);
    }
}
```

---

## 🛠️ Debugging the Compiler

### Compiler Flags for Debugging

```bash
# Show lexer output
maneuver compile --dump-tokens program.mnvr

# Show AST
maneuver compile --dump-ast program.mnvr

# Show HIR
maneuver compile --dump-hir program.mnvr

# Show LLVM IR
maneuver compile --dump-llvm program.mnvr

# Show optimization passes
maneuver compile --verbose-opt program.mnvr

# Show timing for each phase
maneuver compile --time-phases program.mnvr
```

---

## 📚 Further Reading

- **LLVM Documentation:** https://llvm.org/docs/
- **Rust Compiler Internals:** https://rustc-dev-guide.rust-lang.org/
- **Type Systems:** "Types and Programming Languages" by Pierce
- **Compiler Design:** "Engineering a Compiler" by Cooper & Torczon
- **SMT Solvers:** Z3 Tutorial and Guide

---

## 🤝 Contributing to Compiler

See [CONTRIBUTING.md](CONTRIBUTING.md) for:
- Setting up development environment
- Running compiler tests
- Adding new optimization passes
- Debugging techniques

---

<div align="center">

**MANEUVER Compiler: Transforming natural language into optimal machine code**

[Back to Main README](./README.md) • [Performance Details](./PERFORMANCE.md) • [Optimization Guide](./OPTIMIZATION-GUIDE.md)

</div>
