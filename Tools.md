# 🛠️ MANEUVER Tooling

This document describes the tooling ecosystem that supports MANEUVER development—from editors to build systems and analysis tools.

---

## IDE Support

### Official MANEUVER IDE (Planned)

Features:

* Natural-language-aware autocomplete
* Unit & frame visualization
* Real-time timing analysis
* Integrated simulator
* Formal verification assistant

---

## Editor Plugins

### VS Code

Planned features:

* Syntax highlighting
* Inline unit checking
* Hover frame visualization
* One-click deploy

```bash
maneuver tools install vscode
```

### JetBrains (IntelliJ, CLion)

* Language server integration
* Refactoring support

---

## Language Server (LSP)

MANEUVER provides an LSP for:

* Autocomplete
* Diagnostics
* Go-to-definition
* Semantic highlighting

Works across all editors supporting LSP.

---

## Linters & Formatters

### Formatter

```bash
maneuver fmt my_robot.mnvr
```

* Preserves natural language structure
* Stable formatting for diffs

### Linter

```bash
maneuver lint --strict
```

Detects:

* Ambiguous instructions
* Redundant unit conversions
* Potential real-time violations

---

## Build Tools

### Build

```bash
maneuver build --target jetson
```

Targets:

* Microcontrollers
* Embedded Linux
* x86_64
* ARM64

---

### Simulation

```bash
maneuver simulate my_robot
```

* Physics-based simulation
* Sensor noise models
* Deterministic replay

---

## Profiling & Analysis

```bash
maneuver profile --timing
maneuver profile --gpu
maneuver analyze --memory
```

Outputs:

* Worst-case execution time
* Cache miss rates
* GPU utilization

---

## Formal Verification Tools

```bash
maneuver verify --formal
```

* Proves safety properties
* Explains failed proofs
* Generates certification artifacts

---

## Continuous Integration

```yaml
- name: Verify MANEUVER
  run: maneuver verify --formal
```

Ensures:

* No regression in safety
* Timing constraints preserved

---

## Package Manager

```bash
maneuver install maneuver-vision
```

* Versioned, verified packages
* Hardware compatibility metadata

---

## Future Tooling

* Graphical frame editor
* Visual task scheduler
* Real-time trace viewer
* ROS graph visualizer

---

**Great languages need great tools—MANEUVER is built for both.**
