# Contributing to MANEUVER

Thank you for your interest in contributing to MANEUVER! This document provides guidelines for contributing to the project.

---

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Pull Request Process](#pull-request-process)
- [Style Guidelines](#style-guidelines)
- [Community](#community)

---

## Code of Conduct

### Our Pledge

We are committed to providing a welcoming and inclusive environment for everyone, regardless of:
- Age, body size, disability, ethnicity, gender identity and expression
- Level of experience, education, socio-economic status
- Nationality, personal appearance, race, religion
- Sexual identity and orientation

### Our Standards

**Positive behaviors:**
- Using welcoming and inclusive language
- Being respectful of differing viewpoints
- Gracefully accepting constructive criticism
- Focusing on what is best for the community
- Showing empathy towards other community members

**Unacceptable behaviors:**
- Harassment, trolling, or insulting comments
- Publishing others' private information without permission
- Other conduct which could reasonably be considered inappropriate

### Enforcement

Violations of the Code of Conduct should be reported to conduct@maneuver-lang.org. All complaints will be reviewed and investigated promptly and fairly.

---

## Getting Started

### Prerequisites

- Basic knowledge of programming
- Familiarity with robotics concepts (helpful but not required)
- Git installed on your system
- GitHub account

### Areas to Contribute

- 🐛 **Bug fixes** - Fix issues in the compiler or standard library
- ✨ **New features** - Implement language features or library functions
- 📚 **Documentation** - Improve docs, add examples, write tutorials
- 🧪 **Testing** - Write tests, improve test coverage
- 🎨 **Examples** - Create example projects
- 🌍 **Translation** - Translate documentation
- 💡 **Ideas** - Propose new features or improvements

---

## How to Contribute

### Reporting Bugs

Before creating a bug report:
1. Check if the bug has already been reported in [Issues](https://github.com/maneuver-lang/maneuver/issues)
2. Try to reproduce with the latest version
3. Collect relevant information (error messages, OS, version)

**Good bug report includes:**
- Clear title describing the issue
- Steps to reproduce
- Expected vs. actual behavior
- Code sample (minimal reproducible example)
- MANEUVER version and OS
- Screenshots or error logs if relevant

**Example:**
```markdown
**Bug**: Compiler crashes on large arrays

**Steps to Reproduce:**
1. Create file with: `array: [i32; 1000000] = [0; 1000000]`
2. Run: `maneuver build test.mnvr`
3. Compiler crashes with segmentation fault

**Environment:**
- MANEUVER version: 1.0.0
- OS: Ubuntu 22.04
- Hardware: 16GB RAM

**Expected:** Should compile successfully or show helpful error
**Actual:** Compiler crashes
```

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. Include:
- Clear, descriptive title
- Detailed description of the enhancement
- Why it would be useful
- Example code showing the desired syntax/behavior
- Consider alternative approaches

### First-Time Contributors

Look for issues labeled:
- `good first issue` - Simple issues for newcomers
- `help wanted` - Issues where we'd love contributions
- `documentation` - Documentation improvements

---

## Development Setup

### 1. Fork and Clone

```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/maneuver.git
cd maneuver
```

### 2. Install Dependencies

```bash
# Install Rust (for compiler development)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install LLVM
# Ubuntu/Debian:
sudo apt-get install llvm-14 llvm-14-dev

# macOS:
brew install llvm@14

# Install Z3 (SMT solver)
# Ubuntu/Debian:
sudo apt-get install z3 libz3-dev

# macOS:
brew install z3
```

### 3. Build from Source

```bash
# Build the compiler
cargo build --release

# Run tests
cargo test

# Install locally
cargo install --path .
```

### 4. Create a Branch

```bash
# Create a branch for your work
git checkout -b feature/my-feature
# or
git checkout -b fix/issue-123
```

---

## Pull Request Process

### Before Submitting

1. **Test your changes**
   ```bash
   cargo test
   cargo clippy  # Linter
   cargo fmt     # Format code
   ```

2. **Update documentation** if you changed APIs or added features

3. **Add tests** for new functionality

4. **Follow style guidelines** (see STYLE_GUIDE.md)

### Submitting PR

1. **Commit your changes**
   ```bash
   git add .
   git commit -m "Brief description of changes"
   ```

2. **Push to your fork**
   ```bash
   git push origin feature/my-feature
   ```

3. **Create Pull Request** on GitHub with:
   - Clear title describing the change
   - Description of what changed and why
   - Link to related issues (e.g., "Fixes #123")
   - Screenshots if UI changes

4. **Respond to feedback**
   - Address reviewer comments
   - Make requested changes
   - Push updates to the same branch

### PR Review Process

1. Automated CI tests will run
2. Maintainers will review your code
3. You may need to make changes
4. Once approved, a maintainer will merge

**Be patient!** Reviews may take a few days. We appreciate your contribution!

---

## Style Guidelines

### Code Style

Follow the [STYLE_GUIDE.md](STYLE_GUIDE.md) for MANEUVER code.

For compiler code (Rust):
- Use `cargo fmt` for formatting
- Follow Rust standard style
- Add comments for complex logic
- Write descriptive commit messages

### Commit Messages

**Format:**
```
<type>: <short summary>

<optional body>

<optional footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting)
- `refactor`: Code refactoring
- `test`: Adding or updating tests
- `chore`: Maintenance tasks

**Example:**
```
feat: Add support for dynamic arrays

Implements Vec<T> type for dynamic-size arrays.
Includes automatic memory management and bounds checking.

Closes #42
```

### Documentation Style

- Use clear, concise language
- Include code examples
- Explain *why*, not just *what*
- Keep line length under 100 characters
- Use proper markdown formatting

---

## Community

### Communication Channels

- **Discord**: [Join our server](https://discord.gg/maneuver) for real-time chat
- **Forum**: [discuss.maneuver-lang.org](https://discuss.maneuver-lang.org) for discussions
- **GitHub Issues**: For bug reports and feature requests
- **Email**: hello@maneuver-lang.org

### Getting Help

- **New to open source?** Check out [How to Contribute to Open Source](https://opensource.guide/how-to-contribute/)
- **Questions about MANEUVER?** Ask on Discord or the forum
- **Stuck on something?** Tag maintainers in your PR or issue

### Recognition

Contributors are listed in:
- [CONTRIBUTORS.md](CONTRIBUTORS.md)
- Project website
- Release notes

Significant contributions may earn you:
- Collaborator status
- Recognition in talks/presentations
- Swag (coming soon!)

---

## Development Tips

### Working on the Compiler

```bash
# Quick compile check
cargo check

# Run specific test
cargo test test_name

# See what test does
cargo test test_name -- --nocapture

# Build with debug info
cargo build
```

### Working on Standard Library

Standard library is in `stdlib/` directory.

```bash
# Test stdlib
maneuver test stdlib/

# Build docs
maneuver doc stdlib/
```

### Working on Documentation

Documentation is in `docs/` directory.

```bash
# Build docs locally
cd docs
mdbook serve

# View at http://localhost:3000
```

---

## License

By contributing to MANEUVER, you agree that your contributions will be licensed under the [MIT License](LICENSE).

---

## Questions?

Don't hesitate to ask! We're here to help:
- Discord: #contributing channel
- Email: hello@maneuver-lang.org
- Forum: [discuss.maneuver-lang.org](https://discuss.maneuver-lang.org)

**Thank you for contributing to MANEUVER! 🚀**
