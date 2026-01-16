# MANEUVER Security Policy

## Reporting Security Vulnerabilities

### How to Report

**DO NOT** create public GitHub issues for security vulnerabilities.

Instead, email: **security@maneuver-lang.org**

Include:
- Description of the vulnerability
- Steps to reproduce
- Potential impact
- Suggested fix (if any)

### Response Timeline

- **24 hours**: Initial acknowledgment
- **7 days**: Assessment and triage
- **30 days**: Patch development (for confirmed vulnerabilities)
- **Public disclosure**: After patch release or 90 days (whichever comes first)

---

## Supported Versions

| Version | Supported |
|---------|-----------|
| 1.x.x   | ✅ |
| 0.x.x (beta) | ⚠️ Best effort |

---

## Security Features

### 1. Memory Safety

MANEUVER prevents entire classes of vulnerabilities:

✅ **Prevented:**
- Buffer overflows
- Use-after-free
- Double-free
- Null pointer dereferences
- Data races

```maneuver
// Safe by default
arr: [i32; 10] = create_array()
arr[100] = 42  // ✗ Bounds check prevents overflow
```

### 2. Type Safety

```maneuver
// No type confusion
user_id: UserId = UserId(42)
process_payment(user_id)  // ✗ Type error: expecting PaymentId
```

### 3. Input Validation

```maneuver
function handle_user_input(input: string where length ≤ 1000):
    // Length already validated by type system
    process(input)
```

### 4. Safe Unsafe Code

```maneuver
unsafe:
    justification: "Direct hardware register access needed for DMA"
    audit_required: true
    duration: 100μs
    
    write_register(DMA_ADDR, buffer_ptr)
```

---

## Security Best Practices

### 1. Validate All External Input

```maneuver
function parse_command(raw: string) -> Result<Command, Error>:
    // Validate before parsing
    if raw.length > MAX_COMMAND_LENGTH:
        return Err(Error.TooLong)
    
    // Use safe parser
    return safe_parse(raw)
```

### 2. Principle of Least Privilege

```maneuver
// Separate tasks with minimum necessary permissions
task sensor_reader:
    permissions: {Hardware.Read}
    
    data = read_sensor()
    send_to_processor(data)

task controller:
    permissions: {Hardware.Write}
    
    command = compute_control()
    send_to_actuator(command)
```

### 3. Secure Communication

```maneuver
robot distributed_system:
    communication:
        protocol: TLS 1.3
        authentication: mutual_cert
        encryption: AES-256-GCM
```

### 4. Audit Unsafe Code

```maneuver
// All unsafe blocks logged
unsafe:
    audit_id: "UNSAFE-2026-001"
    justification: "Performance-critical inner loop"
    reviewer: "security-team"
    review_date: 2026-01-15
    
    // Unsafe operations
```

### 5. Secrets Management

```maneuver
// Never hardcode secrets
❌ BAD:
api_key = "sk-1234567890abcdef"

✅ GOOD:
api_key = env.get_secret("API_KEY") or panic("API_KEY not set")
```

---

## Threat Model

### In Scope

1. **Memory corruption vulnerabilities**
2. **Type confusion attacks**
3. **Integer overflow/underflow**
4. **Timing attacks** (for cryptographic operations)
5. **Supply chain attacks** (dependency vulnerabilities)
6. **Injection attacks** (command injection, etc.)

### Out of Scope

1. **Physical attacks** (hardware tampering)
2. **Social engineering**
3. **DoS attacks** (resource exhaustion at system level)
4. **Side-channel attacks** (unless cryptographic primitives involved)

---

## Vulnerability Classes

### Critical (CVSS 9.0-10.0)

- Remote code execution
- Arbitrary memory write
- Authentication bypass
- Privilege escalation

**Response:** Immediate patch within 24-48 hours

### High (CVSS 7.0-8.9)

- Memory safety violations
- Information disclosure
- Denial of service (critical systems)

**Response:** Patch within 7 days

### Medium (CVSS 4.0-6.9)

- Input validation issues
- Moderate information disclosure
- Non-critical DoS

**Response:** Patch within 30 days

### Low (CVSS 0.1-3.9)

- Minor information leaks
- Edge case vulnerabilities
- Documentation issues

**Response:** Patch in next regular release

---

## Security Hardening

### Compiler Flags

```bash
# Enable all security features
maneuver build --security-hardened

# Specific flags
--stack-protector       # Stack canaries
--fortify-source        # Buffer overflow detection
--relro                 # Relocation read-only
--pie                   # Position independent executable
--sanitize=address      # AddressSanitizer
--sanitize=undefined    # UndefinedBehaviorSanitizer
```

### Runtime Protections

```maneuver
// Enable runtime security checks
#[security(
    stack_canary: true,
    aslr: true,
    dep: true,
    cfi: true  // Control Flow Integrity
)]
```

---

## Secure Coding Guidelines

### 1. Avoid Unsafe Code

```maneuver
// Only use unsafe when absolutely necessary
// And always justify it
unsafe:
    justification: "Required for FFI with C library"
    // Keep unsafe block minimal
```

### 2. Handle Errors Securely

```maneuver
// Don't leak sensitive info in errors
❌ BAD:
return Err("Login failed: User 'admin' not found")

✅ GOOD:
return Err("Invalid credentials")
```

### 3. Validate Before Use

```maneuver
function execute_command(cmd: string):
    requires: is_safe_command(cmd)
    
    // Validated before execution
    system.execute(cmd)
```

### 4. Clear Sensitive Data

```maneuver
function process_password(pwd: SecureString):
    // Use password
    authenticate(pwd)
    
    // Automatically cleared when out of scope
    // SecureString zeros memory on drop
```

### 5. Constant-Time Operations

```maneuver
function compare_hashes(a: Hash, b: Hash) -> bool:
    // Constant-time comparison prevents timing attacks
    return constant_time_eq(a, b)
```

---

## Dependency Security

### 1. Audit Dependencies

```bash
# Check for known vulnerabilities
maneuver audit

# Update vulnerable dependencies
maneuver update --security-only
```

### 2. Pin Versions

```toml
# Maneuver.toml
[dependencies]
robotics = "1.2.3"  # Exact version, not "^1.2.3"
```

### 3. Verify Checksums

```bash
# Verify package integrity
maneuver verify --checksums
```

---

## Incident Response

### If You Discover a Vulnerability

1. **Assess severity** using CVSS calculator
2. **Develop patch** in private branch
3. **Test thoroughly** (include regression tests)
4. **Coordinate disclosure** with security team
5. **Release patch** before public disclosure
6. **Publish advisory** with CVE ID

### If Vulnerability Reported to You

1. **Acknowledge receipt** within 24 hours
2. **Validate the issue** (can you reproduce it?)
3. **Assess impact** (who is affected?)
4. **Develop fix** collaboratively if needed
5. **Credit reporter** (unless they prefer anonymity)

---

## Security Resources

### Tools

- **maneuver audit** - Dependency vulnerability scanner
- **maneuver fuzz** - Fuzzing support
- **maneuver sanitize** - Runtime sanitizers
- **maneuver verify** - Formal verification

### Documentation

- [SAFETY.md](SAFETY.md) - Safety guidelines
- [TYPE_SYSTEM.md](TYPE_SYSTEM.md) - Type safety details
- [FORMAL_VERIFICATION.md](docs/formal-verification.md) - Verification guide

### External Resources

- [CVE Database](https://cve.mitre.org/)
- [NVD](https://nvd.nist.gov/)
- [OWASP Top 10](https://owasp.org/www-project-top-ten/)

---

## Hall of Fame

Contributors who responsibly disclosed security vulnerabilities:

<!-- Will be populated as vulnerabilities are found and fixed -->

---

## Contact

- **Security Email:** security@maneuver-lang.org
- **PGP Key:** [Download](https://maneuver-lang.org/pgp-key.asc)
- **Bug Bounty:** Coming soon

---

## License

This security policy is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
