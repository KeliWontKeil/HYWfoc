# Development Guide

## Development Environment Setup

### Required Tools
- **Keil μVision 5**: Primary build toolchain
- **VS Code + EIDE Extension**: Development environment
- **ST-LINK Utility**: Firmware flashing
- **Git**: Version control
- **J-Link Commander**: Advanced debugging

### Project Configuration
- Target: GD32F30X_CL
- Compiler: ARM Compiler 5 (AC5)
- Optimization: Balance speed and size
- Debug: ST-LINK with SWD interface
- Control tick source: TIMER1 update interrupt at 1kHz (bound via `FOC_Platform_SetControlTickCallback`)
- Scheduler slots: 1kHz fast-control, 100Hz service, 200Hz monitor, 1Hz heartbeat
- PWM base: TIMER0 center-aligned output synchronized by TIMER2 (24kHz)
- ADC trigger: TIMER3 compare event

### Control Period Contract
- Open-loop speed conversion in `FOC_OpenLoopStep` uses `FOC_CONTROL_DT_SEC` as control period source (defined in config headers).
- If control scheduler frequency is changed, keep `FOC_CONTROL_DT_SEC` and scheduler configuration synchronized.

## Development Workflow

### 1. Planning Phase
- Define requirements in `NEXT_MISSION.md`
- Identify affected modules
- Estimate resource impact (ROM/RAM)

### 2. Implementation Phase
- Follow coding rules in `docs/engineering/dev-guidelines/rules/`
- Use appropriate language version (en/cn)
- Default to direct implementation on `main` unless a dedicated branch is explicitly requested
- Regular commits with descriptive messages

### 3. Testing Phase
- Build with zero warnings
- Unit test utilities where possible
- Hardware validation mandatory
- Performance profiling

### 4. Documentation Phase
- Update module documentation
- Add API comments (Doxygen format)
- Update architecture diagrams if needed

### 5. Release Phase
- Code review
- Merge to main branch
- Create version tag
- Update changelog

## Coding Standards

### Dependency Layer Contract
- L1 Application layer: only externally callable app APIs (for `main`).
- L2 Algorithm layer: FOC algorithm/task/status/runtime management logic.
- L3 Advanced peripheral layer: convert low-level raw data/interfaces into structured interfaces (sensor/SVPWM/protocol parsing).
- L4 Peripheral layer: chip-specific Utilities drivers only.
- Special dependency layer: unified upper API, shared structs, parameter/config macros, algorithm feature-cut macros.
- **Config Convergence Rule**: See [copilot-instructions.md](../copilot-instructions.md#mandatory-dependency-contract) for single-source config principle. All runtime defaults, command parameters, and range limits belong in `foc_cfg_*.h` headers, never scattered in .c files.
- Control algorithm trimming rule: `speed-only` and `speed-angle` are parallel algorithms. Default build should keep both (`FULL`) and allow runtime mode switch; single-algorithm build is an explicit trimming option only.
- Domain layout now includes: command/protocol/debug/task/diag + range/platform/app/sensor/svpwm.
- Mandatory rule: L1/L2/L3 can call L4 only through special dependency layer.
- Mandatory rule: L4 must not depend on L1/L2/L3.

### Language Rules
- C99 with ARM extensions
- Include `<stddef.h>` for NULL
- Use fixed-point math when possible
- Minimize floating-point operations

### Naming Conventions
**See** [copilot-instructions.md - Naming & Code Style](../copilot-instructions.md#naming--code-style) for the authoritative definitions:
- Functions: `ModuleName_FunctionName`
- Variables: `camelCase`
- Macros: `UPPER_CASE`
- Types: `type_name_t`
- Structs: `Module_Struct`

### Code Organization
- One function per responsibility
- Header files for APIs
- For non-self-contained modules, place primary dependency includes in `.h` to keep dependency structure visible
- Keep `.c` includes minimal; only add `.c`-local includes for cyclic-dependency or implementation-private needs
- Static functions for internal use
- Consistent indentation (4 spaces)
- Keep control-layer boundaries explicit: `foc_control.c` for runtime control algorithms, `foc_control_init.c` for startup/init calibration only
- Keep pure scalar math utilities (`wrap/clamp/pi constants`) in `math_transforms` rather than control modules
- New module design must explicitly declare layer ownership in header comments or module docs
- Public headers at L2/L3 should avoid exposing L4 driver headers
- Communication path rule: per-source read/trigger APIs are exposed by platform layer, while multi-source aggregation policy belongs to L3 `protocol_parser`.

## Debugging Procedures

### Hardware Debugging
1. Connect ST-LINK to target
2. Set breakpoints in critical paths
3. Monitor execution time with DWT
4. Check peripheral registers
5. Validate timing with oscilloscope

### Common Issues
- **Stack overflow**: Check stack usage in map file
- **Timing violations**: Profile with DWT counters
- **Memory corruption**: Use memory watchpoints
- **Interrupt conflicts**: Check priority levels

### I2C Recovery Strategy
- Keep bus recovery inside the I2C driver, not in application/debug call sites.
- Runtime timeout/wait strategy should use bounded loop-budget counters and avoid millisecond blocking in control-critical paths.
- Any I2C flag wait timeout should trigger immediate peripheral reconfiguration and bus unlock sequence.
- STOP wait loops must always have timeout protection; do not use unbounded `while (STOP)` loops.
- External modules should only handle returned status codes (`I2C_OK`, `I2C_TIMEOUT`, `I2C_NACK`, `I2C_ERROR`).

### Fault-State Runtime Policy
- Control loop should early-return in FAULT state and skip runtime sensor acquisition path.
- Debug stream should be gated off in FAULT state; only command/recovery path is kept active.
- Fault recovery should use command path (`F+C`) and then revalidate sensor/control chain.

### Redundant Checks Policy
- Remove always-true or duplicate runtime checks from high-frequency paths.
- Keep null/parameter checks only at public API boundaries or when data source is uncertain.
- For internal control-loop helpers, prefer clear contracts over repeated defensive checks.

## Performance Optimization

### Profiling Techniques
- Use `ControlScheduler_EnableDWT()` for cycle counting
- Check `ControlScheduler_GetExecutionCycles()` in tasks
- Monitor stack usage
- Analyze disassembly for bottlenecks

### Optimization Strategies
- Prefer table lookups over calculations
- Use DMA for data movement
- Minimize interrupt latency
- Optimize data structures

## Version Control

### Branch Strategy
```
main (default development and releases)
├── feature/* (optional, when explicitly required)
└── hotfix/* (emergency fixes)
```

### Commit Guidelines
- Use conventional commits: `type(scope): description`
- Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`
- Keep commits focused and atomic

## Quality Assurance

### Code Reviews
- Check adherence to coding standards
- Verify resource usage
- Validate error handling
- Confirm documentation updates

### Testing Requirements
- Build verification (zero warnings)
- Hardware functionality tests
- Performance benchmarks
- Regression testing

## Troubleshooting

### Build Issues
- Clean and rebuild project
- Check include paths
- Verify toolchain installation
- Update project dependencies

### Runtime Issues
- Check power supply stability
- Verify hardware connections
- Monitor for noise/interference
- Use logic analyzer for timing

### Tool Integration
- VS Code settings in `.vscode/`
- EIDE configuration files
- Git hooks for quality checks