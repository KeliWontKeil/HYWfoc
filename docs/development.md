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
- Control scheduler: TIMER1 update interrupt at 1kHz
- PWM base: TIMER0 center-aligned output synchronized by TIMER2 (24kHz)
- ADC trigger: TIMER3 compare event

### Control Period Contract
- Open-loop speed conversion in `FOC_OpenLoopStep` currently assumes a 1ms control period.
- If control scheduler tick frequency is changed, update speed-to-angle conversion accordingly to keep speed command consistent.

## Development Workflow

### 1. Planning Phase
- Define requirements in `NEXT_MISSION.md`
- Identify affected modules
- Estimate resource impact (ROM/RAM)

### 2. Implementation Phase
- Follow coding rules in `dev-guidelines/rules/`
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

### Language Rules
- C99 with ARM extensions
- Include `<stddef.h>` for NULL
- Use fixed-point math when possible
- Minimize floating-point operations

### Naming Conventions
- Functions: `ModuleName_FunctionName`
- Variables: `camelCase`
- Macros: `UPPER_CASE`
- Types: `type_name_t`

### Code Organization
- One function per responsibility
- Header files for APIs
- Static functions for internal use
- Consistent indentation (4 spaces)

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
- Any I2C flag wait timeout should trigger immediate peripheral reconfiguration and bus unlock sequence.
- STOP wait loops must always have timeout protection; do not use unbounded `while (STOP)` loops.
- External modules should only handle returned status codes (`I2C_OK`, `I2C_TIMEOUT`, `I2C_NACK`, `I2C_ERROR`).

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