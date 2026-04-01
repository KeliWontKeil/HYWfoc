# GD32F303CC FOC Motor Control Project

## Overview
This project implements Field Oriented Control (FOC) for motor control on the GD32F303CC microcontroller. It provides a framework for high-performance motor control applications with ADC current sampling, PWM output, and sensor integration.

## Version
Current Version: v0.3.6
- Framework initialization with basic peripherals
- Dual-path debug stream module for monitoring (semantic low-rate + osc high-rate)
- Command manager and runtime state/diagnostic module are integrated
- Protocol parser + command execution chain is integrated (frame parse -> cache -> dispatch)
- Radian-unit unification across encoder, sensor filter, and FOC math path
- Startup calibration for zero electrical angle, direction, and pole pairs
- Torque-control API path with open-loop mode available and closed-loop mode ready for tuning
- Position-loop framework added (angle PID -> torque current reference)
- Speed-loop refactored to position-style cascade (speed reference integration -> angle PID -> torque current reference)
- SVPWM linear interpolation enabled by high-rate TIMER2 callback path for smoother voltage output
- Four-level dependency refactor completed for core control path (`main` -> `foc_app` -> logic/platform API -> utilities)
- `foc_control` responsibilities split: control algorithms remain in `foc_control`, while motor initialization/calibration moved into `foc_control_init`
- Math scalar helpers and constants are unified in `math_transforms` (`Math_WrapRad`, `Math_WrapRadDelta`, `Math_ClampFloat`, `MATH_TWO_PI`)
- Scheduler ownership finalized: app layer owns scheduler task registration; platform layer only owns control-tick source binding
- Shared cross-module public types centralized in `foc_shared_types.h`
- IRQ forwarding now uses direct ISR-to-internal-handler calls in `gd32f30x_it.c`
- Low-speed sensored FOC core functionality is completed for current stage (except precise current-control performance tuning)
- Scheduler task-rate API has been standardized to `FOC_TaskRate_t` with centralized divider configuration
- Domain config convergence is in progress through `foc_config.h` and split config headers
- Algorithm trimming supports `FULL` build (runtime switchable) and explicit single-algorithm builds
- Undervoltage protection logic is integrated in FOC control flow (trip/recover thresholds + fault path), while platform actuator API remains trim-able and currently no-op on this hardware

## Features
- **FOC Open-loop Core**: Inverse Park + Inverse Clarke + SVPWM output path
- **Torque Control API**: Supports open-loop torque command and current-loop mode selection
- **Speed Control API**: Supports speed-loop cascade over torque/current loop
- **Startup Calibration**: Locked zero-angle sampling + stepped probe estimation for direction and pole pairs
- **ADC Sampling**: Synchronous current sampling for FOC (PA6/PA7 channels)
- **PWM Output**: 3-phase complementary PWM with dead time (TIMER0, 24kHz)
- **Sensor Integration**: AS5600 magnetic encoder via I2C
- **Timing Framework**: Multi-rate scheduling via control scheduler (1kHz/100Hz/10Hz/1Hz)
- **Trigger Chain**: TIMER2 master + TIMER3 compare trigger for ADC timing alignment
- **Communication**: USART1 + USART2 with DMA TX, DMA RX + IDLE callback, and frame mux dispatch path

## Hardware Requirements
- GD32F303CC microcontroller
- ST-LINK debugger
- Current sensors (0-20A range, midpoint reference)
- AS5600 magnetic encoder
- Motor driver hardware

## Development Environment
- **Primary IDE**: Keil μVision 5 with ARM Compiler 5
- **Development IDE**: VS Code with EIDE extension
- **Debugger**: ST-LINK V2/V3
- **Version Control**: Git with semantic versioning

## Project Structure
```
├── Application/          # Main application code
│   ├── Include/         # Application headers
│   └── Source/          # Application sources
├── Utilities/           # Peripheral drivers
│   ├── ADC/            # Current sampling
│   ├── PWM/            # PWM generation
│   ├── USART/          # Serial communication
│   └── ...             # Other modules
├── Firmware/           # Vendor libraries (read-only)
├── docs/               # Documentation
├── dev-guidelines/     # Development rules and skills
├── build/              # Build artifacts (e.g. build/GD32F30X_CL/Project.hex)
```

## Quick Start
1. Clone the repository
2. Open in VS Code with EIDE extension
3. Build project using EIDE: Build command
4. Flash using ST-LINK
5. Monitor via USART1 (115200 baud)

## Development Workflow
This project uses AI-assisted development with manual hardware validation:

1. Define requirements in `NEXT_MISSION.md`
2. AI generates implementation code
3. Manual code review and hardware testing
4. Update documentation
5. Create version tag and release

## Documentation
- [Architecture](docs/architecture.md) - System design and module relationships
- [Hardware](docs/hardware.md) - Pin mappings and connections
- [Development Guide](docs/development.md) - Development procedures and rules
- [Structure and Dependency Tree](docs/structure-and-dependency-tree.md) - Current folder tree and layered dependency map
- [Protocol and Runtime Parameters (Bilingual)](docs/protocol-parameters-bilingual.md) - Command protocol and configurable parameter set

## Contributing
- Follow the rules in `dev-guidelines/`
- Use semantic versioning for releases
- Hardware validation required for all changes
- Update documentation for API changes

## License
[Add license information here]