# GD32F303CC FOC Motor Control Project

## Overview
This project implements Field Oriented Control (FOC) for motor control on the GD32F303CC microcontroller. It provides a framework for high-performance motor control applications with ADC current sampling, PWM output, and sensor integration.

## Version
Current Version: v0.4.0
- I2C timeout and unlock path has been refactored to loop-budget based wait/recovery (no ms tick dependency in runtime path)
- Fault-state gating now blocks runtime sensor read chain (including I2C encoder reads) to avoid repeated timeout pressure
- Debug stream is now suppressed in FAULT state to reduce invalid-state output noise
- Protocol/parameter documentation has been synchronized with actual defaults and channel behavior
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
- Domain config convergence has moved to `foc_config.h` + `foc_cfg_*` split headers
- Algorithm trimming supports `FULL` build (runtime switchable) and explicit single-algorithm builds
- Undervoltage protection logic is integrated in FOC control flow (trip/recover thresholds + fault path), while platform actuator API remains trim-able and currently no-op on this hardware
- Multi-source communication aggregation has been moved to L3 `protocol_parser` with per-source platform APIs

## Features
- **FOC Open-loop Core**: Inverse Park + Inverse Clarke + SVPWM output path
- **Torque Control API**: Supports open-loop torque command and current-loop mode selection
- **Speed Control API**: Supports speed-loop cascade over torque/current loop
- **Startup Calibration**: Locked zero-angle sampling + stepped probe estimation for direction and pole pairs
- **ADC Sampling**: Synchronous current sampling for FOC (PA6/PA7 channels)
- **PWM Output**: 3-phase complementary PWM with dead time (TIMER0, 24kHz)
- **Sensor Integration**: AS5600 magnetic encoder via I2C
- **Timing Framework**: Multi-rate scheduling via control scheduler (1kHz/100Hz/200Hz/1Hz)
- **Trigger Chain**: TIMER2 master + TIMER3 compare trigger for ADC timing alignment
- **Communication**: USART1 + USART2 RX source aggregation with USART1 unified TX output (debug text + feedback byte)

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
├── foc/                 # Reusable single-motor FOC library (L1-L3 + config + API contract)
│   ├── include/         # Public headers and config headers
│   ├── src/             # Core control/runtime sources
│   └── port/            # Empty platform API template for new ports
├── examples/            # Platform-specific example projects
│   └── GD32F303_FOCExplore/
│       ├── hardware/    # Board-level docs and hardware integration notes
│       └── software/    # Standalone GD32 project (includes Firmware + Utilities)
├── docs/               # FOC library documents (library-only)
```

## Quick Start
1. Clone the repository
2. Open `examples/GD32F303_FOCExplore/software/Project.uvprojx` in Keil uVision
3. Or open the repo in VS Code + EIDE and target `examples/GD32F303_FOCExplore/software`
4. Build project
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
- [Development Guide](docs/development.md) - Development procedures and rules
- [Structure and Dependency Tree](docs/structure-and-dependency-tree.md) - Current folder tree and layered dependency map
- [Protocol and Runtime Parameters (Bilingual)](docs/protocol-parameters-bilingual.md) - Command protocol and configurable parameter set
- [Example Hardware Guide](examples/GD32F303_FOCExplore/hardware/hardware.md) - Pin mappings and board wiring for GD32F303_FOCExplore

## Contributing
- Follow the rules in `docs/engineering/dev-guidelines/`
- Use semantic versioning for releases
- Hardware validation required for all changes
- Update documentation for API changes

## License
[Add license information here]