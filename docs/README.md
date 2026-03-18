# GD32F303CC FOC Motor Control Project

## Overview
This project implements Field Oriented Control (FOC) for motor control on the GD32F303CC microcontroller. It provides a framework for high-performance motor control applications with ADC current sampling, PWM output, and sensor integration.

## Version
Current Version: v0.2.1
- Framework initialization with basic peripherals
- UART debug module for monitoring
- Code style standardization (K&R brace style)

## Features
- **ADC Sampling**: Synchronous current sampling for FOC (PA6/PA7 channels)
- **PWM Output**: 3-phase complementary PWM with dead time
- **Sensor Integration**: AS5600 magnetic encoder via I2C
- **Timing Framework**: Multi-rate task scheduling (1kHz/100Hz/10Hz/1Hz)
- **Communication**: USART1 for debugging and monitoring

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

## Contributing
- Follow the rules in `dev-guidelines/`
- Use semantic versioning for releases
- Hardware validation required for all changes
- Update documentation for API changes

## License
[Add license information here]