# AI Project Initialization Template

## Context for AI Assistant

You are working on a GD32F303CC microcontroller project with the following setup:

### Project Structure
- **MCU**: GD32F303CC (ARM Cortex-M4, 120MHz)
- **Framework**: Pre-configured with basic peripherals
- **Development**: VS Code + EIDE extension, Keil μVision 5
- **Version Control**: Git with semantic versioning

### Current Framework State
The project has been initialized with core modules:
- System tick (1kHz)
- LED GPIO control
- USART1 serial communication
- PWM generation (TIMER0, 3 channels)
- Timer-based algorithm scheduling (1kHz/100Hz/10Hz/1Hz slots)
- I2C communication interface

### Development Rules
Located in `docs/engineering/dev-guidelines/rules/`:
- `en/` and `cn/` versions available
- Follow project-specific rules for GD32 development
- Use embedded-general rules for ARM Cortex-M best practices
- Adhere to development-workflow for AI-assisted coding

### Key Guidelines
1. **Code Style**: Follow naming conventions (Module_FunctionName, UPPERCASE_MACROS)
2. **Safety**: Include `<stddef.h>` for NULL, avoid blocking operations in ISRs
3. **Resources**: Optimize for embedded constraints (ROM/RAM efficiency)
4. **Testing**: Hardware validation required, use ST-LINK for debugging

### Documentation
- `docs/README.md`: Project overview and quick start
- `docs/architecture.md`: System design and module relationships
- `examples/GD32F303_FOCExplore/hardware/hardware.md`: Pin mappings and connections
- `docs/development.md`: Development procedures and rules

### Workflow for New Features
1. Read relevant documentation and rules
2. Implement in feature branches
3. Test on hardware with ST-LINK
4. Update documentation
5. Merge via pull request

### Common Tasks
- Add new peripheral drivers in `examples/GD32F303_FOCExplore/software/Utilities/`
- Implement control algorithms in timer task slots
- Add communication protocols
- Integrate sensors and actuators

Remember: This is an embedded system - efficiency, reliability, and hardware validation are critical.