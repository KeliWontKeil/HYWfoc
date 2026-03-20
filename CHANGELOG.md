# Changelog

All notable changes to the GD32F303CC FOC Motor Control Project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- (none yet)

### Fixed
- (none yet)

## [0.2.2] - 2026-03-20

### Changed
- Removed redundant 1kHz modulo check in timer callback dispatch path.
- Updated development guidance for where null checks are required versus redundant.

### Fixed
- Reworked I2C0 timeout handling so flag wait timeout now triggers internal bus recovery.
- Replaced unbounded STOP wait loops with timeout-protected wait logic.
- Corrected I2C unlock sequence to operate on both SCL/SDA in open-drain mode and generate a valid STOP.
- Removed misplaced application-level `I2C0_Unlock()` call from UART debug path; recovery is now centralized in I2C driver.

## [0.2.1] - 2026-03-18

### Added
- New SVPWM module under Application layer with six-sector normalized duty output API
- USART2 protocol-oriented send/read interfaces with frame assembly placeholders
- Temporary issue tracking document for planning the next mini-version

### Changed
- Main control loop now runs a 1kHz SVPWM simulation update path for waveform observation
- UART oscilloscope debug output switched to direct SVPWM three-phase duty telemetry
- ADC startup behavior improved by pre-filling initial sample buffer
- Development workflow rules updated to default delayed commits and main-branch development

### Fixed
- USART1 API type safety and const-correctness for byte/string send functions
- Initial SVPWM sector timing and duty normalization consistency

## [0.3.0] - 2026-03-13

### Added
- ADC current sampling implementation (PA6/PA7 synchronous sampling)
- DMA-based ADC data transfer
- Current calculation with zero offset calibration
- Multi-language development guidelines (English/Chinese)
- Comprehensive documentation reorganization
- Semantic versioning and branching strategy

### Changed
- Renamed `.cursor/` to `dev-guidelines/` for generalization
- Reorganized documentation structure under `docs/`
- Simplified main.c by removing test loops for production readiness
- Optimized ADC module error checking for embedded efficiency
- Updated development workflow to support AI-assisted development

### Fixed
- Compiler warnings (newline in header, implicit function declarations)
- Code style consistency across modules
- Resource usage optimization for embedded constraints

### Removed
- Test functions from main loop (moved to conditional compilation)
- Redundant error checking in time-critical paths

## [0.2.0] - 2026-03-11

### Added
- PWM dead time implementation
- Timer1 algorithm callback system
- Hardware I2C driver for GD32F30x
- AS5600 magnetic encoder driver
- LED blink callback encapsulation

### Changed
- Timer modules now use parameterized initialization
- Improved module boundaries and API consistency

## [0.1.0] - 2026-03-10

### Added
- Basic project structure with GD32F303CC
- Timer-based multi-rate scheduling framework
- USART1 interrupt-driven communication
- PWM output for 3-phase motor control
- LED status indication
- Initial development guidelines and rules

---

## Development Guide (Legacy)

### Scope
Single source of truth for AI work in this repository. Keep it compact and accurate.
Prefer referencing files over pasting long code blocks.

### Iteration policy
- **Current baseline**: **Iteration 3** (2026-03-11)
- **Process**:
  - The user writes *next iteration* requirements in `NEXT_MISSION.md`.
  - After completing an iteration, update:
    - `CHANGELOG.md` (version history and changes)
    - `dev-guidelines/rules/*` and `dev-guidelines/skills/*` (AI workflow)
    - `docs/hardware.md` only if pin mapping changes

### Project snapshot (v0.3.0)
- **MCU**: GD32F303CC (Cortex-M4)
- **Dev env**: VS Code / Cursor + EIDE extension (`cl.eide`)
- **Toolchain**: ARM Compiler 5 (AC5)
- **EIDE target**: `GD32F30X_CL`
- **Build outputs**: `build/GD32F30X_CL/` (e.g. `Project.axf`, `Project.hex`)

### What exists in code (high-signal)

- **Entry & init**
  - `Application/Source/main.c` initializes:
    - SysTick (1kHz)
    - LED GPIO
    - USART1 (interrupt-driven)
    - PWM (TIMER0 complementary, 3 channels)
    - Timer1 via `Timer1_Algorithm_Init()` (1kHz callback → algorithm scheduler)

- **Interrupt vectors**
  - `Application/Source/gd32f30x_it.c`:
    - SysTick → `delay_decrement()`
    - TIMER1/TIMER2/USART1 → forward to module-level `*_Internal()` handlers
  - This forwarding structure reflects the **current architecture**, not a permanent constraint. Structural refactors are allowed when required by the mission.

- **Timing / algorithm skeleton**
  - `Application/Source/timer1_algorithm.c`:
    - `Timer1_Algorithm_Init()` initializes TIMER1 with parameters
    - `Timer1_Algorithm_Handler()` provides multi-rate task slots (1kHz/100Hz/10Hz/1Hz)
    - DWT cycle counter is used for execution-time measurement
  - `Utilities/TIMER1/*`:
    - Generic TIMER1 module with parameterized initialization
  - `Utilities/TIMER2/*`:
    - Generic TIMER2 module with parameterized initialization

- **PWM**
  - `Utilities/PWM/*`:

### What exists in code (high-signal)

- **Entry & init**
  - `Application/Source/main.c` initializes:
    - SysTick (1kHz)
    - LED GPIO
    - USART1 (interrupt-driven)
    - PWM (TIMER0 complementary, 3 channels)
    - Timer1 via `Timer1_Algorithm_Init()` (1kHz callback → algorithm scheduler)

- **Interrupt vectors**
  - `Application/Source/gd32f30x_it.c`:
    - SysTick → `delay_decrement()`
    - TIMER1/TIMER2/USART1 → forward to module-level `*_Internal()` handlers
  - This forwarding structure reflects the **current architecture**, not a permanent constraint. Structural refactors are allowed when required by the mission.

- **Timing / algorithm skeleton**
  - `Application/Source/timer1_algorithm.c`:
    - `Timer1_Algorithm_Init()` initializes TIMER1 with parameters
    - `Timer1_Algorithm_Handler()` provides multi-rate task slots (1kHz/100Hz/10Hz/1Hz)
    - DWT cycle counter is used for execution-time measurement
  - `Utilities/TIMER1/*`:
    - Generic TIMER1 module with parameterized initialization
  - `Utilities/TIMER2/*`:
    - Generic TIMER2 module with parameterized initialization

- **PWM**
  - `Utilities/PWM/*`:
    - TIMER0 complementary PWM (3 channels)
    - Configuration parameters moved to initialization function

- **USART**
  - `Utilities/USART/*`:
    - USART1 interrupt-driven TX/RX with ring buffers
    - Optional loopback supported

- **LED**
  - `Utilities/LED/*`:
    - LED GPIO outputs. Pin mapping lives in `Hardware.md`

- **I2C**
  - `Utilities/I2C/*`:
    - Hardware I2C driver for GD32F30x (I2C0 on PB6/PB7)
    - Supports standard 100kHz communication
    - Includes byte and multi-byte read/write functions
    - Uses interrupt-based timeout handling with systick

- **AS5600**
  - `Utilities/AS5600/*`:
    - Complete driver for AS5600 magnetic encoder with I2C interface
    - Provides angle reading in degrees and radians
    - Includes magnet detection and strength checking
    - Supports configuration of start/stop positions and filtering

### Key changes in Iteration 2
1. **PWM dead time implementation** - Added proper `PWM_SetDeadTime()` function using GD32's `timer_break_config()` API
2. **Timer1 algorithm callback system** - Replaced static task functions with dynamic callback registration system (`Timer1_SetAlgorithmCallback()`)
3. **LED blink callback encapsulation** - Moved LED3 blink logic from `timer1_algorithm.c` to `main.c` as a callback function
4. **Hardware I2C driver** - Created complete I2C0 driver for GD32F30x with timeout handling and error checking
5. **AS5600 sensor driver** - Implemented full AS5600 magnetic encoder driver with I2C interface, magnet detection, and angle conversion

### AI guardrails (minimal)
- **Default do-not-edit**: `Firmware/**` (vendor libs), `.eide/**` (project config)
  - Exceptions: only when explicitly required by the mission or to fix build/flash issues
- **Prefer editing**: `Application/**`, `Utilities/**`, docs, `dev-guidelines/**`
- **Change scope**: decide per mission (small precision edits vs module implementation vs refactor)

### External reference (GD32 official examples)
GD32 official examples exist at:
`D:\GD32 相关\GD32F30x_Firmware_Library_V3.0.3\GD32F30x_Firmware_Library_V3.0.3\Examples`

Default policy: **do not read** unless necessary for a specific task (token-cost control)
