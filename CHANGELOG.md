# Changelog

All notable changes to the GD32F303CC FOC Motor Control Project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
- (none yet)

### Fixed
- (none yet)

## [0.3.0] - 2026-03-23

### Changed
- Completed low-speed sensored FOC functional path for this stage: startup calibration, torque/current control entry, position/speed loop framework, and TIMER2-driven SVPWM interpolation.
- Refactored speed-loop implementation to follow position-loop principle by integrating speed reference (rad/s) into accumulated angle reference per 1kHz cycle.
- Unified direction semantics to signed `1/0/-1` across control APIs and internal calculations.
- Consolidated control dataflow around torque/current loop reuse to reduce duplicated logic between outer loops.

### Fixed
- Corrected direction mapping inconsistencies that could cause effective reverse behavior in some paths.
- Removed unstable direct speed-PID path and cleaned obsolete speed-state fields/code.

### Notes
- Current precise current control performance is still the primary remaining gap for the next mini-version.
- Build and flash verification passed in hardware validation for this release.

## [0.2.7] - 2026-03-23

### Changed
- Added SVPWM linear interpolation path driven by TIMER2 update interrupt callback to improve duty-cycle continuity.
- Added current-loop feedforward + PID implementation on `iq` channel with low-current PID bypass (`|iq_ref| < 0.1A`).
- Added speed-loop API (`FOC_SpeedControlStep`) as cascade outer loop over torque/current loop.
- Updated main control integration with speed-loop PID initialization and runtime entry.

### Fixed
- Unified motor direction semantics to signed `1/0/-1` across definitions, APIs, and direction-related calculations.
- Corrected direction-mapping inconsistency that caused effective sign inversion in some control paths.

### Notes
- Current hardware validation shows improved behavior with TIMER2-driven interpolation compared with prior interrupt path.
- Speed-loop gains are initialized with conservative defaults and may require hardware tuning.

## [0.2.6] - 2026-03-23

### Changed
- Added position-loop framework on top of torque control: introduced `foc_angle_loop_t` and `FOC_AngleControlStep()` to generate torque-current reference from angle error.
- Added accumulated mechanical position state in motor model (`mech_angle_accum_rad`) with wrap-aware incremental update for multi-turn position control.
- Updated main control integration with angle-loop PID initialization and retained torque-loop runtime as default path for staged validation.
- Updated next-mission direction to focus on control-effect optimization (current-loop feedforward PID and low-current open-loop fallback).

### Fixed
- Improved startup calibration flow ordering to avoid angle-state discontinuity by finalizing zero-angle lock after direction/pole-pair estimation.

### Notes
- Basic FOC functional verification is completed in current hardware tests.
- Next mini-version will focus on control performance optimization and parameter robustness.

## [0.2.5] - 2026-03-21

### Changed
- Added torque-control APIs in `foc_control` with mode switch support (open-loop and current-loop mode).
- Refactored control path to absolute-voltage semantics: `vbus_voltage` as global limit, `set_voltage` as user clamp, `ud/uq` as absolute voltage commands.
- Updated SVPWM update interface to use explicit `voltage_command` amplitude input.
- Added mechanical-angle to electrical-angle mapping in FOC based on direction, pole-pairs, and calibrated zero electrical reference.
- Decoupled sensor acquisition from control algorithm by passing measured current and mechanical angle into FOC control APIs.

### Fixed
- Corrected torque-axis command routing so torque command is applied on q-axis in control path.
- Fixed open-loop torque branch polarity/axis assignment issue in control-step implementation.
- Reduced electrical-angle wrap ambiguity by applying modulo-based single-electrical-cycle mapping in mechanical-to-electrical conversion.

### Notes
- Open-loop torque control is validated in current hardware tests.
- Closed-loop behavior is logically correct but still requires parameter tuning for final dynamic performance.

## [0.2.4] - 2026-03-21

### Changed
- Refactored FOC motor model naming to explicit rad-based semantics (`electrical_phase_angle`, `mech_angle_at_elec_zero_rad`).
- Unified encoder angle processing and UART debug output to radians across Sensor/AS5600/FOC paths.
- Added startup calibration entry `FOC_CalibrateElectricalAngleAndDirection()` and invoked it at the end of `FOC_MotorInit()`.
- Reworked zero electrical angle measurement to locked static sampling at electrical angle 0, with settle and multi-sample circular averaging.
- Reworked direction and pole-pair estimation to one-way stepped electrical-angle sampling with larger total probe span.

### Fixed
- Removed mechanical/electrical angle mixed-domain operations from calibration flow.
- Corrected open-loop output to use tracked electrical phase command instead of fixed zero-angle forcing.
- Calibration now updates zero angle, direction, and pole pairs only when each field is undefined.

## [0.2.3] - 2026-03-20

### Changed
- Removed redundant 1kHz modulo check in timer callback dispatch path.
- Updated development guidance for where null checks are required versus redundant.
- Refactored open-loop FOC motor model around simplified `foc_motor_t` parameter set.
- Updated SVPWM init API to `SVPWM_Init(freq_kHz, deadtime_percent)` and passed bus voltage explicitly in `SVPWM_Update`.
- Updated `main.c` control path to use current open-loop call pattern (`FOC_OpenLoopStep(&g_motor, turn_speed_hz)`).
- Adjusted ADC sample trigger offset from 94% to 96%.

### Fixed
- Reworked I2C0 timeout handling so flag wait timeout now triggers internal bus recovery.
- Replaced unbounded STOP wait loops with timeout-protected wait logic.
- Corrected I2C unlock sequence to operate on both SCL/SDA in open-drain mode and generate a valid STOP.
- Removed misplaced application-level `I2C0_Unlock()` call from UART debug path; recovery is now centralized in I2C driver.
- Added neutral-current compensation step in sensor current processing path.

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

## [0.3.0-legacy] - 2026-03-13

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
