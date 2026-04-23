# Changelog

All notable changes to the HYWfoc (何易位FOC) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.4.0] - 2026-04-23
### Changed
- 修复部分状态机BUG，恢复电压限幅逻辑
- 调整状态机组织结构，修复LED闪烁不正常以及错误恢复不正常的BUG
- 完善欠压保护功能，监测VBUS电压并在过低时进入FAULT状态
- 大量文档未更新

## [1.3.6] - 2026-04-21

### Changed
- Completed structure-convergence closure as the current baseline: L2 production runtime pipeline is fixed at `runtime_c1_entry -> runtime_c2_frame_source -> runtime_c3_runtime_fsm -> runtime_c4_runtime_core -> runtime_c5_output_adapter`.
- Aligned cogging compile guards in `foc_control_c24_compensation`: when cogging feature is trimmed off, runtime call path now exits before referencing feature-scoped helper functions.
- Extended protocol state text mapping with cogging state entry (`S:G`) under `FOC_PROTOCOL_ENABLE_COGGING_COMP` guard.

### Documentation
- Rewrote `docs/architecture.md` as current-code SSOT using actual `L1/L2/L3/L41/L42/LS` paths and current control/runtime chain naming.
- Rewrote `docs/development.md` to remove outdated temporary-plan and legacy numbering references, and synced it with current compile/macro-governance rules.
- Updated protocol bilingual guide defaults/ranges/path references to current `foc_cfg_*` values and documented current cogging protocol-state scope.
- Synced mission/workflow docs (`NEXT_MISSION.md`, `.github/AGENTS.md`, agent docs, instance software README) to current directory structure and version baseline.

## [1.3.4] - 2026-04-17

### Changed
- Completed P1 architecture-prep inventory and consolidated file-level boundary facts (control/protocol split, config ownership, platform API minimal set, timing impact) into the SSOT architecture document.
- Completed P2 structure-clarification pass for runtime interrupt lifecycle: application init now explicitly keeps control runtime IRQs disabled until runtime start, preserving init/runtime execution-path separation.
- Added compile-time timing constraints for PWM/current-loop divider legality to prevent invalid non-integer or zero-divider configurations.
- Completed physical directory convergence for L1/L2/L4-1 source layout: runtime orchestration/service modules now build from `core`/`service`, and math/LUT modules build from dedicated `math` directories.
- Split `command_manager` into focused submodules (`command_manager_dispatch`, `command_manager_diag`) and split `foc_control` auxiliary strategies into dedicated modules (`foc_control_softswitch`, `foc_control_compensation`).
- Synced EIDE project sources and builder source list with the refactored module paths and newly introduced source files.
- Completed trig LUT unification for init calibration path in `foc_control_init`: replaced direct trig-library usage with `FOC_MathLut_Sin` / `FOC_MathLut_Atan2` to align init/runtime math behavior.
- Fixed state-to-control propagation after L2/L3 decoupling: writing current soft-switch enable state now marks runtime params dirty so L1 applies it in unified runtime config refresh.

### Documentation
- Added P3 regression-prep records in `docs/development.md`, including build result, resource usage summary, startup calibration path check, and runtime SVPWM interpolation path check.
- Updated `NEXT_MISSION.md` to mark P1/P2/P3 as completed and recorded this cycle's closure summary.
- Updated `docs/architecture.md` structure tree and layer mapping to reflect `math` directory extraction and service/algorithm internal module splits.

## [1.3.2] - 2026-04-15

### Changed
- 将结构/依赖文档收敛到单一事实源 `docs/architecture.md`，并删除重复的结构文档入口，避免兼容式跳转维护。
- 重写并中文化仓库流程治理文档，统一边界口径：`AI_INITIALIZATION.md`、`copilot-instructions.md`、`.github/AGENTS.md`、`.github/WORKFLOW_CHECKLIST.md`、`.github/DOCUMENTATION_STRUCTURE.md`。
- 重构 `NEXT_MISSION.md` 为面向结构重排的分阶段任务单，并将"待确认问题"固化为"已确认决议"。
- 清理失效文档入口与过期工作流引用，修复 `docs/README.md`、`.github/*`、`README.md` 的断链与版本口径。

### Documentation
- 在 `docs/development.md` 增补"编译/调试经验沉淀"条目，并与仓库长期记忆保持同措辞同步。

## [1.3.1] - 2026-04-14

### Added
- Added optional cogging compensation framework with feature switches, runtime status tracking, LUT load API, and low-speed lookup injection in speed/speed-angle outer-loop paths.
- Added startup cogging source selection logic: static profile first, init-time learning fallback, and no-source graceful disable path with diagnostics.
- Added init-time cogging learning routine with finer lock sampling, table quantization, and structured debug dump output.

### Changed
- Refined motor init calibration stepping strategy to use finer electrical-angle subdivision for direction/pole-pair estimation.
- Added new compile-limit checks for cogging-related feature switches and init-calibration trimming constraints.

## [1.3.0] - 2026-04-13

### Changed
- Reformatted current-loop anti-noise and soft-switch config macros in `foc_cfg_init_values.h` with aligned value columns for easier scanning.
- Added concise inline `//` comments for soft-switch defaults, anti-noise deadband/integral/Ki-scaling macros, and blend time constants.
- Added explicit `FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_MIN_SEC` default definition for soft-switch blend lower-bound configuration.

### Documentation
- Updated repository baseline/version references to `v1.3.0` and moved active mission target to `v1.3.1`.
- Updated protocol documentation to describe both soft-switch blend time-constant macros (`TAU_MIN` and `TAU_DEFAULT`).

## [1.2.0] - 2026-04-11

### Changed
- Completed high-rate execution-path refactor: PWM update ISR callback is now the canonical interpolation/current-loop path and protocol parsing is fully polling-based (`IsFrameReady + ReadFrame`).
- Completed outer/inner loop decoupling baseline: outer loop stays on scheduler task path, while current-loop execution is driven by PWM ISR cadence.
- Updated ADC sampling pipeline to support distinct slow/fast averaging windows and moved average-count boundary normalization into ADC driver layer.
- Added 16-bit trig lookup module (`0.001 rad` step, symmetry reconstruction) under special dependency layer and switched SVPWM trigonometric hotspots to LUT path.
- Improved macro clipping semantics for current-loop disable mode: ISR keeps interpolation-only behavior and skips fast-loop compute chain.
- Added optional first-order `iq` LPF switch/alpha configuration in current-loop path for noise suppression experiments.
- Cleaned instance platform API adapter responsibility by removing non-adapter helper logic from `foc_platform_api`.

### Documentation
- Updated release baseline references and mission document targets to align with `v1.2.0` snapshot delivery.

## [1.1.0] - 2026-04-10

### Changed
- Added independent speed reference parameters for speed-only and speed-angle control paths, including dedicated subcommand mapping and runtime getters.
- Updated speed-only runtime path to consume the dedicated speed-only reference value instead of reusing the speed-angle parameter.
- Added circular-domain angle LPF processing path in sensor readout and kept it behind compile-time feature switches.
- Unified sensor filter compile-time trimming structure so declaration/call/definition guards are aligned for Kalman and angle LPF paths.
- Updated sensing defaults to use angle LPF while keeping current and angle Kalman paths disabled by default for the current hardware baseline.
- Expanded command parameter compile limits and defaults to include signed speed-only range and related initialization values.

### Documentation
- Updated protocol bilingual parameter documentation for speed-only and speed-angle subcommand mapping and examples.
- Updated mission planning document to set v1.1.0 as baseline and define next-stage algorithm priorities (cogging feedforward and current-loop tuning).
- Refreshed project/hardware readme descriptions for current repository and instance status.

## [1.0.0] - 2026-04-09

### Changed
- Extended command frame format to `a<driver_id><cmd><subcmd><param>b` with protocol-level address filtering.
- Added driver-id validation range (`0x32-0x7E`) and broadcast support (`0xFF`); non-targeted valid frames are silently dropped.
- Kept instance platform timing and indicator constants as local macros in platform implementation to avoid direct dependency on library config headers.
- Decoupled instance I2C driver from `foc_config.h` by using local timeout and unlock loop macros in `i2c0`.

### Documentation
- Updated protocol bilingual guide and GD32F303 instance adaptation examples to the new driver-id command format.
- Reworked root `README.md` as external-facing Chinese project introduction with unified naming baseline (HYWfoc / 何易位FOC).
- Reworked instance `examples/GD32F303_FOCExplore/README.md` into platform usage guide and aligned links to detailed instance docs.
- Updated `docs/README.md` and `NEXT_MISSION.md` (P3.5) for document-role boundary and naming/task consistency.
- Updated repository workflow and initialization docs to repository-level governance (`AI_INITIALIZATION.md`, `copilot-instructions.md`, `docs/development.md`, and workflow rules in `docs/engineering/dev-guidelines/rules/`).
- Added third-party dependency and license notice inventory in `THIRD_PARTY_NOTICES.md`.

## [0.4.0] - 2026-04-03

### Changed
- Reworked I2C wait-timeout strategy to loop-budget polling in `i2c0` driver path, removing runtime dependency on millisecond tick waits for flag/STOP handling.
- Reworked I2C unlock timing to bounded busy-loop delays (non-DWT) for SCL pulse and STOP generation path.
- Added control-loop FAULT-state early gate in app runtime path to skip sensor refresh and avoid repeated encoder I2C access in invalid hardware states.
- Added debug-stream FAULT-state gate to suppress semantic/osc periodic output while fault remains active.
- Kept fault recovery path on command channel (`F+C`) for runtime reinit without power cycle.

### Documentation
- Updated root/docs version baseline to `v0.4.0`.
- Aligned scheduler-rate descriptions to `1kHz/100Hz/200Hz/1Hz`.
- Corrected protocol document channel mapping to current implementation (`USART1` unified TX feedback + debug text).
- Synced protocol parameter defaults with code (`angle_speed_rad_s`, semantic enable default, speed PID defaults).
- Rewrote mission plan for next iteration as architecture/organization/documentation-only scope (no business-logic changes).

### Added
- Added trim-able undervoltage protection feature switch (`FOC_FEATURE_UNDERVOLTAGE_PROTECTION`) in startup config.
- Added platform placeholder API `FOC_Platform_UndervoltageProtect(vbus_voltage)` with no-op implementation for current hardware.
- Added FOC-side undervoltage fault logic path in control loop with trip/recover thresholds and fault-state propagation via command manager.

### Documentation
- Updated project README to describe the undervoltage protection placeholder path and current hardware limitation.

## [0.3.8] - 2026-04-02

### Changed
- Completed communication path refactor: multi-source frame aggregation is now implemented in L3 `protocol_parser` with trigger-source-priority behavior.
- Reworked platform communication APIs in `foc_platform_api` to one global init plus four explicit source read/trigger interfaces.
- Reorganized configuration architecture into the `foc_cfg_*` split (`symbol_defs`, `feature_switches`, `init_values`, `compile_limits`) and converged includes through `foc_config.h`.
- Unified scheduler/control timing semantics using dedicated control-rate macros and divider-based callback gating.
- Unified project-wide mathematical constants into `foc_shared_types.h` and aligned control/SVPWM/sensor math usage.

### Removed
- Removed legacy L4 communication multiplexer module (`Utilities/USART/comm_frame_mux.[ch]`).
- Removed deprecated legacy `foc_config_*` header set in favor of the `foc_cfg_*` split.

### Documentation
- Updated architecture and dependency-tree documents to reflect L3 communication aggregation and current L4 module set.
- Refreshed mission baseline to v0.3.8 and rewrote next-iteration objectives.

## [0.3.6] - 2026-03-28

### Changed
- Completed P1.2 scheduler standardization: task-rate naming is unified to `FOC_TaskRate_t`, scheduler dividers are configured through centralized config macros, and old compatibility aliases were removed.
- Continued P1.3 macro/config convergence: domain configuration headers are connected through `foc_config.h`, and command/protocol/debug/runtime defaults were further migrated from implementation files into config headers.
- Advanced P1.4 header-boundary convergence: public headers were slimmed to reduce cross-layer exposure, and includes were moved to source files where implementation ownership belongs.
- Completed feature-cut macro introduction for diagnostics and control algorithms: diagnostics output/statistics can be trimmed independently, and control algorithm build set supports `FULL`, `SPEED_ONLY`, and `SPEED_ANGLE_ONLY` with explicit compile-time guards.

### Documentation
- Refreshed mission planning document for `v0.3.6`: removed completed tasks and reordered unfinished tasks by priority and execution dependency.
- Updated project README version section to reflect current delivery baseline and focus.

### Changed
- Moved scalar math helpers and constants into `math_transforms` (`Math_WrapRad`, `Math_WrapRadDelta`, `Math_ClampFloat`, `MATH_PI`, `MATH_TWO_PI`).
- Split control-layer responsibilities by introducing `foc_control_init` for motor initialization/calibration while keeping `foc_control` focused on runtime control algorithms.
- Added L2 internal bridge header `foc_control_internal.h` to share control-only helpers with init/calibration module.

## [0.3.5] - 2026-03-27

### Added
- Added command and runtime-state management module (`command_manager`) with unified command dispatch, parameter read/write, and init/runtime diagnostics.
- Added protocol parser module (`protocol_parser`) with frame extraction, structured command cache, parse-pending trigger model, and single-char feedback path.
- Added dual debug stream module (`debug_stream`) supporting semantic low-rate output and high-rate osc payload output.
- Added USART frame multiplexer (`comm_frame_mux`) with source mask and round-robin arbitration.
- Added protocol and parameter bilingual reference documentation (`docs/protocol-parameters-bilingual.md`).

### Changed
- Completed P0 communication base: USART1/USART2 moved to DMA RX + IDLE event frame capture with double-buffer strategy and DMA TX output path.
- Reworked platform communication API to transport-agnostic frame interfaces (`FOC_Platform_CommInit`, `FOC_Platform_ReceiveFrame`, `FOC_Platform_SetCommRxTriggerCallback`).
- Updated app loop communication path to non-blocking pending-poll processing with one-frame-per-step budget.
- Updated architecture and structure docs to reflect new module boundaries and communication flow.

### Fixed
- Unified invalid-command and invalid-parameter error accounting/reporting path.
- Fixed frame parsing robustness on mixed DMA chunks by extracting valid head-tail frame window before parse.

### Notes
- Build and flash validation passed on GD32F30X_CL target in this release cycle.

## [0.3.3] - 2026-03-25

### Changed
- Completed speed/speed-angle decoupling updates and stabilized transition behavior in the outer-loop control path.
- Removed USART protocol/loopback coupling and migrated USART1/USART2 TX paths from TBE interrupt sending to DMA-based sending.
- Simplified USART interrupt paths to focus on RX callback and buffer handling.

### Fixed
- Removed ADC debug pin usage on PB11 to eliminate conflict with USART2 RX and avoid pseudo-UART interrupt noise.
- Restored ADC init/start in platform sensor-input initialization after resolving the PB11 conflict.

## [0.3.2] - 2026-03-24

### Changed
- Finalized layered refactor for scheduler ownership: `ControlScheduler_Init` and task callback registration are now owned by application layer, while platform layer only encapsulates control-tick source init/bind/start.
- Completed sensor-path decoupling: sensor acquisition now reads raw device values via level-3 platform API wrappers, and level-2 `sensor` keeps processing/filter/structuring responsibilities.
- Introduced shared type hub `foc_shared_types.h` and centralized cross-module public types (`sensor_data_t`, `kalman_filter_t`, `foc_motor_t`, `foc_pid_t`, `foc_torque_mode_t`, scheduler callback/rate types).
- Renamed IRQ forwarding implementation source from `foc_irq_api_gd32.c` to `foc_irq_api.c` and synchronized project/docs references.
- Updated API and IRQ naming toward functional semantics and reduced peripheral-oriented names at upper layers.

### Fixed
- Resolved scheduler/platform coupling compile errors caused by header-only include removal by reassigning lifecycle boundaries instead of adding superficial include fixes.
- Restored build consistency after refactor by aligning callback binding path with the new control-tick source API.

### Notes
- Hardware build and flash verification passed for this release.

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
    - `examples/GD32F303_FOCExplore/hardware/hardware.md` only if pin mapping changes

### Project snapshot (v0.3.0)
- **MCU**: GD32F303CC (Cortex-M4)
- **Dev env**: VS Code / Cursor + EIDE extension (`cl.eide`)
- **Toolchain**: ARM Compiler 5 (AC5)
- **EIDE target**: `GD32F30X_CL`
- **Build outputs**: `examples/<instance>/software/build/GD32F30X_CL/` (e.g. `Project.axf`, `Project.hex`)

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
