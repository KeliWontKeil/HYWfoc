# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is an embedded C project (GD32F303CC, ARM Cortex-M4) using the **EIDE** (ARM-Clang Embedded IDE) extension in VS Code with **ARM Compiler 5 (AC5)** backend.

### Build command
```batch
set DOTNET_ROLL_FORWARD=Major
"C:\Users\MSI-NB\.vscode\extensions\cl.eide-3.26.9\res\tools\win32\unify_builder\unify_builder.exe" --rebuild -p "examples\GD32F303_FOCExplore\software\build\GD32F30X_CL\builder.params"
```

- `--rebuild`: full clean rebuild
- Target constraints: 0 errors, no new warnings, ROM ≤ 256KB, RAM ≤ 96KB
- Build entry: `examples/GD32F303_FOCExplore/software/` EIDE project
- Linker config: ROM `0x08000000`-`0x0803FFFF` (256KB), RAM `0x20000000`-`0x20017FFF` (96KB)
- Compiler flags: C99 mode, one-elf-section-per-function, microLIB, warning: all, optimization: level-1 (CL target)

### Common build errors
| Symptom | Root cause |
|---|---|
| `Not found any source files` | `builder.params` sourceList path mismatch with rootDir |
| `L6218E: Undefined symbol` | .c file not in sourceList, or declaration/definition mismatch |
| `#20: identifier undefined` | Missing include path in builder.params.incDirs |
| `#147-D: declaration is incompatible` | .h and .c function signatures differ |

Build logs: `build/GD32F30X_CL/unify_builder.log` or stdout of unify_builder.exe.

## Architecture Overview

Project uses a **"core library + board instance"** organization. The core library `foc_core/` is platform-independent; the board instance lives in `examples/GD32F303_FOCExplore/`.

### Layer architecture (strict unidirectional)
```
LS → L1 → L2 → L3 → L5
```

| Layer | Location | Responsibility |
|---|---|---|
| `LS` | `foc_core/include/LS_Config/` | Symbol definitions, feature switches, init values, compile constraints, types |
| `L1` | `foc_core/src/L1_Orchestration/foc_app.c` + `foc_service_handler.c` | Startup, service task, reinit, output manager, indicators |
| `L2/Control` | `foc_core/src/L2_Core/Control/foc_ctrl_*.c` (8 modules) | Control algorithms: executor, config, init, outer loop, current loop, param learn, compensation, actuation |
| `L2/Protocol` | `foc_core/src/L2_Core/Protocol/foc_protocol_*.c` | Frame parsing, command execution (P/S/Y channels), output adaptation |
| `L2/Runtime` | `foc_core/src/L2_Core/Runtime/foc_task_scheduler.c`, `foc_debug_stream.c` | Scheduler, debug stream (independent tools) |
| `L3` | `foc_core/include/L3_Hal/`, `foc_core/src/L3_Hal/` | Math transforms, LUT, platform API (PAL), sensor, SVPWM |
| `L5` | `examples/.../Utilities/` | Peripheral drivers and chip library |

### Key constraints
- `L1/L2/L3` access hardware **only** through `L3/foc_platform_api.h`
- Public headers must never expose `gd32f30x_*` device headers
- `L5` must not reverse-depend on `foc_core/src/*`
- Configuration constants must converge in `foc_cfg_*.h` — no scattered defaults in `.c` files

### L2 Control module naming
```
foc_ctrl_executor.c     — Control executor (ISR path + outer loop dispatch)
foc_ctrl_cfg.c          — Config state management (fine-tuning, soft-switch, cogging setters)
foc_ctrl_init.c         — Motor initialization and calibration
foc_ctrl_outer_loop.c   — Speed and speed+angle outer loop
foc_ctrl_current_loop.c — Current inner loop (PID + soft-switch)
foc_ctrl_param_learn.c  — Motor parameter learning (direction/pole-pairs estimation)
foc_ctrl_compensation.c — Cogging compensation + calibration
foc_ctrl_actuation.c    — Electrical angle application + SVPWM drive
```

### Application entry
```c
int main(void) {
    FOC_App_Init();   // Library init
    FOC_App_Start();  // Start control
    while (1) {
        FOC_App_Loop();  // Background service loop
    }
}
```

### Control timing
- PWM update ISR: high-speed current loop path
- Scheduler callback: service tasks, monitor tasks, control main loop
- Sampling trigger: aligned with PWM for current/angle acquisition

## Configuration System

Single entry: include `"foc_config.h"` (aggregates `foc_symbol_defs.h`, `foc_cfg_feature_switches.h`, `foc_cfg_init_values.h`, `foc_compile_limits.h`).

Three categories:
1. **Feature switches** (`foc_cfg_feature_switches.h`): `FOC_CURRENT_LOOP_PID_ENABLE`, `FOC_COGGING_COMP_ENABLE`, `FOC_SENSOR_KALMAN_*`, `FOC_PROTOCOL_ENABLE_*`, etc.
2. **Init values** (`foc_cfg_init_values.h`): default motor params, PID gains, scheduler frequencies
3. **Compile limits** (`foc_compile_limits.h`): static assertions, cross-feature conflict detection

**Important**: `FOC_PROTOCOL_ENABLE_*` macros control protocol command visibility only, not control algorithm behavior. Control algorithm behavior is gated by `FOC_*_ENABLE` macros.

## Communication Protocol

Frame format: `a<driver_id><cmd><subcmd><param>b`
- Default local ID: `0x61` (`'a'`)
- Command channels: `P` (parameter), `S` (status), `Y` (system)
- Return codes: `O`(OK), `E`(format error), `P`(invalid param), `I`(invalid cmd), `T`(timeout)
- Minimum set (non-trimmable): `P:A/R/S/D`, `S:M`, `Y:R/C`

## Coding Conventions

- **Braces**: Allman style (braces on own lines)
- **Function naming**: `Module_FunctionName` (e.g., `FOC_MotorInit`, `SVPWM_Update`)
- **Macros**: `UPPERCASE_WITH_UNDERSCORES`
- **Types**: `_t` suffix
- **Macro consistency**: when a feature switch is disabled, all declaration/definition/call sites must be consistently gated

## Version Control

- Semantic versioning (current: `v1.8.2`, target: `v2.0.0`)
- Default branch: `main` (develop directly on main)
- Push gate: only on `MINOR` or `MAJOR` version bumps
- Document sync: `architecture.md`, `development.md`, `README.md`, `CHANGELOG.md`, `NEXT_MISSION.md`, `copilot-instructions.md` must be updated in same iteration as code changes

## Current Mission

See `NEXT_MISSION.md` for current milestone plan.

## Key Files Reference

| File | Purpose |
|---|---|
| `foc_core/include/L3_Hal/foc_platform_api.h` | Platform adaptation API (must implement for new boards) |
| `foc_core/src/L3_Hal/foc_platform_api_empty.c` does not exist; see `examples/.../foc_platform_api.c` | Empty reference implementation for porting |
| `foc_core/include/LS_Config/foc_cfg_feature_switches.h` | All feature/trim switches |
| `foc_core/include/LS_Config/foc_cfg_init_values.h` | Default init values |
| `foc_core/include/LS_Config/foc_compile_limits.h` | Compile-time static assertions |
| `docs/architecture.md` | Single source of truth for architecture |
| `NEXT_MISSION.md` | Current and upcoming work items |
