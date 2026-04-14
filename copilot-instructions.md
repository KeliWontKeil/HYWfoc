---
description: "HYWfoc repository-level development instructions. Use when: implementing control algorithms, adding peripherals, fixing build issues, reviewing architecture, optimizing embedded code, and governing commit/version workflow."
---

# Copilot Instructions: HYWfoc Repository Workflow

## Project Overview

**HYWfoc** is a repository-level FOC development project. It provides a reusable control library plus instance-specific board integrations. Board details (pin maps, transport bindings, exact build outputs) must be maintained under `examples/<instance>/` rather than in global workflow instructions.

**Current Version**: v1.3.1 (snapshot release baseline)

### Key Build Outputs
- **Toolchain**: ARM Compiler 5.06 update 6 via EIDE extension
- **Build Command**: `eide.project.build` → incremental compilation to `examples/GD32F303_FOCExplore/software/build/GD32F30X_CL/Project.hex`
- **Flash Command**: `eide.project.buildAndFlash` → ST-LINK upload
- **Memory**: ROM 35.5KB/256KB (13.9%), RAM 2.5KB/96KB (2.6%)

## Directory Structure & Responsibility

```
foc/
├── include/              # L1/L2/L3 public headers + config headers
├── src/                  # L1/L2/L3 canonical sources
└── port/                 # platform API contract template

examples/GD32F303_FOCExplore/software/
├── Application/          # board entry + ISR + platform API implementation
├── Utilities/            # L4 peripheral drivers (instance-owned)
├── Firmware/             # vendor firmware (instance-owned)
├── .eide/                # instance EIDE project config
├── .pack/                # instance chip-pack assets
├── .cmsis/               # instance CMSIS helper assets
├── .vscode/              # instance build/debug tasks and C/C++ config
└── build/GD32F30X_CL/    # instance compiler output

docs/
└── engineering/dev-guidelines/rules/  # coding standards (en/cn)
```

## Workspace Responsibilities

- **Outer workspace** (`Project.code-workspace` at repo root): management-only (Git, docs, browsing).
- Outer workspace should not be used as the compile/flash/debug entry.
- Outer workspace should keep a single folder root (`.`) to avoid exposing instance build tasks.
- **Inner workspace** (`examples/<instance>/software/Project.code-workspace`): the only compile/flash/debug entry for that instance.

## Mandatory Dependency Contract

**Strict layering rule**: L1/L2/L3 must access L4 **only** through `foc_platform_api.[ch]` and shared types in `foc_shared_types.h`.

| Layer | Files | Purpose | External Access |
|-------|-------|---------|-----------------|
| **L1 (App)** | `main.c`, `foc_app.c` | Bootstrap & entry point | Only app-level APIs |
| **L2 (Algorithm)** | `foc_control.c`, scheduler, command mgr, debug/protocol | FOC logic, task dispatch | Via unified platform API |
| **L3 (Advanced Peripheral)** | `sensor.c`, `svpwm.c`, `protocol_parser.c` | Raw↔structured conversion | Via unified platform API |
| **L4 (Peripheral)** | `examples/<instance>/software/Utilities/` drivers (ADC, PWM, USART, I2C, timer, LED, AS5600) | GD32 hardware control | **Only via platform API** |
| **Special (Config & Shared)** | `foc_config.h` + domain headers, `foc_shared_types.h` | Unified defaults, shared structs, feature macros | All layers |

**Config Convergence Rule** (Single Source Principle): 
- Use `foc_config.h` with `foc_cfg_symbol_defs.h`, `foc_cfg_feature_switches.h`, `foc_cfg_init_values.h`, and `foc_cfg_compile_limits.h` as the **single source** for all configuration parameters.
- Apply this rule to **three categories**:
  1. **Runtime defaults** (e.g., `TORQUE_LIMIT`, `SPEED_MAX`) → `foc_cfg_init_values.h`
  2. **Feature switches and trim controls** (runtime mode gates / build variants) → `foc_cfg_feature_switches.h`
  3. **Range limits and compile constraints** (saturation, guardrails, hard limits) → `foc_cfg_compile_limits.h`
- Shared symbolic definitions and compatibility aliases must be centralized in `foc_cfg_symbol_defs.h`.
- **Forbidden**: Do NOT scatter defaults, constants, or parameters across `.c` files. All configurable values must be centralized.

## Naming & Code Style

- **Functions**: `ModuleName_FunctionName` (e.g., `FOC_Control_Update`, `ADC_Init`)
- **Variables**: `camelCase` (e.g., `controlState`, `dutyCycle`)
- **Macros**: `UPPER_CASE` (e.g., `MATH_TWO_PI`, `PWM_FREQUENCY`)
- **Types**: `type_name_t` (e.g., `foc_state_t`, `adc_sample_t`)
- **Structs**: `Module_Struct` (e.g., `FOC_State`, `ControlScheduler_Task`)

Language: **C99 with ARM Cortex-M4 extensions**. Always include `<stddef.h>` for NULL. Minimize floating-point; prefer fixed-point math.

## Control Timing Architecture

- **Master Tick**: TIMER1 @ 1kHz (control period = 1ms) — `FOC_Platform_BindControlTickCallback`
- **PWM Base**: TIMER0 center-aligned @ 24kHz, synchronized by TIMER2
- **ADC Trigger**: TIMER3 compare event (synchronous dual-channel current sampling)
- **Multi-Rate Tasks**: Scheduler dividers → 1kHz, 100Hz, 200Hz, 1Hz slots
- **⚠️ Speed Conversion**: `FOC_OpenLoopStep` assumes 1ms period. If changing base frequency, update speed-to-angle conversion.

See [Timing Architecture](docs/architecture.md) for full details.

## Common Workflows

### Adding a New Peripheral Driver

1. Create `examples/<instance>/software/Utilities/module_name.[ch]` (e.g., `can.c`)
2. Implement low-level init (`Module_Init`), read/write ops
3. **Do not** add device headers or config macros to L2/L3
4. Expose via instance platform API wrapper in `examples/<instance>/software/Application/Source/foc_platform_api.c` if upper layers need it
5. Update `examples/<instance>/hardware/hardware.md` and include a Doxygen header block

### Modifying Control Algorithm

1. Edit target file in `foc/src/` for shared logic or `examples/<instance>/software/Application/Source/` for instance logic
2. Follow layer contract: read sensor via L3 interface, call platform API for PWM
3. Add config macros to appropriate `foc_cfg_*.h` header **before** hardcoding values
4. Rebuild: `eide.project.build` (incremental compilation)
5. Validate: ROM/RAM usage + target hardware behavior

### Debugging & Profiling

- **Build with warnings enabled**: Compile should report 0 errors and no newly introduced warnings
- **Serial debug output**: Use `FOC_DebugStream_*` API (low-rate semantic + high-rate OSC paths)
- **Hardware validation**: Flash via ST-LINK, observe LED status + encoder feedback
- **GDB console**: Available via EIDE + ST-LINK adapter

### Feature Build Variants

**Control algorithm trimming** (see `foc_cfg_feature_switches.h`):
- `FULL`: Both speed-only and speed-angle modes (default, runtime switch)
- Single-algorithm build: Explicit feature cut (trim-to-one only)

## Coding Rules & Constraints

**See** [docs/engineering/dev-guidelines/rules/en/](docs/engineering/dev-guidelines/rules/en/) (or `cn/` for Chinese).

Key highlights:
- **Interrupt safety**: No blocking I/O in ISRs; use DMA or queues
- **Stack efficiency**: Minimize local arrays; pre-allocate buffers at module level
- **ROM optimization**: Use const for lookup tables, prefer inline for small 1-liners
- **Floating-point**: Avoid in real-time paths; check generated ASM

## Documentation Index

| Document | Purpose |
|----------|---------|
| [README.md](README.md) | Quick start, feature summary, version history |
| [architecture.md](docs/architecture.md) | Layering, timing, data flow, module contracts |
| [hardware.md](examples/GD32F303_FOCExplore/hardware/hardware.md) | Pin mappings, clock config, peripheral assignments |
| [development.md](docs/development.md) | Build steps, debugging, release process |
| [structure-and-dependency-tree.md](docs/structure-and-dependency-tree.md) | Include graph, data structure catalog |
| [CHANGELOG.md](CHANGELOG.md) | Version history & feature tracking |
| [NEXT_MISSION.md](NEXT_MISSION.md) | Current sprint goals (P1/P2 phases) |

## Build & Runtime Validation

### Build Checklist
- [ ] Compile with `eide.project.build` (incremental) or `eide.project.rebuild` (clean)
- [ ] Zero compiler errors; no newly introduced warnings
- [ ] ROM usage ≤ 256KB, RAM usage ≤ 96KB
- [ ] `examples/GD32F303_FOCExplore/software/build/GD32F30X_CL/Project.hex` generated
- [ ] Dependency layer contract maintained (L1/L2/L3 → L4 via platform API only)

### Hardware Validation
- [ ] Flash: `eide.project.buildAndFlash` → ST-LINK upload 
- [ ] LED status change observable (idle flashing, active/comms states distinguish)
- [ ] Serial debug output appears on USART1 (baud rate: 115200)
- [ ] Encoder I2C communication responds
- [ ] PWM observe on TIMER0 channels (PA8/PA9/PA10)

## Workflow Preferences

Per user convention:
- **Git commits**: After each completed modification cycle, create local `git commit` on `main` (unless user explicitly requests a different branch).
- **Git push**: Do not push by default. Push only when user explicitly requests.
- **Versioning policy**: Use `1.2.3` semantics where `1`=major, `2`=pushable minor, `3`=local revision. Each local commit must increment the local revision (`.3`) by `+1`.
- **Amend policy**: If user asks to revise the current commit, use `git commit --amend` instead of creating a new standalone commit.
- **Policy activation**: Active from `1.0.0` (first push milestone baseline).
- **Documentation**: Update before code review. Keep docs linked, not duplicated.
- **Task tracking**: Confirm "改代码/仅设计/暂缓" per item **before** auto-expanding scope.

---

## Agent Customizations

This workspace includes four specialized agents:
- **@primary-developer** — General development tasks (bugs, features, refactoring)
- **@foc-algorithm-review** — Control algorithm diagnostics and optimization
- **@architecture-review** — Code structure, coupling, and layering improvements
- **@documentation-compliance** — Documentation-code alignment validation

---

### Related Prompts & Skills

After reviewing this file, consider:
- `/debug-embedded` — Troubleshoot GD32 build/runtime issues
- `/optimize-rom-ram` — Profile ROM/RAM usage and reduce bloat
- `/review-arch` — Validate dependency layering and data flow
- `/add-peripheral` — Guided workflow for new driver integration
