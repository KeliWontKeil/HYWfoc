---
name: "Config Macro Single-Source Convergence"
description: "Consolidate scattered default macros and constants into domain-specific foc_config_*.h headers. Eliminate duplicate definitions and establish single-source-of-truth for all configurable parameters. See NEXT_MISSION.md for current phase. Coordinate with @architecture-review, @primary-developer, and @documentation-compliance. Estimated: 3-4 hours."
argument-hint: "Check NEXT_MISSION.md for task phase and objectives before starting."
---

# Config Macro Single-Source Convergence

## Objective

**Ensure every configuration parameter is defined exactly once** in the appropriate `foc_config_*.h` header. Eliminate scattered defaults in `.c` files, consolidate duplicates, and establish clear ownership of each macro domain.

---

## Current State Assessment

Scan `Application/Source/*.c` for these patterns:
```c
#define PWM_FREQUENCY           24000   // ❌ Should be in foc_config_pwm.h
#define TORQUE_LIMIT            5.0f    // ❌ Should be in foc_config_control.h
#define SCHEDULER_TICK_MS       1       // ❌ Should be in foc_config_task.h
```

**Goal**: Move all such macros to centralized headers and remove duplicates.

---

## Config Header Ownership Map

| Domain | Header File | Ownership | Scope |
|---|---|---|---|
| **Control Algorithm** | `foc_config_control.h` | FOC_Control_* module | Torque/speed/position limits, PID tuning, ramp rates |
| **Task Scheduler** | `foc_config_task.h` | ControlScheduler_* module | Tick rates (1kHz), task dividers, priority levels |
| **Debug & Telemetry** | `foc_config_debug.h` | FOC_DebugStream_* module | Buffer sizes, sample rates, output modes |
| **Platform & Timing** | `foc_config_platform.h` | FOC_Platform_* API | Timer frequencies, ISR stack sizes, callback routing |
| **Hardware Peripherals** | `foc_config_hardware.h` | Utilities drivers | Pin assignments, ADC channels, PWM deadtime |
| **Communication** | `foc_config_comms.h` | Protocol_* / DMA modules | Baud rates, frame sizes, timeout values |

---

## Task Breakdown

### Phase 1: Inventory & Analysis

**Action**: Run a full codebase scan for `#define` statements in `Application/Source/`

```bash
grep -rn "^\s*#define" Application/Source/ | grep -v "^Binary"
```

**Deliverable**: Categorized list of all found macros
```
Control Layer:
  - foc_control.c: TORQUE_LIMIT=5.0, SPEED_MAX=200, ...
  - foc_control_init.c: CALIBRATION_PROBE_STEPS=10, ...
  
Scheduler Layer:
  - control_scheduler.c: SCHEDULER_DIVIDER_100HZ=10, ...

Debug Layer:
  - debug_stream.c: DEBUG_BUFFER_SIZE=256, ...
```

### Phase 2: Consolidation

**For each macro found**:
1. Check if already in `foc_config_*.h` (eliminate duplicates if same value)
2. If not in config, **move to appropriate `foc_config_*.h`** and update `.c` file includes
3. Add **comment block** explaining dependency or constraints:
   ```c
   // ⚠️ TORQUE_LIMIT: Must not exceed motor nameplate torque (5.0 Nm typical).
   //    Paired with TORQUE_RAMP_TIME_MS for smooth startup.
   #define TORQUE_LIMIT                    5.0f
   ```

**Example refactoring**:

Before (foc_control.c):
```c
#define TORQUE_LIMIT     5.0f
#define SPEED_MAX        300.0f
```

After (foc_config_control.h):
```c
/**
 * @defgroup FOC_Config_Control FOC Control Parameters
 * @{
 */

// ⚠️ TORQUE_LIMIT: Firmware soft limit (motors: typ. 5.0 Nm).
//    Update if hardware rating changes. See docs/hardware.md.
#define FOC_CONFIG_TORQUE_LIMIT_NM          5.0f

// ⚠️ SPEED_MAX: Maximum motor speed in electrical RPM.
//    Prevents field-weakening regions if tuning enabled.
#define FOC_CONFIG_SPEED_MAX_RPM            300.0f

/** @} */
```

After (foc_control.c):
```c
#include "foc_config_control.h"
// Remove local #define TORQUE_LIMIT, SPEED_MAX

// Use the centralized macro:
if (torque_cmd > FOC_CONFIG_TORQUE_LIMIT_NM) {
    torque_cmd = FOC_CONFIG_TORQUE_LIMIT_NM;
}
```

### Phase 3: Dependency Annotation

**In each config header**, add a **dependency comment** at the top:

```c
/**
 * @file foc_config_control.h
 * @brief FOC Control Algorithm Configuration
 *
 * Dependencies:
 *  - math_transforms.h (MATH_TWO_PI)
 *  - foc_config_task.h (scheduler tick rate affects speed integration)
 *  - foc_config_hardware.h (ADC resolution affects current scaling)
 *
 * Default values map to GD32F303CC @ 120MHz, 24kHz PWM, 1kHz control tick.
 */
```

### Phase 4: Verification

**Compilation check**: 
```bash
./eide.project.build  # Should output: 0 information, 0 warning, 0 error
grep -rn "^\s*#define" Application/Source/ | wc -l  # Should be << original count
```

**Uniqueness check**:
```bash
# Ensure no macro is defined in multiple config headers
for macro in $(grep "^#define FOC_CONFIG_" foc_config_*.h | awk '{print $2}' | sort); do
  count=$(grep -c "^#define $macro" foc_config_*.h)
  if [ $count -gt 1 ]; then
    echo "ERROR: $macro defined $count times"
  fi
done
```

---

## Acceptance Criteria

- [ ] **Inventory complete**: All macros in `Application/Source/` catalogued and categorized
- [ ] **No duplicates**: Each macro value defined exactly once in appropriate config header
- [ ] **All `.c` files updated**: Include correct config header; remove local defines
- [ ] **Dependency documented**: Each config header lists its dependencies in file header
- [ ] **Naming consistent**: All migrated macros use `FOC_CONFIG_DOMAIN_NAME` pattern
- [ ] **Zero warnings**: `eide.project.build` → "0 information, 0 warning, 0 error"
- [ ] **Memory stable**: ROM/RAM usage unchanged (or within ±100 bytes)
- [ ] **Docs updated**: `docs/development.md` reflects new config layout with examples

---

## Configuration Macro Naming Convention

Establish and enforce this pattern:

```
FOC_CONFIG_<DOMAIN>_<PARAMETER_NAME>

Examples:
  FOC_CONFIG_CONTROL_TORQUE_LIMIT_NM
  FOC_CONFIG_TASK_TICK_RATE_HZ
  FOC_CONFIG_DEBUG_BUFFER_SIZE_BYTES
  FOC_CONFIG_PLATFORM_PWM_FREQUENCY_HZ
  FOC_CONFIG_HARDWARE_ADC_RESOLUTION_BITS
```

---

## Implementation Checklist

- [ ] **Step 1**: Scan and list all macros in Application/Source/
- [ ] **Step 2**: Create/update foc_config_*.h files with ownership assignments
- [ ] **Step 3**: Move macros and update includes in all .c files
- [ ] **Step 4**: Add dependency comments to config headers
- [ ] **Step 5**: Run uniqueness verification
- [ ] **Step 6**: Compile + verify zero warnings
- [ ] **Step 7**: Update CHANGELOG.md with consolidated macro list
- [ ] **Step 8**: Update docs/engineering/dev-guidelines/rules/ if new patterns established

---

## Related Files

- Config header template: `foc_config_control.h` (existing, use as reference)
- Project includes: `Application/Include/` (verify no .c includes beyond config headers)
- Build output: Expect no linker warnings about macro conflicts

---

## Success Signal

✅ **Single-source principle enforced**: Every configurable default is defined in exactly one config header, with clear ownership and documented dependencies.
