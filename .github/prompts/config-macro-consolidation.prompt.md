---
name: "Config Macro Single-Source Convergence"
description: "Consolidate scattered default macros and constants into domain-specific foc_cfg_*.h headers. Eliminate duplicate definitions and establish single-source-of-truth for all configurable parameters. See NEXT_MISSION.md for current phase. Coordinate with @architecture-review, @primary-developer, and @documentation-compliance. Estimated: 3-4 hours."
argument-hint: "Check NEXT_MISSION.md for task phase and objectives before starting."
---

# Config Macro Single-Source Convergence

## Objective

**Ensure every configuration parameter is defined exactly once** in the appropriate `foc_cfg_*.h` header. Eliminate scattered defaults in `.c` files, consolidate duplicates, and establish clear ownership of each macro domain.

---

## Current State Assessment

Scan `foc/src/{interface,algorithm}/*.c` for these patterns:
```c
#define PWM_FREQUENCY           24000   // ❌ Should be in config/foc_cfg_init_values.h
#define TORQUE_LIMIT            5.0f    // ❌ Should be in config/foc_cfg_init_values.h
#define SCHEDULER_TICK_MS       1       // ❌ Should be in config/foc_cfg_symbol_defs.h
```

**Goal**: Move all such macros to centralized headers and remove duplicates.

---

## Config Header Ownership Map

| Domain | Header File | Ownership | Scope |
|---|---|---|---|
| **Init Defaults** | `config/foc_cfg_init_values.h` | Init + algorithm modules | Limits, default gains, startup values, platform base timing |
| **Symbol Definitions** | `config/foc_cfg_symbol_defs.h` | Scheduler + protocol modules | Tick divisors, frame symbols, constants/enums |
| **Feature Switches** | `config/foc_cfg_feature_switches.h` | Build configuration | Feature on/off switches and build-set options |
| **Compile Limits** | `config/foc_cfg_compile_limits.h` | Build-time checks | Compile-time range/consistency limits |
| **Aggregator** | `config/foc_config.h` | Public include entry | Pulls all `foc_cfg_*` headers as SSOT |

---

## Task Breakdown

### Phase 1: Inventory & Analysis

**Action**: Run a full codebase scan for `#define` statements in `foc/src/`

```bash
grep -rn "^\s*#define" foc/src/ | grep -v "^Binary"
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
1. Check if already in `foc_cfg_*.h` (eliminate duplicates if same value)
2. If not in config, **move to appropriate `foc_cfg_*.h`** and update `.c` file includes
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

After (config/foc_cfg_init_values.h):
```c
/**
 * @brief Centralized initialization and tuning defaults
 * @{
 */

// ⚠️ TORQUE_LIMIT: Firmware soft limit (motors: typ. 5.0 Nm).
//    Update if hardware rating changes. See examples/GD32F303_FOCExplore/hardware/hardware.md.
#define FOC_TORQUE_LIMIT_NM                 5.0f

// ⚠️ SPEED_MAX: Maximum motor speed in electrical RPM.
//    Prevents field-weakening regions if tuning enabled.
#define FOC_SPEED_MAX_RPM                   300.0f

/** @} */
```

After (foc_control.c):
```c
#include "config/foc_cfg_init_values.h"
// Remove local #define TORQUE_LIMIT, SPEED_MAX

// Use the centralized macro:
if (torque_cmd > FOC_TORQUE_LIMIT_NM) {
  torque_cmd = FOC_TORQUE_LIMIT_NM;
}
```

### Phase 3: Dependency Annotation

**In each config header**, add a **dependency comment** at the top:

```c
/**
 * @file config/foc_cfg_init_values.h
 * @brief FOC initialization and default runtime values
 *
 * Dependencies:
 *  - math_transforms.h (MATH_TWO_PI)
 *  - config/foc_cfg_symbol_defs.h (scheduler/protocol symbols)
 *  - config/foc_cfg_feature_switches.h (feature gates)
 *
 * Default values map to GD32F303CC @ 120MHz, 24kHz PWM, 1kHz control tick.
 */
```

### Phase 4: Verification

**Compilation check**: 
```bash
./eide.project.build  # Should output: 0 errors and no newly introduced warnings
grep -rn "^\s*#define" foc/src/ | wc -l  # Should be << original count
```

**Uniqueness check**:
```bash
# Ensure no macro is defined in multiple config headers
grep -rh "^#define FOC_" foc/include/config/foc_cfg_*.h | awk '{print $2}' | sort | uniq -d
# Should return empty output (no duplicated macro names)
```

---

## Acceptance Criteria

- [ ] **Inventory complete**: All macros in `foc/src/` catalogued and categorized
- [ ] **No duplicates**: Each macro value defined exactly once in appropriate config header
- [ ] **All `.c` files updated**: Include correct config header; remove local defines
- [ ] **Dependency documented**: Each config header lists its dependencies in file header
- [ ] **Naming consistent**: All migrated macros use project `FOC_*` naming and live in `foc_cfg_*.h`
- [ ] **No newly introduced warnings**: `eide.project.build` introduces no new warnings
- [ ] **Memory stable**: ROM/RAM usage unchanged (or within ±100 bytes)
- [ ] **Docs updated**: `docs/development.md` reflects new config layout with examples

---

## Configuration Macro Naming Convention

Establish and enforce this pattern:

```
FOC_<DOMAIN>_<PARAMETER_NAME>

Examples:
  FOC_SCHEDULER_TICK_HZ
  FOC_TORQUE_LIMIT_NM
  FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS
  FOC_PROTOCOL_FRAME_HEAD_CHAR
  FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT
```

---

## Implementation Checklist

- [ ] **Step 1**: Scan and list all macros in foc/src/
- [ ] **Step 2**: Create/update foc_cfg_*.h files with ownership assignments
- [ ] **Step 3**: Move macros and update includes in all .c files
- [ ] **Step 4**: Add dependency comments to config headers
- [ ] **Step 5**: Run uniqueness verification
- [ ] **Step 6**: Compile + verify no newly introduced warnings
- [ ] **Step 7**: Update CHANGELOG.md with consolidated macro list
- [ ] **Step 8**: Update docs/engineering/dev-guidelines/rules/ if new patterns established

---

## Related Files

- Config header templates: `foc/include/config/foc_cfg_*.h` + `foc/include/config/foc_config.h`
- Project includes: `foc/include/` (verify no `.c` local defaults bypass config headers)
- Build output: Expect no linker warnings about macro conflicts

---

## Success Signal

✅ **Single-source principle enforced**: Every configurable default is defined in exactly one config header, with clear ownership and documented dependencies.
