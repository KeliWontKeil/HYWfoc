---
name: "Header Decoupling & Forward Declaration Convergence"
description: "Reduce header tight coupling by systematically applying forward declarations, eliminating transitive includes, and enforcing layer boundaries. Second phase of header refactoring. See NEXT_MISSION.md for current phase. Coordinate with @architecture-review and @primary-developer. Estimated: 4-6 hours."
argument-hint: "Check NEXT_MISSION.md for task phase and objectives before starting."
---

# Header Decoupling & Forward Declaration Convergence

## Objective

Further **reduce header coupling** through strategic application of forward declarations and elimination of transitive includes. Maintain existing type definitions while decreasing compilation dependency graph and strengthening layer boundaries.

**Key constraint**: Do NOT break existing public API types or force API consumers to know layering details.

---

## Problem Statement

Current state:
```c
// foc/include/algorithm/foc_control.h
#include "gd32f30x_timer.h"      // ❌ L2 exposes L4 device header
#include "Utilities/pwm_driver.h" // ❌ Unnecessary transitive include
#include "interface/foc_platform_api.h"     // ✅ Correct (via mandate)

// This means:
// - Any user of foc_control.h must compile GD32 headers
// - Tight coupling: renaming pwm_driver.h breaks this file
// - Slow compilation: L2 headers pull in entire peripheral tree
```

**Goal**:
```c
// foc/include/algorithm/foc_control.h (refactored)
#include "config/foc_shared_types.h"     // ✅ Only shared types needed
#include "interface/foc_platform_api.h"     // ✅ Mandate fulfilled
// Forward declarations in .h; actual includes only in .c

// Result:
// - Compile-time dependency on L4 removed
// - Layer boundaries visually clear
// - Header cascade reduced
```

---

## Task Breakdown

### Phase 1: Header Dependency Analysis

**Tool**: Create a dependency map script (or manual analysis)

```bash
# Find all #include lines in library public headers
for hdr in foc/include/interface/*.h foc/include/algorithm/*.h; do
  echo "=== $hdr ==="
  grep "^#include" "$hdr" | grep -v "^//"
done
```

**Categorize includes**:
- ✅ **Good**: `#include "config/foc_shared_types.h"` (same layer, essential types)
- ✅ **Good**: `#include "interface/foc_platform_api.h"` (mandate: cross-layer via API)
- ✅ **Good**: `#include <stdint.h>` (C standard library)
- ❌ **Bad**: `#include "gd32f30x_timer.h"` (L4 device header leaked)
- ❌ **Bad**: `#include "Utilities/pwm_driver.h"` (transitive; should go via platform API)
- ⚠️ **Check**: `#include "debug_stream.h"` (same layer; decide if forward-decl possible)

**Deliverable**: Dependency matrix (can be spreadsheet or text document)

| Header | Current Includes | Problematic? | Forward Decl Candidate? | Action |
|---|---|---|---|---|
| foc_control.h | gd32f30x_timer.h, math_transforms.h, foc_platform_api.h | YES: gd32f30x_timer.h | YES: for timer ISR callback | Remove timer.h, use function pointer typedef |
| foc_app.h | foc_control.h | NO | NO | Keep |
| protocol_parser.h | stdlib.h, stdint.h, config/foc_shared_types.h | NO | NO | Keep |

---

### Phase 2: Forward Declaration Strategy

**Definition**: Replace full `#include` with `typedef` or `struct` forward declaration when:
- Type is only used in **function signatures** (not member definitions)
- Actual type definition resides in a different layer
- Including the full header would create needless dependency

**Pattern 1: Opaque Pointer**
```c
// foc_control.h (BEFORE)
#include "gd32f30x_timer.h"
typedef struct {
    timer_parameter_struct_t timer_cfg;  // ❌ Exposes timer type
} ControlContext_t;

// foc_control.h (AFTER)
typedef void* TimerHandle_t;  // Forward declare (opaque)
typedef struct {
    TimerHandle_t timer_handle;  // ✅ Only a pointer, no type details
} ControlContext_t;

// foc_control.c (implementation only)
#include "gd32f30x_timer.h"  // Now hidden; users don't see it
```

**Pattern 2: Function Pointer Type**
```c
// foc_control.h (BEFORE)
#include "Utilities/pwm_driver.h"
void FOC_Control_SetPWMOutput(PWM_Channel_t ch, float duty);  // ❌ Exposes PWM enum

// foc_control.h (AFTER)
typedef void (*PWMOutputFunc_t)(uint8_t channel, float duty_percent);

// foc_control.c (implementation)
#include "Utilities/pwm_driver.h"
static PWMOutputFunc_t pwm_func = NULL;

void FOC_Control_Init(PWMOutputFunc_t output_fn) {
    pwm_func = output_fn;  // ✅ Deferred binding
}
```

**Pattern 3: Struct Forward Declaration**
```c
// foc_control.h
typedef struct SensorContext SensorContext_t;  // Forward declared
void FOC_Control_AttachSensor(SensorContext_t *sensor);  // Only in signature

// foc_control.c
#include "sensor.h"  // Full definition included here
void FOC_Control_AttachSensor(SensorContext_t *sensor) {
    sensor->value = ...;  // ✅ Can access now
}
```

---

### Phase 3: Transitive Include Elimination

**Algorithm**:
1. For each `#include "X"` in header H:
   - Check: Is X's type used in any inline function or macro in H?
   - If YES: Keep include
   - If NO: Move include to `.c` file (or replace with forward decl)

**Example refactoring**:

Before (foc_control.h):
```c
#include "Utilities/pwm_driver.h"  // Transitive
#include "math_transforms.h"       // Used in inline: MATH_TWO_PI
#include "config/foc_shared_types.h"      // Typedef definitions

void FOC_Control_Update(FOC_State *state);  // No PWM_* types in signature
```

After (foc_control.h):
```c
#include "math_transforms.h"       // ✅ Used inline
#include "config/foc_shared_types.h"      // ✅ Typedef definitions

void FOC_Control_Update(FOC_State *state);

// PWM_Driver_t removed from this scope
// Moved to implementation in foc_control.c
```

After (foc_control.c):
```c
#include "foc_control.h"
#include "Utilities/pwm_driver.h"  // ✅ Now in .c only
#include "interface/foc_platform_api.h"      // ✅ Mandate

// Implementation gets full type access
```

---

### Phase 4: Layer Boundary Enforcement

**Validation**: Ensure no **L2/L3 public header inadvertently includes L4 device headers**.

```bash
# Check for device header leaks in public interfaces
for hdr in foc/include/interface/*.h foc/include/algorithm/*.h; do
  if grep -q "gd32f30x_" "$hdr"; then
    echo "⚠️  LEAK: $hdr includes GD32 device header"
    grep "gd32f30x_" "$hdr"
  fi
done
# Should output: nothing (or only in marked exceptions)
```

**Allowed exceptions** (document in header):
```c
/* 
 * EXCEPTION: This header includes gd32f30x_timer.h because:
 * - It defines a public callback typedef that references timer_parameter_struct_t
 * - Alternative: Opaque pointer approach (see related prompt)
 * - Status: Candidate for P2 refactoring (library stage)
 */
```

---

### Phase 5: Verification & Compilation

**Full compilation check**:
```bash
eide.project.build
# Expected: 0 errors and no newly introduced warnings
# No new undefined references
```

**Dependency graph check** (if tools available):
```bash
# Example: gcc -M on each header to measure cascading includes
# Goal: Reduce header cascade depth by 2-3 levels minimum
```

**Symbol availability check**:
```c
// After refactoring, verify this still works:
FOC_State state = {0};
FOC_Control_Update(&state);  // ✅ Should compile
// Internals hidden: users don't see PWM driver types or timer configs
```

---

## Acceptance Criteria

- [ ] **No L4 device headers in L2/L3 public headers** (except pre-approved exceptions with justification)
- [ ] **Transitive includes eliminated**: Public headers include only direct dependencies
- [ ] **Forward declarations applied** where feasible (min. 5 candidates identified and converted)
- [ ] **No newly introduced compilation warnings**: `eide.project.build` introduces no new warnings
- [ ] **No API breaking changes**: Existing function signatures and types remain compatible
- [ ] **ROM/RAM stable**: ≤ ±100 bytes of current binary size
- [ ] **Layer boundaries clear**: Visual inspection confirms L1/L2/L3 do not expose L4 internals
- [ ] **Documentation updated**: Architecture diagram refinements in `docs/architecture.md`

---

## Implementation Checklist

- [ ] **Step 1**: Create header dependency matrix (Phase 1)
- [ ] **Step 2**: Identify forward-declaration candidates (Pattern analysis)
- [ ] **Step 3**: Apply forward declarations and remove transitive includes (Phase 3)
- [ ] **Step 4**: Run device-header leak scan (Phase 4)
- [ ] **Step 5**: Compile and verify (Phase 5)
- [ ] **Step 6**: Update architecture documentation
- [ ] **Step 7**: Add comments justifying any remaining exceptions
- [ ] **Step 8**: Verify `docs/engineering/dev-guidelines/rules/` alignment

---

## Example: Before & After

### Before (Tight Coupling)
```c
// foc/include/algorithm/foc_control.h
#include "gd32f30x.h"               // ❌ L4
#include "gd32f30x_timer.h"         // ❌ L4
#include "Utilities/pwm_driver.h"   // ❌ L4 via transitive
#include "Utilities/adc.h"          // ❌ L4 via transitive
#include "interface/foc_platform_api.h"       // ✅ Mandate
#include "math_transforms.h"        // ✅ Same layer, used inline

typedef struct {
    timer_parameter_struct_t timer;  // ❌ Exposes L4 type
    PWM_Channel_t pwm_channel;       // ❌ Exposes L4 enum
} ControlHandler_t;
```

### After (Decoupled)
```c
// foc/include/algorithm/foc_control.h
#include "config/foc_shared_types.h"       // ✅ Types only
#include "interface/foc_platform_api.h"       // ✅ Mandate
#include "math_transforms.h"        // ✅ Used inline

// Forward declarations
typedef void* TimerHandle_t;
typedef void* PWMHandle_t;

typedef struct {
    TimerHandle_t timer_handle;      // ✅ Opaque
    PWMHandle_t pwm_handle;          // ✅ Opaque
} ControlHandler_t;

// Implementation detail hidden in .c
```

---

## Related Documents

For phase context and current objectives, refer to `NEXT_MISSION.md` in the project root.

For architecture details and dependency rules, see `copilot-instructions.md` and `docs/architecture.md`.

---

## Success Signal

✅ **Header coupling removed**: L2/L3 public headers no longer expose L4 implementation details. Compilation cascade reduced, layer boundaries visually clear.

