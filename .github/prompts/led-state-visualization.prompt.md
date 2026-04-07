---
name: "LED State Visualization & Interrupt Tracing"
description: "Implement comprehensive LED state machine that visually indicates system status without serial dependency. Includes state-to-LED mapping and optional interrupt tracing via GPIO. See NEXT_MISSION.md for current phase. Coordinate with @primary-developer and @architecture-review. Estimated: 4-6 hours."
argument-hint: "Check NEXT_MISSION.md for task phase and objectives before starting."
---

# LED State Visualization & Interrupt Tracing

## Objective

Transform the current simple LED blinking logic into a **robust state-machine-driven visualization system** that allows diagnosis of system state with **zero serial dependency**. Additionally, add optional interrupt tracing via GPIO for real-time event monitoring.

---

## Requirements

### 1. State Machine → LED Mapping

Define a **deterministic state-to-LED behavior** table:

| System State | LED Pattern | Visual Meaning | Recovery Action |
|---|---|---|---|
| **Initialization** | Slow blink (0.5Hz) | System booting | Auto-transition to Ready after init complete |
| **Ready (Idle)** | Fast blink (2Hz) | Awaiting command | None |
| **Control Active** | Solid ON | FOC running, motor controlled | None |
| **Communication Active** | Double blink + hold (pattern) | Data RX/TX in progress | Auto-revert to Ready after 500ms idle |
| **Underload / Low Current** | Slow blink (1Hz) | Motor spinning but low torque | None (normal) |
| **Fault: Overcurrent** | Fast blink (5Hz) | Current limit triggered | Manual reset required |
| **Fault: Undervoltage** | SOS pattern (· · · — — — · · ·) | Supply voltage too low | Manual reset required |
| **Fault: Overspeed** | Rapid double-blink (3 on/off pairs then hold) | Speed limit exceeded | Manual reset or param change |
| **Degraded Mode** | Dim/PWM pattern (50% brightness) | Performance limited, continuing | Auto-upgrade when condition clears |

### 2. Implementation Location

Create or refactor the LED module at:
- **Header**: `foc/include/interface/led_visualizer.h`
- **Implementation**: `foc/src/interface/led_visualizer.c`

**Dependencies**:
- `config/foc_shared_types.h` (for state enums)
- `interface/foc_platform_api.h` (GPIO abstraction)
- `config/foc_cfg_init_values.h` + `config/foc_cfg_feature_switches.h` (LED defaults and feature switches)

### 3. Key Functions

```c
typedef enum {
    LED_STATE_INIT,
    LED_STATE_READY,
    LED_STATE_CONTROL_ACTIVE,
    LED_STATE_COMM_ACTIVE,
    LED_STATE_UNDERLOAD,
    LED_STATE_FAULT_OVERCURRENT,
    LED_STATE_FAULT_UNDERVOLTAGE,
    LED_STATE_FAULT_OVERSPEED,
    LED_STATE_DEGRADED
} LED_VisualizerState_t;

// Called by control loop each 1kHz tick
void LED_Visualizer_Update(LED_VisualizerState_t state);

// System state → LED state wrapper
void LED_Visualizer_SetSystemState(FOC_State state, FOC_Fault fault_code);

// Manual pattern override (for diagnostics)
void LED_Visualizer_OverridePattern(uint16_t pattern_bitmask, uint16_t period_ms);
```

### 4. Optional: Interrupt Tracing (GPIO Toggle)

If `FEATURE_LED_ISR_TRACE` is enabled in `foc_config.h`:
- On ISR entry: toggle GPIO (PA9 or configurable)
- On ISR exit: toggle GPIO back
- **Purpose**: Oscilloscope capture of interrupt timing + jitter

```c
#define FEATURE_LED_ISR_TRACE    0   // Set to 1 to enable

// In each ISR handler:
if (FEATURE_LED_ISR_TRACE) {
    GPIO_OutputBit_Toggle(GPIOA, GPIO_PIN_9);
    // ... ISR body ...
    GPIO_OutputBit_Toggle(GPIOA, GPIO_PIN_9);
}
```

---

## Acceptance Criteria

- [ ] **State machine** defined and implemented with no hardcoded delays
- [ ] **All 9 states** have visually distinct patterns (documented in table above)
- [ ] **Diagnostic capability**: Given any system state, LED pattern is deterministic and observable within **5 seconds**
- [ ] **Zero serial dependency**: LED state visible even if USART is misconfigured
- [ ] **Integration**: System state → LED mapping linked to actual FOC state enum in `foc/src/interface/foc_app.c`
- [ ] **No ROM/RAM regression**: LED module adds ≤1KB ROM, ≤512B RAM
- [ ] **Compile with no newly introduced warnings** on ARM Compiler 5
- [ ] **Hardware validation**: Patterns verified on GD32F303CC dev board using oscilloscope + visual inspection
- [ ] **Documentation**: LED state table added to `docs/README.md` and Doxygen comments in header

---

## Implementation Notes

### Architecture Constraints
- **Layer placement**: L3 (Advanced Peripheral) — converts application state → LED actuation
- **Dependencies**: Must use platform API for GPIO access; do not include `gd32f30x_gpio.h` directly
- **Timing**: 1kHz tick in control scheduler → LED pattern counters advance safely

### Config Integration

Add to `foc/include/config/foc_cfg_init_values.h` and `foc/include/config/foc_cfg_feature_switches.h`:
```c
#define LED_STATE_BLINK_INIT_FREQ_HZ     0.5f
#define LED_STATE_BLINK_READY_FREQ_HZ    2.0f
#define LED_STATE_ON_DIM_PERCENT         100   // 0-100
#define LED_TRACE_ISR_PIN                GPIO_PIN_9
#define FEATURE_LED_ISR_TRACE            0
```

### Testing Plan
1. Simulate state transitions in a test harness or GDBGUI
2. Flash to hardware, trigger each fault condition, observe LED
3. Measure pattern timing with oscilloscope to verify accuracy
4. Cross-check visual clarity against requirement table

---

## Related Files & Context

- Current LED driver: `Utilities/led.c` (check existing API)
- System state enum: `config/foc_shared_types.h` (FOC_State, FOC_Fault)
- Control loop: `foc/src/interface/foc_app.c` (FOC_App_Loop)
- Config structure: See existing split in `foc/include/config/foc_cfg_*.h`

---

## Success Signal

✅ System is **100% debuggable via LED alone**—no serial console required. All states are visually distinct and deterministic.
