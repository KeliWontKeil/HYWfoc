---
name: gd32-algorithm-workflow
description: Guides the agent when adding or modifying timer-based algorithm tasks, PWM settings, and USART behavior in this GD32F303CC project. Use when working in timer1_algorithm.c, timer modules, PWM, or USART-related C files.
---

# GD32 Algorithm & Timer Workflow

## When to Use This Skill

Use this skill when:
- Adding or modifying tasks in `timer1_algorithm.c` (1kHz / 100Hz / 10Hz / 1Hz 任务槽)。
- Changing TIMER1 or TIMER2 callback behavior。
- Adjusting TIMER0 PWM behavior that interacts with control loops。
- Wiring new control/monitoring logic into periodic tasks instead of ad-hoc delays。

## High-Level Workflow

1. **Clarify Task Frequency**
   - Decide which rate is appropriate:
     - **1kHz**: inner control loops, fast current/voltage control, time‑critical monitoring。
     - **100Hz**: speed loops, parameter updates, medium‑rate observers。
     - **10Hz**: communications, diagnostics aggregation, slower observers。
     - **1Hz**: status LEDs, health checks, very slow housekeeping。
   - Map the task to `Algorithm_1kHz_Task`, `Algorithm_100Hz_Task`, `Algorithm_10Hz_Task`, or `Algorithm_1Hz_Task` as appropriate.

2. **Locate the Right Integration Point**
   - For periodic algorithms:
     - Prefer editing `timer1_algorithm.c` and its `Algorithm_*Hz_Task` functions.
     - Note: `Timer1_Algorithm_Init()` now handles TIMER1 initialization with parameters.
   - For simpler low‑frequency callbacks:
     - Consider using `Timer2_SetCallback` for non‑critical tasks that can tolerate ~500ms period.
   - For pure hardware configuration (GPIO, basic timer setup, USART configuration, etc.):
     - Modify only the appropriate `Utilities` module (`PWM`, `TIMER1`, `TIMER2`, `USART`) and keep configuration code out of `gd32f30x_it.c` and `timer1_algorithm.c` where possible.
     - **Timer modules now use parameterized initialization**: `Timer1_Init(prescaler, period)`, `Timer2_Init(prescaler, period)`, `PWM_Timer_Config(prescaler, period)`.

3. **Implement Logic Safely**
   - Keep the body of each `Algorithm_*Hz_Task` reasonably small and predictable.
   - If logic grows large, extract it into helper functions in a dedicated module and call from the task.
   - Avoid inside 1kHz / timer ISRs:
     - Blocking delays such as `delay_1ms`。
     - Long loops with unbounded iteration counts。
     - Large stack allocations or heavy dynamic memory usage。

4. **Respect Execution Time Budget**
   - For substantial 1kHz work:
     - Ensure `Timer1_EnableDWT()` is used (if not already).
     - Optionally expose or log `Timer1_GetExecutionTime()` for debugging.
   - Aim to keep worst‑case execution time safely below the 1ms period, especially when combining multiple subtasks.

5. **Check Hardware & Pin Mapping**
   - When tasks affect physical IO (PWM, LEDs, USART, ADC, etc.)：
     - Confirm pin mappings via `Hardware.md` rather than guessing。
     - Prefer reusing existing GPIO/USART/PWM configurations in `Utilities` instead of reconfiguring peripherals in algorithm code or interrupt vectors。

6. **Coordinate Scope with User**
   - When the requested change is ambiguous in scope:
     - Prefer a minimal, well‑scoped implementation that clearly satisfies the described behavior。
     - If a structural refactor (e.g., reorganizing timer responsibilities or consolidating tasks) would provide clear benefits, outline the proposed structure and its impact in the response before applying large changes。

7. **Testing Strategy (Minimum)**
   - After any change that touches timers, PWM, or USART:
     - Ensure the project still builds via the existing EIDE tasks。
     - On hardware, at least sanity‑check that:
       - LED and USART behaviors remain sensible (startup messages, echo if enabled, LED3 blink unless intentionally changed)。
       - PWM frequency and approximate duty cycles remain as expected, unless the change is specifically meant to alter them。

