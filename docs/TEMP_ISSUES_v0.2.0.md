# Temporary Issue List for v0.2.0

This file collects potential risk points found during the v0.2.0 code check.
It is intended for next mini-version planning and selective fix execution.

## High Priority

1. Concurrency risk on shared sensor data
- `Sensor_ReadAll()` updates data in the 1kHz timer callback.
- Main loop reads the same `sensor_data` fields for debug output.
- No `volatile` protection, lock, or snapshot copy is used.
- Risk: torn reads / inconsistent mixed-frame values.

2. Blocking zero-offset calibration during startup
- `Sensor_SetZeroOffset()` blocks for ~1000ms (`1000 * delay_1ms(1)`).
- No timeout fallback for invalid ADC/encoder status.
- Risk: long startup stall and poor fault diagnosability.

3. `printf` debug path can affect real-time behavior
- `UART_Debug_OutputOscilloscope()` uses `printf` in runtime loop.
- Retargeted `fputc` writes to USART1 byte-by-byte.
- Risk: extra CPU load and jitter in control-adjacent execution windows.

## Medium Priority

4. ADC latest-sample API indexing is static in old helpers
- `ADC_GetSample()` / `ADC_GetAllSamples()` still use a fixed tail index.
- New average API uses DMA transfer count and is better aligned.
- Risk: stale data if old helpers are used by future modules.

5. Early-buffer validity is not explicitly gated
- Average sampling can run immediately after ADC start.
- No explicit warm-up sample count check before using averaged result.
- Risk: first several control cycles may include incomplete history.

6. `AS5600_ReadAll()` lacks null pointer guard
- Accesses `data->...` directly without checking `data != NULL`.
- Risk: unexpected hard fault if caller misuse occurs.

## Low Priority

7. USART API constness regression
- `USART1_SendString` now accepts `char *` instead of `const char *`.
- Most call sites pass string literals.
- Risk: weaker API safety and avoidable implicit cast pressure.

8. Type consistency in USART send-byte API
- `USART1_SendByte` changed to `char` from byte-like type.
- Functional behavior is currently valid, but semantics are less explicit.
- Risk: portability/readability degradation for binary-oriented paths.

## Suggested Next-Step Fix Order

1. Shared data snapshot / critical-section strategy for `sensor_data`.
2. Non-blocking zero-offset state machine (or bounded startup calibration).
3. Replace `printf` telemetry path with framed non-blocking TX API.
4. Normalize ADC helper APIs to DMA write-index aware sampling.
5. Add guard rails (`NULL` checks, const-correctness, type cleanup).
