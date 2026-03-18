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

## User Suggestions
我将会根据你提到的8点问题给出建议：
1无需修改。1KHz任务中的计算和读取可以完全保证一致性。主循环中的读取即使存在一致性问题，也不会出现较大误差和影响算法逻辑，无需修改，修改这会带来不必要的麻烦。
2暂时不管，这是有意为之，以确保数据稳定性。之后会根据实际情况再改。目前无需管它。
3printf用于调试信息的方便输出，注意它的影响即可，不影响实际算法不用管。
4api仍然保留，它们尽管没有平均，但是优势在于CPU时间占用少，可以保留，可能有用。
5需要修改，我认为可以在ADC初始化函数中加入数组的填充。
6不管，单片机程序无需在意这个问题，设计完成的程序传入必然不为NULL。
7需要修改
8需要修改