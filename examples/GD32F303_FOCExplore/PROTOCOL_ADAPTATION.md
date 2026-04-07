# GD32F303_FOCExplore Protocol Adaptation

## Scope
This document defines transport/channel binding for the GD32F303_FOCExplore instance.
Protocol semantics remain in `../../docs/protocol-parameters-bilingual.md`.

## Channel Mapping
- Source1 RX: USART1 RX
- Source2 RX: USART2 RX
- Source3 RX: not wired (weak stub)
- Source4 RX: not wired (weak stub)
- Unified output channel: USART1 TX

## Serial Parameters
- Baudrate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None

## Driver ID Baseline
- Current default local driver id: `0x61` (`'a'`) via `FOC_PROTOCOL_LOCAL_DRIVER_ID_DEFAULT`.
- Quick-validation frames below use `aa...` for out-of-box behavior.

## Typical Host Setup
1. Terminal A bound to USART2 for command frame injection.
2. Terminal B bound to USART1 for status bytes and debug text.

## Quick Validation
1. Send `aaRXb` and verify full parameter dump on USART1 output.
2. Send `aaWP3.0b` and verify `O` feedback plus updated parameter text.
3. Send `aaQXb` and check runtime status summary output.
4. Send `aaFCb` and verify fault counters are cleared.

