# Protocol Parameters and Test Guide / 协议参数与测试指南

本文件面向上位机调试和联调测试，描述当前固件实际实现的协议规则。
This document is user-facing and follows the current firmware implementation.

## 1. Transport Binding / 传输绑定

- Input command RX sources (abstract): Source1, Source2, Source3, Source4.
- Unified output channel (abstract): Output channel for feedback bytes and debug text.

Implementation note:

- The library parser supports up to 4 input sources. A concrete instance may wire only a subset (for example Source1 and Source2) and keep remaining sources as empty/weak stubs.

This document defines protocol semantics only; concrete UART/USB/CAN mapping belongs to each instance document.

For GD32F303 reference binding, see:
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

## 2. Frame Format / 帧格式

### 2.1 Canonical Format / 标准格式

```
a<driver_id><cmd><subcmd><param>b
```

- Frame head: `a` (lowercase, fixed)
- Frame tail: `b` (lowercase, fixed)
- `<driver_id>`: one byte, valid range `0x32-0x7E` (`'2'..'~'`), broadcast `0xFF`
- `<cmd>`: uppercase letter `A-Z`
- `<subcmd>`: uppercase letter `A-Z`
- `<param>`: optional for read/state/fault control, mandatory for write command

Example:

```
aaWA3.14b
```

### 2.2 Driver ID Rules / 驱动器编号规则

- Local driver id default (current code baseline): `0x61` (`'a'`)
- Valid unicast range: `0x32-0x7E`
- Broadcast id: `0xFF`
- Invalid `driver_id` format causes frame error (`E`)
- Valid but non-targeted id is silently dropped (no status byte)

Note:

- Examples in this document use the current default local id (`'a'`). If you change `FOC_PROTOCOL_LOCAL_DRIVER_ID_DEFAULT`, update test frames accordingly.

### 2.3 Numeric Parsing Rules / 数值解析规则

`<param>` supports only signed decimal float text:

- Allowed: `123`, `-1.5`, `+0.25`, `.5`
- Not allowed: scientific notation (`1e-3`), spaces, extra symbols

### 2.4 Length and Packeting Rules / 长度与发包规则

- Parser RX buffer max length: 64 bytes
- Protocol minimum frame length: 5 bytes (`a` + driver_id + cmd + subcmd + `b`)
- Param text max length: 31 chars
- Recommended: send one frame per packet with a small inter-frame gap

Important behavior:

- The parser extracts only the first valid `a...b` frame from one received chunk.
- If you concatenate multiple frames in one burst, trailing frames may be dropped.
- If `driver_id` format is valid but not targeted to local ID and not broadcast, the frame is silently dropped.

## 3. Command Characters / 指令字符定义

| Command | Meaning (EN) | 含义 (中文) | Typical form |
|---|---|---|---|
| `W` | Write one parameter | 写单参数 | `a<id>W<subcmd><value>b` |
| `R` | Read one parameter | 读单参数 | `a<id>R<subcmd>b` |
| `R` + `X` | Read all parameters | 读全部参数 | `a<id>RXb` |
| `Q` | Read runtime state summary | 读取运行状态摘要 | `a<id>QXb` (subcmd placeholder) |
| `F` + `C` | Fault clear + soft diag reinit | 清故障计数并软重置诊断 | `a<id>FCb` |

Notes:

- `Q` command ignores subcmd in execution, but frame format still requires one uppercase subcmd character.
- `F` currently supports only subcmd `C`.

## 4. Parameter Subcommands / 参数子命令

### 4.1 Full Reference Table / 完整参数表

| Subcmd | Parameter name | Type | Range | Default | Unit | W Example | R Example |
|---|---|---|---|---|---|---|---|
| `A` | target_angle_rad | float | [-12.566, 12.566] | 3.14 | rad | `aaWA1.57b` | `aaRAb` |
| `R` | angle_position_speed_rad_s | float | [0, 200] | 18.0 | rad/s | `aaWS10b` | `aaRSb` |
| `S` | speed_only_speed_rad_s | float | [-200, 200] | 18.0 | rad/s | `aaWR-10b` | `aaRRb` |
| `W` | sensor_sample_offset_percent | float | [0, 100] | 96.0 | % | `aaWW96b` | `aaRWb` |
| `L` | semantic_report_frequency_hz | uint | [1, 200] | 2 | Hz | `aaWL10b` | `aaRLb` |
| `H` | oscilloscope_report_frequency_hz | uint | [1, 200] | 50 | Hz | `aaWH100b` | `aaRHb` |
| `Y` | semantic_report_enabled | bool | 0 or 1 | 0 | - | `aaWY1b` | `aaRYb` |
| `Z` | oscilloscope_report_enabled | bool | 0 or 1 | 0 | - | `aaWZ1b` | `aaRZb` |
| `O` | oscilloscope_param_mask | uint | [0, 65535] | 24 (0x0018) | bitmask | `aaWO56b` | `aaROb` |
| `C` | pid_current_kp | float | [0, 50] | 0.0 | - | `aaWC0.2b` | `aaRCb` |
| `I` | pid_current_ki | float | [0, 50] | 0.0 | - | `aaWI0.1b` | `aaRIb` |
| `J` | pid_current_kd | float | [0, 10] | 0.0 | - | `aaWJ0.01b` | `aaRJb` |
| `G` | pid_angle_kp | float | [0, 50] | 2.0 | - | `aaWG2.5b` | `aaRGb` |
| `K` | pid_angle_ki | float | [0, 50] | 0.8 | - | `aaWK0.9b` | `aaRKb` |
| `N` | pid_angle_kd | float | [0, 10] | 0.01 | - | `aaWN0.02b` | `aaRNb` |
| `P` | pid_speed_kp | float | [0, 50] | 1.5 | - | `aaWP3.0b` | `aaRPb` |
| `U` | pid_speed_ki | float | [0, 50] | 0.8 | - | `aaWU0.6b` | `aaRUb` |
| `V` | pid_speed_kd | float | [0, 10] | 0.01 | - | `aaWV0.05b` | `aaRVb` |
| `M` | control_min_mech_angle_accum_delta_rad | float | >= 0 | 0.001 | rad | `aaWM0.002b` | `aaRMb` |
| `B` | control_angle_hold_integral_limit | float | >= 0 | 0.2 | - | `aaWB0.3b` | `aaRBb` |
| `E` | control_angle_hold_pid_deadband_rad | float | >= 0 | 0.005 | rad | `aaWE0.004b` | `aaREb` |
| `F` | control_speed_angle_transition_start_rad | float | >= 0 | 0.40 | rad | `aaWF0.45b` | `aaRFb` |
| `T` | control_speed_angle_transition_end_rad | float | >= 0 | 0.60 | rad | `aaWT0.70b` | `aaRTb` |
| `D` | control_mode | uint | 0 or 1 | 0 (FULL build default) | - | `aaWD0b` | `aaRDb` |
| `Q` | motor_enable | bool | 0 or 1 | 1 | - | `aaWQ1b` | `aaRQb` |
| `X` | read_all sentinel | - | R only | - | - | N/A | `aaRXb` |

Bool write note:

- Implementation maps `0` to disable and any non-zero value to enable.
- For interoperability and clear test logs, always use `0` or `1`.

### 4.2 Control Mode Values / 控制模式值

- `0`: speed + angle mode
- `1`: speed-only mode

Build-trim constraint:

- FULL build: allows `0` and `1`
- SPEED_ONLY build: only `1` is accepted
- SPEED_ANGLE_ONLY build: only `0` is accepted

Speed parameter mapping:

- `S` (`angle_position_speed_rad_s`): speed reference used by speed+angle mode (non-negative limit)
- `R` (`speed_only_speed_rad_s`): speed reference used by speed-only mode (supports negative direction)

### 4.3 Oscilloscope Mask Bits / 示波参数掩码位

| Bit | Hex | Field |
|---|---|---|
| 0 | 0x0001 | current_a |
| 1 | 0x0002 | current_b |
| 2 | 0x0004 | current_c |
| 3 | 0x0008 | angle_filtered |
| 4 | 0x0010 | angle_accum |
| 5 | 0x0020 | execution_time_us |

Default mask is `0x0018` (angle_filtered + angle_accum).

## 5. Responses and Status Codes / 返回与状态码

### 5.1 Feedback Byte / 单字节反馈

| Code | Meaning |
|---|---|
| `O` | Frame parsed successfully |
| `E` | Frame format error (head/tail/cmd/subcmd/param parse failed) |
| `P` | Parameter invalid (unknown subcmd, missing write value, out of range) |
| `I` | Command invalid (unknown cmd) |
| `T` | Reserved timeout code (currently defined but not emitted) |

Execution order note:

- A syntactically correct but semantically wrong frame can produce two feedback bytes in sequence: first `O`, then `P` or `I`.

### 5.2 Debug Text Output / 文本返回

When diagnostics output is enabled, responses are plain text lines on the configured output channel, for example:

```
parameter.pid_speed_kp=3.000
parameter.semantic_report_enabled=ENABLE
STATE SYS=1 COMM=1 REPORT=1 DIRTY=1 LAST=1 INIT=1 FAULT=NONE SENS_INV=0 PROTO_ERR=0 PARAM_ERR=0 CTRL_SKIP=0
FAULT_CTRL state=1 fault=NONE proto_err=0 param_err=0 ctrl_skip=0
```

Formatting rules:

- Float parameters: 3 decimal places
- Integer parameters: unsigned decimal text
- Enable/disable parameters: `ENABLE` or `DISABLE`

Fault-state behavior:

- In FAULT state, debug stream periodic semantic/osc output is suppressed.
- Command-path diagnostics and explicit query/clear commands are still available.

## 6. Complete Command Examples / 完整命令示例

### 6.1 Core commands / 核心命令

```text
aaWA3.14b   # write target_angle_rad
aaRAb       # read target_angle_rad
aaRXb       # read all parameters
aaQXb       # read runtime state summary
aaFCb       # clear fault counters + soft diag reinit
```

### 6.2 Typical parameter writes / 常用参数写入示例

```text
aaWD0b      # control_mode = speed+angle
aaWD1b      # control_mode = speed-only
aaWR-20b    # speed-only speed reference = -20rad/s
aaWS12b     # speed+angle mode speed limit/reference = 12rad/s
aaWQ1b      # motor enable
aaWQ0b      # motor disable
aaWW96b     # sensor sample offset percent = 96
aaWL20b     # semantic report frequency = 20Hz
aaWH100b    # osc report frequency = 100Hz
aaWO63b     # osc mask bits 0..5 all on
aaWP3.0b    # speed kp
aaWU0.6b    # speed ki
aaWV0.05b   # speed kd
```

### 6.3 Typical reads / 常用读取示例

```text
aaRDb       # read control_mode
aaRRb       # read speed_only_speed_rad_s
aaRSb       # read angle_position_speed_rad_s
aaRQb       # read motor_enable
aaRWb       # read sensor_sample_offset_percent
aaROb       # read oscilloscope_param_mask
aaRPb       # read pid_speed_kp
aaRMb       # read min_mech_angle_accum_delta
```

## 7. Common Error Cases / 常见错误与原因

| Frame | Expected feedback | Reason |
|---|---|---|
| `aaWA1e-3b` | `E` | Scientific notation not supported |
| `aaWAb` | `O` then `P` | Write command missing value |
| `aaWZonb` | `E` | Non-numeric write value cannot be parsed as float |
| `aaRxb` | `E` | subcmd must be uppercase `A-Z` |
| `aa?A1b` | `E` | cmd must be uppercase `A-Z` |
| `aaZA1b` | `O` then `I` | Unknown command group |
| `2WA1b` | `E` | Missing frame head `a` |
| `aaWA1` | `E` | Missing frame tail `b` |

## 8. Recommended Test Procedure / 推荐测试流程

1. Open host tools bound to instance input/output channels.
2. Send `aaRXb` and confirm a full parameter dump appears on output channel.
3. Send one write command (for example `aaWP3.0b`), verify:
  - output channel receives `O`
  - output channel prints updated parameter line.
4. Send `aaQXb` to check runtime status summary.
5. Send `aaFCb` and confirm fault counters are cleared.

Instance-specific serial terminal setup and channel wiring are documented in:
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

Best practices:

- Send one frame at a time (do not concatenate multiple frames in one burst).
- Keep command/subcommand uppercase and frame delimiters lowercase (`a`, `b`).
- Keep write values inside valid ranges.

