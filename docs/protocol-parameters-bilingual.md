# Protocol Parameters and Test Guide / 协议参数与测试指南

本文件面向上位机调试和联调测试，描述当前固件实际实现的协议规则。
This document is user-facing and follows the current firmware implementation.

## 1. Communication Channels / 通信通道

- Input command RX sources (both supported):
  - Source1: USART1 RX
  - Source2: USART2 RX
- Unified output channel:
  - USART1 TX (single-byte feedback codes and human-readable debug/state lines)

Serial settings (both USART1 and USART2):

- Baudrate: 115200
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None

## 2. Frame Format / 帧格式

### 2.1 Canonical Format / 标准格式

```
a<cmd><subcmd><param>b
```

- Frame head: `a` (lowercase, fixed)
- Frame tail: `b` (lowercase, fixed)
- `<cmd>`: uppercase letter `A-Z`
- `<subcmd>`: uppercase letter `A-Z`
- `<param>`: optional for read/state/fault control, mandatory for write command

Example:

```
aWA3.14b
```

### 2.2 Numeric Parsing Rules / 数值解析规则

`<param>` supports only signed decimal float text:

- Allowed: `123`, `-1.5`, `+0.25`, `.5`
- Not allowed: scientific notation (`1e-3`), spaces, extra symbols

### 2.3 Length and Packeting Rules / 长度与发包规则

- Parser RX buffer max length: 64 bytes
- Protocol minimum frame length: 4 bytes (`a` + cmd + subcmd + `b`)
- Param text max length: 31 chars
- Recommended: send one frame per packet with a small inter-frame gap

Important behavior:

- The parser extracts only the first valid `a...b` frame from one received chunk.
- If you concatenate multiple frames in one burst, trailing frames may be dropped.

## 3. Command Characters / 指令字符定义

| Command | Meaning (EN) | 含义 (中文) | Typical form |
|---|---|---|---|
| `W` | Write one parameter | 写单参数 | `aW<subcmd><value>b` |
| `R` | Read one parameter | 读单参数 | `aR<subcmd>b` |
| `R` + `X` | Read all parameters | 读全部参数 | `aRXb` |
| `Q` | Read runtime state summary | 读取运行状态摘要 | `aQXb` (subcmd placeholder) |
| `F` + `C` | Fault clear + soft diag reinit | 清故障计数并软重置诊断 | `aFCb` |

Notes:

- `Q` command ignores subcmd in execution, but frame format still requires one uppercase subcmd character.
- `F` currently supports only subcmd `C`.

## 4. Parameter Subcommands / 参数子命令

### 4.1 Full Reference Table / 完整参数表

| Subcmd | Parameter name | Type | Range | Default | Unit | W Example | R Example |
|---|---|---|---|---|---|---|---|
| `A` | target_angle_rad | float | [-12.566, 12.566] | 3.14 | rad | `aWA1.57b` | `aRAb` |
| `S` | angle_position_speed_rad_s | float | [0, 200] | 8.0 | rad/s | `aWS10b` | `aRSb` |
| `W` | sensor_sample_offset_percent | float | [0, 100] | 96.0 | % | `aWW96b` | `aRWb` |
| `L` | semantic_report_frequency_hz | uint | [1, 200] | 2 | Hz | `aWL10b` | `aRLb` |
| `H` | oscilloscope_report_frequency_hz | uint | [1, 200] | 50 | Hz | `aWH100b` | `aRHb` |
| `Y` | semantic_report_enabled | bool | 0 or 1 | 0 | - | `aWY1b` | `aRYb` |
| `Z` | oscilloscope_report_enabled | bool | 0 or 1 | 0 | - | `aWZ1b` | `aRZb` |
| `O` | oscilloscope_param_mask | uint | [0, 65535] | 24 (0x0018) | bitmask | `aWO56b` | `aROb` |
| `C` | pid_current_kp | float | [0, 50] | 0.0 | - | `aWC0.2b` | `aRCb` |
| `I` | pid_current_ki | float | [0, 50] | 0.0 | - | `aWI0.1b` | `aRIb` |
| `J` | pid_current_kd | float | [0, 10] | 0.0 | - | `aWJ0.01b` | `aRJb` |
| `G` | pid_angle_kp | float | [0, 50] | 2.0 | - | `aWG2.5b` | `aRGb` |
| `K` | pid_angle_ki | float | [0, 50] | 0.8 | - | `aWK0.9b` | `aRKb` |
| `N` | pid_angle_kd | float | [0, 10] | 0.01 | - | `aWN0.02b` | `aRNb` |
| `P` | pid_speed_kp | float | [0, 50] | 1.5 | - | `aWP3.0b` | `aRPb` |
| `U` | pid_speed_ki | float | [0, 50] | 0.8 | - | `aWU0.6b` | `aRUb` |
| `V` | pid_speed_kd | float | [0, 10] | 0.01 | - | `aWV0.05b` | `aRVb` |
| `M` | control_min_mech_angle_accum_delta_rad | float | >= 0 | 0.001 | rad | `aWM0.002b` | `aRMb` |
| `B` | control_angle_hold_integral_limit | float | >= 0 | 0.2 | - | `aWB0.3b` | `aRBb` |
| `E` | control_angle_hold_pid_deadband_rad | float | >= 0 | 0.005 | rad | `aWE0.004b` | `aREb` |
| `F` | control_speed_angle_transition_start_rad | float | >= 0 | 0.40 | rad | `aWF0.45b` | `aRFb` |
| `T` | control_speed_angle_transition_end_rad | float | >= 0 | 0.60 | rad | `aWT0.70b` | `aRTb` |
| `D` | control_mode | uint | 0 or 1 | 0 (FULL build default) | - | `aWD0b` | `aRDb` |
| `Q` | motor_enable | bool | 0 or 1 | 1 | - | `aWQ1b` | `aRQb` |
| `X` | read_all sentinel | - | R only | - | - | N/A | `aRXb` |

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

### 5.1 Feedback Byte (USART1) / 单字节反馈

| Code | Meaning |
|---|---|
| `O` | Frame parsed successfully |
| `E` | Frame format error (head/tail/cmd/subcmd/param parse failed) |
| `P` | Parameter invalid (unknown subcmd, missing write value, out of range) |
| `I` | Command invalid (unknown cmd) |
| `T` | Reserved timeout code (currently defined but not emitted) |

Execution order note:

- A syntactically correct but semantically wrong frame can produce two feedback bytes in sequence: first `O`, then `P` or `I`.

### 5.2 Debug Text Output (USART1) / 文本返回

When diagnostics output is enabled, responses are plain text lines on USART1, for example:

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
aWA3.14b    # write target_angle_rad
aRAb        # read target_angle_rad
aRXb        # read all parameters
aQXb        # read runtime state summary
aFCb        # clear fault counters + soft diag reinit
```

### 6.2 Typical parameter writes / 常用参数写入示例

```text
aWD0b       # control_mode = speed+angle
aWD1b       # control_mode = speed-only
aWQ1b       # motor enable
aWQ0b       # motor disable
aWW96b      # sensor sample offset percent = 96
aWL20b      # semantic report frequency = 20Hz
aWH100b     # osc report frequency = 100Hz
aWO63b      # osc mask bits 0..5 all on
aWP3.0b     # speed kp
aWU0.6b     # speed ki
aWV0.05b    # speed kd
```

### 6.3 Typical reads / 常用读取示例

```text
aRDb        # read control_mode
aRQb        # read motor_enable
aRWb        # read sensor_sample_offset_percent
aROb        # read oscilloscope_param_mask
aRPb        # read pid_speed_kp
aRMb        # read min_mech_angle_accum_delta
```

## 7. Common Error Cases / 常见错误与原因

| Frame | Expected feedback | Reason |
|---|---|---|
| `aWA1e-3b` | `E` | Scientific notation not supported |
| `aWAb` | `O` then `P` | Write command missing value |
| `aWZonb` | `E` | Non-numeric write value cannot be parsed as float |
| `aRxb` | `E` | subcmd must be uppercase `A-Z` |
| `a?A1b` | `E` | cmd must be uppercase `A-Z` |
| `aZA1b` | `O` then `I` | Unknown command group |
| `WA1b` | `E` | Missing frame head `a` |
| `aWA1` | `E` | Missing frame tail `b` |

## 8. Recommended Test Procedure / 推荐测试流程

1. Open two serial terminals:
   - Terminal A on USART2: send command frames.
   - Terminal B on USART1: watch feedback bytes and debug text output.
2. Send `aRXb` and confirm a full parameter dump appears on USART1.
3. Send one write command (for example `aWP3.0b`), verify:
   - USART1 receives `O`
   - USART1 prints updated parameter line.
4. Send `aQXb` to check runtime status summary.
5. Send `aFCb` and confirm fault counters are cleared.

Best practices:

- Send one frame at a time (do not concatenate multiple frames in one burst).
- Keep command/subcommand uppercase and frame delimiters lowercase (`a`, `b`).
- Keep write values inside valid ranges.
