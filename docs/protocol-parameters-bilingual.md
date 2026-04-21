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
- `<param>`: optional; command-specific meaning:
  - `P`/`S` with param = write, without param = read
  - `Y` must not carry param

Example:

```
aaPA3.14b
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
| `P` | Parameter channel | 参数通道 | `a<id>P<subcmd>[value]b` |
| `S` | State channel (ENABLE/DISABLE only) | 状态通道（仅开关态） | `a<id>S<subcmd>[0/1]b` |
| `Y` | System channel | 系统语义通道 | `a<id>Y<subcmd>b` |

Execution semantics:

- `P`: with value = write parameter; without value = read parameter.
- `S`: with value = write state; without value = read state.
- `Y`: no value allowed.

System subcommands:

| `Y` subcmd | Meaning |
|---|---|
| `R` | runtime summary |
| `C` | fault clear + soft diag reinit |

Compatibility note:

- This version is a clean protocol update and does not keep compatibility aliases for `W/R/Q/F`.

### 3.1 Build-Time Protocol Trimming / 编译期协议裁剪

Protocol availability is split into a fixed minimal set and optional groups controlled by compile macros in `foc/include/LS_Config/foc_cfg_feature_switches.h`.

Minimal set (always enabled, non-trimmable):

- `P`: `A/R/S/D`
- `S`: `M`
- `Y`: `R/C`

Optional groups:

| Macro | Affected subcommands |
|---|---|
| `FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET` | `P:W` |
| `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `P:L/H/O`, `S:S/O` |
| `FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING` | `P:C/I/J` |
| `FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING` | `P:G/K/N` |
| `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` | `P:P/U/V` |
| `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `P:M/B/E/F/T` |
| `FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH` | `P:Q/Y/Z`, `S:C` |
| `FOC_PROTOCOL_ENABLE_COGGING_COMP` | `S:G` |

When an optional subcommand is trimmed off, write/read on that subcommand returns parameter-invalid (`P`) after frame parse success (`O`).

Current implementation note:

- Cogging parameter symbols (`P:J/K/N`) are reserved in symbol definitions but not yet wired into runtime parameter read/write handlers.
- Cogging runtime control currently exposes the state subcommand `S:G` when `FOC_PROTOCOL_ENABLE_COGGING_COMP` is enabled.

## 4. Parameter Subcommands / 参数子命令

### 4.1 Full Reference Table / 完整参数表

Note: this table corresponds to FULL protocol profile. In trimmed builds, optional entries may be unavailable per section 3.1.

| Subcmd | Parameter name | Type | Range | Default | Unit | Write Example (`P`) | Read Example (`P`) |
|---|---|---|---|---|---|---|---|
| `A` | target_angle_rad | float | [-100, 100] | 3.14 | rad | `aaPA1.57b` | `aaPAb` |
| `R` | angle_position_speed_rad_s | float | [0, 36] | 18.0 | rad/s | `aaPR12b` | `aaPRb` |
| `S` | speed_only_speed_rad_s | float | [-36, 36] | 2.0 | rad/s | `aaPS-20b` | `aaPSb` |
| `W` | sensor_sample_offset_percent | float | [0, 100] | 96.0 | % | `aaPW96b` | `aaPWb` |
| `L` | semantic_report_frequency_hz | uint | [1, 200] | 2 | Hz | `aaPL20b` | `aaPLb` |
| `H` | oscilloscope_report_frequency_hz | uint | [1, 200] | 100 | Hz | `aaPH100b` | `aaPHb` |
| `O` | oscilloscope_param_mask | uint | [0, 65535] | 8 (0x0008) | bitmask | `aaPO63b` | `aaPOb` |
| `C` | pid_current_kp | float | [0, 50] | 1.2 | - | `aaPC0.2b` | `aaPCb` |
| `I` | pid_current_ki | float | [0, 50] | 12.0 | - | `aaPI0.1b` | `aaPIb` |
| `J` | pid_current_kd | float | [0, 10] | 0.0 | - | `aaPJ0.01b` | `aaPJb` |
| `G` | pid_angle_kp | float | [0, 50] | 2.0 | - | `aaPG2.5b` | `aaPGb` |
| `K` | pid_angle_ki | float | [0, 50] | 0.8 | - | `aaPK0.9b` | `aaPKb` |
| `N` | pid_angle_kd | float | [0, 10] | 0.01 | - | `aaPN0.02b` | `aaPNb` |
| `P` | pid_speed_kp | float | [0, 50] | 1.5 | - | `aaPP3.0b` | `aaPPb` |
| `U` | pid_speed_ki | float | [0, 50] | 0.6 | - | `aaPU0.6b` | `aaPUb` |
| `V` | pid_speed_kd | float | [0, 10] | 0.005 | - | `aaPV0.05b` | `aaPVb` |
| `M` | control_min_mech_angle_accum_delta_rad | float | >= 0 | 0.001 | rad | `aaPM0.002b` | `aaPMb` |
| `B` | control_angle_hold_integral_limit | float | >= 0 | 0.2 | - | `aaPB0.3b` | `aaPBb` |
| `E` | control_angle_hold_pid_deadband_rad | float | >= 0 | 0.005 | rad | `aaPE0.004b` | `aaPEb` |
| `F` | control_speed_angle_transition_start_rad | float | >= 0 | 0.60 | rad | `aaPF0.45b` | `aaPFb` |
| `T` | control_speed_angle_transition_end_rad | float | >= 0 | 1.00 | rad | `aaPT0.70b` | `aaPTb` |
| `D` | control_mode | uint | 0 or 1 | 0 (FULL build default) | - | `aaPD0b` | `aaPDb` |
| `Q` | current_soft_switch_mode | uint | 0/1/2 | 2 | - | `aaPQ2b` | `aaPQb` |
| `Y` | current_soft_switch_auto_open_iq_a | float | [0, 100] | 0.25 | A | `aaPY1.5b` | `aaPYb` |
| `Z` | current_soft_switch_auto_closed_iq_a | float | [0, 100] and >= `Y` | 0.80 | A | `aaPZ3.0b` | `aaPZb` |
| `X` | read_all sentinel | - | read only | - | - | N/A | `aaPXb` |

### 4.2 Control Mode Values / 控制模式值

- `0`: speed + angle mode
- `1`: speed-only mode

Build-trim constraint:

- FULL build: allows `0` and `1`
- SPEED_ONLY build: only `1` is accepted
- SPEED_ANGLE_ONLY build: only `0` is accepted

Speed parameter mapping:

- `R` (`angle_position_speed_rad_s`): speed reference used by speed+angle mode (non-negative limit)
- `S` (`speed_only_speed_rad_s`): speed reference used by speed-only mode (supports negative direction)

### 4.3 Current Soft Switch Mode Values / 电流软切换模式值

- `0`: OPEN (pure open-loop current model)
- `1`: CLOSED (pure current PID)
- `2`: AUTO (threshold+hysteresis with first-order blend)
- Blend time constants currently use compile-time macro `FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_DEFAULT_SEC` (not exposed as a `P` runtime parameter in this version).

### 4.4 Oscilloscope Mask Bits / 示波参数掩码位

| Bit | Hex | Field |
|---|---|---|
| 0 | 0x0001 | current_a |
| 1 | 0x0002 | current_b |
| 2 | 0x0004 | current_c |
| 3 | 0x0008 | angle_filtered |
| 4 | 0x0010 | angle_accum |
| 5 | 0x0020 | execution_time_us |

Default mask is `0x0008` (angle_filtered).

## 5. State Subcommands / 状态子命令

State channel uses command `S`, and state values are strict numeric `0` or `1`.

| Subcmd | State name | Default | Write Example (`S`) | Read Example (`S`) |
|---|---|---|---|---|
| `M` | motor_enable | 1 | `aaSM1b` | `aaSMb` |
| `S` | semantic_report_enabled | 0 | `aaSS1b` | `aaSSb` |
| `O` | oscilloscope_report_enabled | 1 | `aaSO1b` | `aaSOb` |
| `C` | current_soft_switch_enabled | 1 (macro default, effective value depends on feature/protocol switches) | `aaSC1b` | `aaSCb` |
| `G` | cogging_comp_enabled | 0 (follows `FOC_COGGING_COMP_ENABLE`) | `aaSG1b` | `aaSGb` |
| `X` | read_all sentinel | - | N/A | `aaSXb` |

## 6. Responses and Status Codes / 返回与状态码

### 6.1 Feedback Byte / 单字节反馈

| Code | Meaning |
|---|---|
| `O` | Frame parsed successfully |
| `E` | Frame format error (head/tail/cmd/subcmd/param parse failed) |
| `P` | Parameter invalid (unknown subcmd, out of range, illegal state value, illegal Y payload) |
| `I` | Command invalid (unknown cmd) |
| `T` | Reserved timeout code (currently defined but not emitted) |

Execution order note:

- A syntactically correct but semantically wrong frame can produce two feedback bytes in sequence: first `O`, then `P` or `I`.

### 6.2 Debug Text Output / 文本返回

When diagnostics output is enabled, responses are plain text lines on the configured output channel, for example:

```
parameter.pid_speed_kp=3.000
state.semantic_report_enabled=ENABLE
STATE SYS=1 COMM=1 REPORT=1 DIRTY=1 LAST=1 INIT=1 FAULT=NONE SENS_INV=0 PROTO_ERR=0 PARAM_ERR=0 CTRL_SKIP=0
FAULT_CTRL state=1 fault=NONE proto_err=0 param_err=0 ctrl_skip=0
```

Formatting rules:

- Float parameters: 3 decimal places
- Integer parameters: unsigned decimal text
- State output: `ENABLE` or `DISABLE`

Fault-state behavior:

- In FAULT state, debug stream periodic semantic/osc output is suppressed.
- Command-path diagnostics and explicit query/clear commands are still available.

## 7. Complete Command Examples / 完整命令示例

### 7.1 Core commands / 核心命令

```text
aaPA3.14b   # write target_angle_rad
aaPAb       # read target_angle_rad
aaPXb       # read all parameters
aaSM1b      # motor_enable = ENABLE
aaSMb       # read motor_enable
aaSXb       # read all states
aaYRb       # read runtime state summary
aaYCb       # clear fault counters + soft diag reinit
```

### 7.2 Typical parameter writes / 常用参数写入示例

```text
aaPD0b      # control_mode = speed+angle
aaPD1b      # control_mode = speed-only
aaPS-20b    # speed-only speed reference = -20rad/s
aaPR12b     # speed+angle mode speed limit/reference = 12rad/s
aaPW96b     # sensor sample offset percent = 96
aaPL20b     # semantic report frequency parameter example value = 20
aaPH100b    # osc report frequency parameter example value = 100
aaPO63b     # osc mask bits 0..5 all on
aaPP3.0b    # speed kp
aaPU0.6b    # speed ki
aaPV0.05b   # speed kd
aaPQ2b      # current_soft_switch_mode = AUTO
aaPY1.5b    # current_soft_switch_auto_open_iq_a = 1.5A
aaPZ3.0b    # current_soft_switch_auto_closed_iq_a = 3.0A
```

### 7.3 Typical state writes / 常用状态写入示例

```text
aaSM1b      # motor enable
aaSM0b      # motor disable
aaSS1b      # semantic report enable
aaSO1b      # osc report enable
aaSC1b      # current soft switch enable
aaSG1b      # cogging compensation enable (only when FOC_PROTOCOL_ENABLE_COGGING_COMP=ENABLE)
```

### 7.4 Typical reads / 常用读取示例

```text
aaPDb       # read control_mode
aaPSb       # read speed_only_speed_rad_s
aaPRb       # read angle_position_speed_rad_s
aaSMb       # read motor_enable
aaPWb       # read sensor_sample_offset_percent
aaPOb       # read oscilloscope_param_mask
aaPPb       # read pid_speed_kp
aaPMb       # read min_mech_angle_accum_delta
aaSCb       # read current_soft_switch_enabled
```

## 8. Common Error Cases / 常见错误与原因

| Frame | Expected feedback | Reason |
|---|---|---|
| `aaPA1e-3b` | `E` | Scientific notation not supported |
| `aaSS2b` | `O` then `P` | State write accepts only `0` or `1` |
| `aaSConb` | `E` | Non-numeric write value cannot be parsed as float |
| `aaPxb` | `E` | subcmd must be uppercase `A-Z` |
| `aa?A1b` | `E` | cmd must be uppercase `A-Z` |
| `aaRA1b` | `O` then `I` | Unknown command group (`R` no longer valid) |
| `2PA1b` | `E` | Missing frame head `a` |
| `aaPA1` | `E` | Missing frame tail `b` |

## 9. Recommended Test Procedure / 推荐测试流程

1. Open host tools bound to instance input/output channels.
2. Send `aaPXb` and confirm a full parameter dump appears on output channel.
3. Send one parameter write command (for example `aaPP3.0b`), verify:
  - output channel receives `O`
  - output channel prints updated parameter line.
4. Send one state write command (for example `aaSM1b`) and verify state output line.
5. Send `aaYRb` to check runtime status summary.
6. Send `aaYCb` and confirm fault counters are cleared.

Instance-specific serial terminal setup and channel wiring are documented in:
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

Best practices:

- Send one frame at a time (do not concatenate multiple frames in one burst).
- Keep command/subcommand uppercase and frame delimiters lowercase (`a`, `b`).
- Keep write values inside valid ranges.

