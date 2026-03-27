# Protocol and Runtime Parameters (中英文说明)

## 1. Overview / 概述

EN:
- This project uses a transport-agnostic protocol parser module.
- Any communication channel can feed complete frames into the parser as long as it provides frame buffers.
- The current platform input source is USART2 DMA + IDLE + double-buffer.

中文：
- 本项目使用与传输介质解耦的协议解析模块。
- 只要通信通道能提供完整帧缓存，即可接入该解析器。
- 当前平台输入来源为 USART2 的 DMA + IDLE + 双缓冲。

## 2. Frame Format / 帧格式

EN:
- Default frame: `a <cmd> <subcmd> <param> b`
- Example: `aWA3.141593b`
- Constraints:
  - One command updates one parameter only.
  - Numeric parameter supports signed decimal float only.
  - Scientific notation is not supported.

中文：
- 默认帧格式：`a <cmd> <subcmd> <param> b`
- 示例：`aWA3.141593b`
- 约束：
  - 一条命令只修改一个参数。
  - 参数仅支持带符号十进制浮点。
  - 不支持科学计数法。

## 3. Command Set / 命令集

EN:
- `W`: write one parameter
- `R`: read one parameter
- `R + X`: read all parameters
- `Q`: read runtime state summary

中文：
- `W`：写单参数
- `R`：读单参数
- `R + X`：读全部参数
- `Q`：读运行状态摘要

## 4. Sub-command Mapping / 子命令映射

| Subcmd | Parameter Name | Description |
|---|---|---|
| A | target_angle_rad | Position target angle (rad) |
| S | angle_position_speed_rad_s | Speed limit for position loop blending (rad/s) |
| L | debug_low_frequency_divider | Low-frequency semantic output divider |
| H | debug_high_frequency_divider | High-frequency oscilloscope output divider |
| Y | debug_low_frequency_enabled | Enable low-frequency semantic output (0/1) |
| Z | debug_high_frequency_enabled | Enable high-frequency output (0/1) |
| C | pid_current_kp | Current loop PID Kp |
| I | pid_current_ki | Current loop PID Ki |
| J | pid_current_kd | Current loop PID Kd |
| G | pid_angle_kp | Angle hold PID Kp |
| K | pid_angle_ki | Angle hold PID Ki |
| N | pid_angle_kd | Angle hold PID Kd |
| P | pid_speed_kp | Speed loop PID Kp |
| U | pid_speed_ki | Speed loop PID Ki |
| V | pid_speed_kd | Speed loop PID Kd |
| M | control_min_mech_angle_accum_delta_rad | Min mechanical delta for accumulation |
| B | control_angle_hold_integral_limit | Integral clamp for angle hold PID |
| E | control_angle_hold_pid_deadband_rad | Deadband for angle hold PID |
| F | control_speed_angle_transition_start_rad | Speed-angle blend start threshold |
| T | control_speed_angle_transition_end_rad | Speed-angle blend end threshold |
| X | (read-all sentinel) | Only used with read command |

## 5. Default Runtime Parameters / 默认运行参数

EN:
- target_angle_rad = 3.14
- angle_position_speed_rad_s = 16.0
- pid_current = (0.0, 0.0, 0.0)
- pid_angle = (2.0, 0.8, 0.01)
- pid_speed = (3.0, 0.5, 0.0)
- control_min_mech_angle_accum_delta_rad = 0.001
- control_angle_hold_integral_limit = 0.2
- control_angle_hold_pid_deadband_rad = 0.002
- control_speed_angle_transition_start_rad = 0.40
- control_speed_angle_transition_end_rad = 0.60

中文：
- target_angle_rad = 3.14
- angle_position_speed_rad_s = 16.0
- pid_current = (0.0, 0.0, 0.0)
- pid_angle = (2.0, 0.8, 0.01)
- pid_speed = (3.0, 0.5, 0.0)
- control_min_mech_angle_accum_delta_rad = 0.001
- control_angle_hold_integral_limit = 0.2
- control_angle_hold_pid_deadband_rad = 0.002
- control_speed_angle_transition_start_rad = 0.40
- control_speed_angle_transition_end_rad = 0.60

## 6. Output Semantics / 输出语义

EN:
- Read responses use full semantic names, not abbreviations.
- Example: `parameter.pid_speed_kp=3.000000`

中文：
- 读取输出使用完整语义参数名，不再使用缩写。
- 示例：`parameter.pid_speed_kp=3.000000`

## 7. Status Code / 状态码

EN:
- `O`: parser/frame ok
- `E`: frame format error
- `I`: invalid command
- `P`: invalid parameter

中文：
- `O`：帧解析正常
- `E`：帧格式错误
- `I`：无效命令
- `P`：无效参数
