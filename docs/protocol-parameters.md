# 协议参数与测试指南

本文件面向上位机调试和联调测试，描述当前固件实际实现的协议规则。

---

## 1. 传输绑定

- 输入命令 RX 源（抽象）：Source1、Source2、Source3、Source4
- 统一输出通道（抽象）：反馈字节与调试文本的输出通道

实现说明：

- 库解析器支持最多 4 个输入源。具体实例可仅连接部分源（例如 Source1 和 Source2），其余保持为空/弱桩。

本文件仅定义协议语义；具体 UART/USB/CAN 映射归属各实例文档。

GD32F303 参考绑定见：`../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

---

## 2. 帧格式

### 2.1 标准格式

```
a<driver_id><cmd><subcmd><param>b
```

- 帧头：`a`（小写，固定）
- 帧尾：`b`（小写，固定）
- `<driver_id>`：一个字节，有效范围 `0x32-0x7E`（`'2'..'~'`），广播 `0xFF`
- `<cmd>`：大写字母 `A-Z`
- `<subcmd>`：大写字母 `A-Z`
- `<param>`：可选，命令特定的含义：
  - `P`/`C`/`S` 带参数表示写入，不带参数表示读取
  - `Y` 不能携带参数

示例：

```
aaPA3.14b
```

### 2.2 驱动器编号规则

- 本地驱动器 ID 默认值（当前代码基线）：`0x61`（`'a'`）
- 单播有效范围：`0x32-0x7E`
- 广播 ID：`0xFF`
- 无效 `driver_id` 格式引发帧错误（`E`）
- 有效但非本机 ID 的帧静默丢弃（无状态码返回）

注意：

- 本文档示例使用当前默认本地 ID（`'a'`）。若修改 `FOC_PROTOCOL_LOCAL_DRIVER_ID_DEFAULT`，需同步更新测试帧。

### 2.3 数值解析规则

`<param>` 仅支持有符号十进制浮点文本：

- 允许：`123`、`-1.5`、`+0.25`、`.5`
- 不允许：科学计数法（`1e-3`）、空格、额外符号

### 2.4 长度与发包规则

- 解析器 RX 缓冲区最大长度：64 字节
- 协议最小帧长度：5 字节（`a` + driver_id + cmd + subcmd + `b`）
- 参数文本最大长度：31 字符
- 建议：每包只发一帧，帧间留适当间隔

重要行为：

- 解析器只从一次接收中提取第一个有效的 `a...b` 帧。
- 若在单次突发中拼接多个帧，后续帧可能被丢弃。
- 若 `driver_id` 格式有效但不匹配本机 ID 且非广播，帧被静默丢弃。

## 3. 指令字符定义

| 命令 | 含义 | 典型格式 |
|------|------|---------|
| `P` | 运行参数通道（读写） | `a<id>P<subcmd>[value]b` |
| `C` | 调优/配置参数通道（读写） | `a<id>C<subcmd>[value]b` |
| `S` | 状态通道（开关量，读写） | `a<id>S<subcmd>[0/1]b` |
| `Y` | 系统语义通道（只读/执行） | `a<id>Y<subcmd>b` |

执行语义：

- `P/C`：带值=写参数；不带值=读参数
- `S`：带值=写状态；不带值=读状态
- `Y`：不允许带值

### 3.1 全子命令索引

按字母排序，快速查找每个子命令所属命令组和作用。

| 子命令 | 命令组 | 简要描述 |
|--------|--------|---------|
| `P:A` | 运行参数 | target_angle_rad |
| `P:D` | 运行参数 | control_mode |
| `P:H` | 运行参数 | oscilloscope_report_frequency_hz |
| `P:L` | 运行参数 | semantic_report_frequency_hz |
| `P:O` | 运行参数 | oscilloscope_param_mask |
| `P:R` | 运行参数 | angle_position_speed_rad_s |
| `P:S` | 运行参数 | speed_only_speed_rad_s |
| `P:W` | 运行参数 | sensor_sample_offset_percent |
| `P:X` | 运行参数 | 读取全部（批读哨兵） |
| `C:A` | 配置参数 | cogging_comp_speed_gate_rad_s |
| `C:B` | 配置参数 | control_angle_hold_integral_limit |
| `C:C` | 配置参数 | pid_current_kp |
| `C:E` | 配置参数 | control_angle_hold_pid_deadband_rad |
| `C:F` | 配置参数 | control_speed_angle_transition_start_rad |
| `C:G` | 配置参数 | pid_angle_kp |
| `C:I` | 配置参数 | pid_current_ki |
| `C:J` | 配置参数 | pid_current_kd |
| `C:K` | 配置参数 | pid_angle_ki |
| `C:L` | 配置参数 | cogging_comp_iq_limit_a |
| `C:M` | 配置参数 | control_min_mech_angle_accum_delta_rad |
| `C:N` | 配置参数 | pid_angle_kd |
| `C:O` | 配置参数 | current_soft_switch_auto_closed_iq_a |
| `C:P` | 配置参数 | pid_speed_kp |
| `C:Q` | 配置参数 | current_soft_switch_mode |
| `C:R` | 配置参数 | pid_speed_kd |
| `C:S` | 配置参数 | pid_speed_ki |
| `C:T` | 配置参数 | control_speed_angle_transition_end_rad |
| `C:X` | 配置参数 | 读取全部（批读哨兵） |
| `C:Y` | 配置参数 | cogging_calib_gain_k |
| `C:Z` | 配置参数 | current_soft_switch_auto_open_iq_a |
| `S:C` | 状态 | current_soft_switch_enabled |
| `S:G` | 状态 | cogging_comp_enabled |
| `S:M` | 状态 | motor_enable |
| `S:O` | 状态 | oscilloscope_report_enabled |
| `S:S` | 状态 | semantic_report_enabled |
| `S:X` | 状态 | 读取全部（批读哨兵） |
| `Y:C` | 系统 | 故障清除 + 软诊断重初始化 |
| `Y:D` | 系统 | 导出齿槽表 |
| `Y:G` | 系统 | 启动齿槽标定 |
| `Y:I` | 系统 | 运行时电机参数重初始化 |
| `Y:R` | 系统 | 运行时摘要 |
| `Y:T` | 系统 | 以 C 代码形式导出齿槽表 |
| `Y:X` | 系统 | 系统信息（只读） |

### 3.2 编译期协议裁剪

协议可用性由 `foc_core/include/LS_Config/foc_cfg_feature_switches.h` 中的编译宏控制。每个子命令的裁剪宏在第 4、5 节的参数表中标注。

不可裁剪的固定最小集：

| 命令组 | 固定子命令 |
|--------|-----------|
| `P` | `A`、`R`、`S`、`D`（读写） |
| `C` | （无固定子命令） |
| `S` | `M`（读写） |
| `Y` | `R`、`C`（只读/执行） |
| 所有 `*:X` | `FOC_PROTOCOL_ENABLE_BATCH_READ`（总控宏，参见第 4 节） |

当子命令对应的裁剪宏禁用后，对该子命令的写/读在帧解析成功后返回参数无效（`P`）。

## 4. 参数子命令

### 4.1 P 组：运行参数

| 子命令 | 参数名 | 类型 | 范围 | 默认值 | 单位 | 裁剪宏 | 写示例 | 读示例 |
|--------|--------|------|------|--------|------|--------|--------|--------|
| `A` | target_angle_rad | float | [-100, 100] | 3.14 | rad | （固定） | `aaPA1.57b` | `aaPAb` |
| `R` | angle_position_speed_rad_s | float | [0, 36] | 18.0 | rad/s | （固定） | `aaPR12b` | `aaPRb` |
| `S` | speed_only_speed_rad_s | float | [-36, 36] | 2.0 | rad/s | （固定） | `aaPS-20b` | `aaPSb` |
| `D` | control_mode | uint | 0 或 1 | 0 | - | （固定） | `aaPD0b` | `aaPDb` |
| `W` | sensor_sample_offset_percent | float | [0, 100] | 45.0 | % | `FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE` | `aaPW45b` | `aaPWb` |
| `L` | semantic_report_frequency_hz | uint | [1, 200] | 2 | Hz | `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `aaPL20b` | `aaPLb` |
| `H` | oscilloscope_report_frequency_hz | uint | [1, 200] | 100 | Hz | `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `aaPH100b` | `aaPHb` |
| `O` | oscilloscope_param_mask | uint | [0, 65535] | 30872 (0x7898) | bitmask | `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `aaPO63b` | `aaPOb` |
| `X` | read_all 哨兵 | - | 只读 | - | - | `FOC_PROTOCOL_ENABLE_BATCH_READ` | 不可用 | `aaPXb` |

### 4.2 C 组：调优/配置参数

| 子命令 | 参数名 | 类型 | 范围 | 默认值 | 单位 | 裁剪宏 | 写示例 | 读示例 |
|--------|--------|------|------|--------|------|--------|--------|--------|
| `C` | pid_current_kp | float | [0, 50] | 4.0 | - | `FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING` | `aaCC2.0b` | `aaCCb` |
| `I` | pid_current_ki | float | [0, 50] | 60.0 | - | `FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING` | `aaCI10b` | `aaCIb` |
| `J` | pid_current_kd | float | [0, 10] | 0.00 | - | `FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING` | `aaCJ0.01b` | `aaCJb` |
| `G` | pid_angle_kp | float | [0, 50] | 2.0 | - | `FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING` | `aaCG2.5b` | `aaCGb` |
| `K` | pid_angle_ki | float | [0, 50] | 0.8 | - | `FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING` | `aaCK0.9b` | `aaCKb` |
| `N` | pid_angle_kd | float | [0, 10] | 0.01 | - | `FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING` | `aaCN0.02b` | `aaCNb` |
| `P` | pid_speed_kp | float | [0, 50] | 0.8 | - | `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` | `aaCP1.5b` | `aaCPb` |
| `S` | pid_speed_ki | float | [0, 50] | 0.6 | - | `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` | `aaCS0.6b` | `aaCSb` |
| `R` | pid_speed_kd | float | [0, 10] | 0.005 | - | `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` | `aaCR0.005b` | `aaCRb` |
| `M` | control_min_mech_angle_accum_delta_rad | float | >= 0 | 0.001 | rad | `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `aaCM0.002b` | `aaCMb` |
| `B` | control_angle_hold_integral_limit | float | >= 0 | 0.05 | - | `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `aaCB0.10b` | `aaCBb` |
| `E` | control_angle_hold_pid_deadband_rad | float | >= 0 | 0.005 | rad | `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `aaCE0.004b` | `aaCEb` |
| `F` | control_speed_angle_transition_start_rad | float | >= 0 | 0.60 | rad | `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `aaCF0.45b` | `aaCFb` |
| `T` | control_speed_angle_transition_end_rad | float | >= 0 | 1.00 | rad | `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `aaCT0.70b` | `aaCTb` |
| `Q` | current_soft_switch_mode | uint | 0/1/2 | 2 | - | `FOC_CURRENT_SOFT_SWITCH_ENABLE` | `aaCQ2b` | `aaCQb` |
| `Z` | current_soft_switch_auto_open_iq_a | float | [0, 100] | 0.20 | A | `FOC_CURRENT_SOFT_SWITCH_ENABLE` | `aaCZ0.5b` | `aaCZb` |
| `O` | current_soft_switch_auto_closed_iq_a | float | [0, 100] 且 >= open | 0.50 | A | `FOC_CURRENT_SOFT_SWITCH_ENABLE` | `aaCO1.0b` | `aaCOb` |
| `L` | cogging_comp_iq_limit_a | float | [0, 10] | 0.50 | A | `FOC_COGGING_COMP_ENABLE` | `aaCL1.0b` | `aaCLb` |
| `A` | cogging_comp_speed_gate_rad_s | float | [0, 36] | 12.0 | rad/s | `FOC_COGGING_COMP_ENABLE` | `aaCA8.0b` | `aaCAb` |
| `Y` | cogging_calib_gain_k | float | >= 0 | 0.05 | - | `FOC_COGGING_CALIB_ENABLE` | `aaCY0.10b` | `aaCYb` |
| `X` | read_all 哨兵 | - | 只读 | - | - | `FOC_PROTOCOL_ENABLE_BATCH_READ` | 不可用 | `aaCXb` |

### 4.3 控制模式值

- `0`：速度+角度模式
- `1`：纯速度模式

构建裁剪约束：

- 完整构建：允许 `0` 和 `1`
- 纯速度构建：仅接受 `1`
- 速度角度构建：仅接受 `0`

速度参数映射：

- `P:R`（`angle_position_speed_rad_s`）：速度+角度模式使用的速度参考（非负限幅）
- `P:S`（`speed_only_speed_rad_s`）：纯速度模式使用的速度参考（支持负方向）

### 4.4 电流环 PID 抗饱和算法

电流环 PID 使用**条件积分（Conditional Integration）**抗饱和策略：

```
如果（输出未饱和 OR 误差方向与饱和方向相反）
    正常积分；
否则
    冻结（回滚本次积分增量）；
```

并附加积分钳位安全网 `|积分| ≤ |输出最大 / ki|` 作为极端情况保护。

与 back-calculation 相比，条件积分允许积分器在瞬态响应中自由建立，更适合电流环小 KP + 大 KI 的参数风格。

### 4.5 电流软切换模式值

- `0`：开环（纯开环电流模型）
- `1`：闭环（纯电流 PID）
- `2`：自动（阈值+滞回，一阶混合）

混合时间常数当前使用编译期宏 `FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_DEFAULT_SEC`（本版本不暴露为运行时参数）。

### 4.6 示波参数掩码位

| 位 | 十六进制 | 字段 | 默认启用 |
|----|---------|------|---------|
| 0 | 0x0001 | current_a | 否 |
| 1 | 0x0002 | current_b | 否 |
| 2 | 0x0004 | current_c | 否 |
| 3 | 0x0008 | angle_active_mech | 是 |
| 4 | 0x0010 | angle_standby_mech | 是 |
| 5 | 0x0020 | execution_time_us | 否 |
| 6 | 0x0040 | vbus_voltage | 否 |
| 7 | 0x0080 | iq_target | 是 |
| 8 | 0x0100 | iq_measured | 否 |
| 9 | 0x0200 | current_a_raw | 否 |
| 10 | 0x0400 | current_b_raw | 否 |
| 11 | 0x0800 | speed_active_mech | 是 |
| 12 | 0x1000 | speed_standby_mech | 是 |
| 13 | 0x2000 | angle_active_elec | 是 |
| 14 | 0x4000 | angle_standby_elec | 是 |
| 15 | 0x8000 | （保留） | 否 |

默认掩码：由 `DEBUG_STREAM_OSC_DEFAULT_SHOW_*` 编译宏组合生成（当前默认含 bit3/bit4/bit7/bit11/bit12/bit13/bit14，即 `0x7898`）。

说明：

- `angle_*_mech`：活跃/备用源的机械角度（rad）
- `angle_*_elec`：活跃/备用源的电角度（rad）
- 不同源的原生角度类型由 Source Manager 统一封装（Encoder 原生机械角度、SMO/OpenLoop 原生电角度），示波器输出始终同时提供机械与电角度两种视图。

### 4.7 语义调试行说明

启用语义报告（`S:S=1`）后，调试流按周期输出以下行（行 0~8 共 9 行）：

| 行 | 标签 | 输出格式示例 | 说明 |
|----|------|-------------|------|
| 0 | current_a | `measurement.phase_current_a_ampere=0.123` | A 相电流 |
| 1 | current_b | `measurement.phase_current_b_ampere=-0.456` | B 相电流 |
| 2 | current_c | `measurement.phase_current_c_ampere=-0.333` | C 相电流 |
| 3 | angle_raw | `measurement.mech_angle_raw_rad=1.570` | 机械角度原始值 |
| 4 | angle_filtered | `measurement.mech_angle_filtered_rad=1.571` | 机械角度滤波后值 |
| 5 | vbus_raw | `measurement.vbus_voltage_raw_v=23.800` | VBUS 原始电压 |
| 6 | vbus_filtered | `measurement.vbus_voltage_filtered_v=23.900` | VBUS 滤波后电压 |
| 7 | exec_time | `control.execution_time_us=15.200` | 调度器 tick 执行时间 |
| 8 | current_loop_time | `control.current_loop_execution_time_us=8.500` | 电流环 ISR 执行时间 |

传感器无效时跳过对应行，末尾以一个空行结束帧。

## 5. 状态子命令

状态通道使用命令 `S`，状态值为严格数字 `0` 或 `1`。

| 子命令 | 状态名 | 默认值 | 裁剪宏 | 写示例 | 读示例 |
|--------|--------|--------|--------|--------|--------|
| `M` | motor_enable | 1 | （固定） | `aaSM1b` | `aaSMb` |
| `S` | semantic_report_enabled | 0 | `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `aaSS1b` | `aaSSb` |
| `O` | oscilloscope_report_enabled | 0 | `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `aaSO1b` | `aaSOb` |
| `C` | current_soft_switch_enabled | 1 | `FOC_CURRENT_SOFT_SWITCH_ENABLE` | `aaSC1b` | `aaSCb` |
| `G` | cogging_comp_enabled | 0 | `FOC_COGGING_COMP_ENABLE` | `aaSG1b` | `aaSGb` |
| `X` | read_all 哨兵 | - | `FOC_PROTOCOL_ENABLE_BATCH_READ` | 不可用 | `aaSXb` |

## 6. 返回与状态码

### 6.1 反馈字节

| 码 | 含义 |
|----|------|
| `O` | 帧解析成功 |
| `E` | 帧格式错误（帧头/尾/命令/子命令/参数解析失败） |
| `P` | 参数无效（未知子命令、超出范围、非法状态值、非法 Y 载荷） |
| `I` | 命令无效（未知命令） |
| `T` | 保留超时码（当前已定义但未使用） |

执行顺序说明：

- 一个语法正确但语义错误的帧可能依次产生两个反馈字节：先 `O`，后 `P` 或 `I`。

### 6.2 文本返回

当调试输出启用时，响应为配置输出通道上的纯文本行，例如：

```
parameter.pid_speed_kp=3.000
config.pid_speed_kp=3.000
state.semantic_report_enabled=ENABLE
STATE RUN=1 FLT=0 INIT=0xFFFF/0x0000 SENS_INV=0 PROTO_ERR=0 PARAM_ERR=0 CTRL_SKIP=0
```

文本前缀规则：

- `P` 组参数输出：`parameter.<name>=<value>`
- `C` 组配置参数输出：`config.<name>=<value>`
- `S` 组状态输出：`state.<name>=ENABLE/DISABLE`
- `Y:X` 系统信息输出：`system.<name>=<value>`

`Y:X` 额外暴露当前 Source/Control 架构状态，均为只读：

| 字段 | 说明 |
|------|------|
| `system.source.active` | Source Manager 当前 active source id |
| `system.source.standby` | Source Manager 当前 standby source id |
| `system.source.region` | 当前 low/high/full control region |
| `system.source.region_state` | 当前速域切换状态机状态 |
| `system.source.config_valid` | 当前 low/high source 切换配置是否有效 |
| `system.source.switching` | Source Manager 是否处于切换消抖窗口 |
| `system.active_source.valid` | 当前 active source snapshot 是否有效 |
| `system.openloop.state` | OpenLoop source 私有状态 |

格式化规则：

- 浮点参数：3 位小数
- 整数参数：无符号十进制文本
- 状态输出：`ENABLE` 或 `DISABLE`

故障状态行为：

- 在 FAULT 状态下，调试流定期语义/示波输出被抑制。
- 命令路径诊断和显式查询/清除命令仍可用。

## 7. 常见错误与原因

| 帧 | 预期反馈 | 原因 |
|----|---------|------|
| `aaPA1e-3b` | `E` | 不支持科学计数法 |
| `aaSS2b` | `O` 然后 `P` | 状态写入仅接受 `0` 或 `1` |
| `aaSConb` | `E` | 非数字写入值无法解析为浮点数 |
| `aaPxb` | `E` | 子命令必须为大写 `A-Z` |
| `aa?A1b` | `E` | 命令必须为大写 `A-Z` |
| `aaRA1b` | `O` 然后 `I` | 未知命令组（`R` 已不再有效） |
| `2PA1b` | `E` | 缺少帧头 `a` |
| `aaPA1` | `E` | 缺少帧尾 `b` |

## 8. 推荐测试流程

1. 打开绑定到实例输入/输出通道的主机工具。
2. 发送 `aaPXb` 确认 P 组参数转储出现在输出通道。
3. 发送一个参数写入命令（例如 `aaCP1.5b`），验证：
   - 输出通道收到 `O`
   - 输出通道打印 `config.pid_speed_kp=1.500`
4. 发送一个状态写入命令（例如 `aaSM1b`）并验证状态输出行。
5. 发送 `aaYRb` 检查运行时状态摘要。
6. 发送 `aaYCb` 确认故障计数器已清除。

实例特定的串口终端设置和通道布线文档见：
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

最佳实践：

- 一次只发送一帧（不要在单次突发中拼接多帧）。
- 保持命令/子命令大写，帧分隔符小写（`a`、`b`）。
- 将写入值保持在有效范围内。

---

## 附录 A：完整命令参考

### A.1 核心命令

```text
aaPA3.14b   # 写入 target_angle_rad（P:A）
aaPAb       # 读取 target_angle_rad（P:A）
aaPXb       # 读取所有 P 组参数（P:X）
aaSM1b      # motor_enable = ENABLE（S:M）
aaSMb       # 读取 motor_enable（S:M）
aaSXb       # 读取所有状态（S:X）
aaYRb       # 读取运行时状态摘要（Y:R）
aaYCb       # 清除故障计数器 + 软诊断重初始化（Y:C）
aaYIb       # 运行时电机参数重初始化（Y:I）
aaYXb       # 读取系统参数信息（Y:X）
```

### A.2 常用参数写入

```text
# P 组（运行参数）
aaPD0b      # control_mode = 速度+角度（P:D）
aaPD1b      # control_mode = 纯速度（P:D）
aaPS-20b    # 纯速度速度参考 = -20rad/s（P:S）
aaPR12b     # 速度+角度模式速度限制/参考 = 12rad/s（P:R）
aaPW45b     # 传感器采样偏移百分比 = 45（P:W）
aaPL20b     # 语义报告频率 = 20（P:L）
aaPH100b    # 示波报告频率 = 100（P:H）
aaPO63b     # 示波掩码位 0..5 全开（P:O）

# C 组（调优/配置参数）
aaCP1.5b    # pid_speed_kp（C:P）
aaCS0.6b    # pid_speed_ki（C:S）
aaCR0.005b  # pid_speed_kd（C:R）
aaCC2.0b    # pid_current_kp（C:C）
aaCG2.5b    # pid_angle_kp（C:G）
aaCQ2b      # current_soft_switch_mode = 自动（C:Q）
aaCZ0.5b    # current_soft_switch_auto_open_iq_a = 0.5A（C:Z）
aaCO1.0b    # current_soft_switch_auto_closed_iq_a = 1.0A（C:O）
aaCY0.10b   # cogging_calib_gain_k = 0.10（C:Y）
aaCL1.0b    # cogging_comp_iq_limit_a = 1.0A（C:L）
aaCA8.0b    # cogging_comp_speed_gate_rad_s = 8.0（C:A）
```

### A.3 常用状态写入

```text
aaSM1b      # 电机使能（S:M）
aaSM0b      # 电机关闭（S:M）
aaSS1b      # 语义报告使能（S:S）
aaSO1b      # 示波报告使能（S:O）
aaSC1b      # 电流软切换使能（S:C）
aaSG1b      # 齿槽补偿使能（S:G）
aaYGb       # 启动运行时齿槽标定（Y:G）
aaYDb       # 导出齿槽补偿表到串口（Y:D）
aaYTb       # 以 C 代码形式导出齿槽补偿表（Y:T）
```

### A.4 常用读取

```text
aaPDb       # 读取 control_mode（P:D）
aaPSb       # 读取 speed_only_speed_rad_s（P:S）
aaPRb       # 读取 angle_position_speed_rad_s（P:R）
aaSMb       # 读取 motor_enable（S:M）
aaPWb       # 读取 sensor_sample_offset_percent（P:W）
aaPOb       # 读取 oscilloscope_param_mask（P:O）
aaCPb       # 读取 pid_speed_kp（C:P）
aaCMb       # 读取 min_mech_angle_accum_delta（C:M）
aaSCb       # 读取 current_soft_switch_enabled（S:C）
aaCYb       # 读取 cogging_calib_gain_k（C:Y）
aaYXb       # 读取系统信息（Y:X）
