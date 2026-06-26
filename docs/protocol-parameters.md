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
  - `P`/`S` 带参数表示写入，不带参数表示读取
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
| `P` | 参数通道 | `a<id>P<subcmd>[value]b` |
| `S` | 状态通道（仅开关态） | `a<id>S<subcmd>[0/1]b` |
| `Y` | 系统语义通道 | `a<id>Y<subcmd>b` |

执行语义：

- `P`：带值=写参数；不带值=读参数
- `S`：带值=写状态；不带值=读状态
- `Y`：不允许带值

系统子命令：

| `Y` 子命令 | 含义 |
|-----------|------|
| `R` | 运行时摘要 |
| `C` | 故障清除 + 软诊断重初始化 |
| `I` | 运行时电机参数重初始化（方向/极对数/零位重新标定） |
| `G` | 启动齿槽标定（仅当 `FOC_COGGING_CALIB_ENABLE`） |
| `D` | 导出齿槽表（仅当 `FOC_COGGING_CALIB_ENABLE`） |
| `T` | 以 C 代码形式导出齿槽表（仅当 `FOC_COGGING_CALIB_ENABLE`） |

兼容性说明：

- 本版本为全新协议更新，不再为 `W/R/Q/F` 保留兼容别名。

### 3.1 编译期协议裁剪

协议可用性分为固定最小集和可选组，由 `foc_core/include/LS_Config/foc_cfg_feature_switches.h` 中的编译宏控制。

最小集（始终启用，不可裁剪）：

- `P`：`A/R/S/D`
- `S`：`M`
- `Y`：`R/C`

可选组：

| 宏 | 影响的子命令 |
|----|------------|
| `FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET` | `P:W` |
| `FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT` | `P:L/H/O`、`S:S/O` |
| `FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING` | `P:C/I/J` |
| `FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING` | `P:G/K/N` |
| `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` | `P:P/U/V` |
| `FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING` | `P:M/B/E/F/T` |
| `FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH` | `P:Q/Z/Y`、`S:C` |
| `FOC_PROTOCOL_ENABLE_COGGING_COMP` | `P:U/V`、`S:G`、`Y:G`、`Y:D`、`Y:T` |

当可选子命令被裁剪后，对该子命令的写/读在帧解析成功后返回参数无效（`P`）。

注意：

- `P:k` 子命令（齿槽标定增益）由 `FOC_COGGING_CALIB_ENABLE` 守卫，不由 `FOC_PROTOCOL_ENABLE_COGGING_COMP` 守卫。

## 4. 参数子命令

### 4.1 完整参数表

说明：下表对应完整协议配置。在裁剪构建中，可选条目按第 3.1 节可能不可用。

| 子命令 | 参数名 | 类型 | 范围 | 默认值 | 单位 | 写示例（`P`） | 读示例（`P`） |
|--------|--------|------|------|--------|------|--------------|-------------|
| `A` | target_angle_rad | float | [-100, 100] | 3.14 | rad | `aaPA1.57b` | `aaPAb` |
| `R` | angle_position_speed_rad_s | float | [0, 36] | 18.0 | rad/s | `aaPR12b` | `aaPRb` |
| `S` | speed_only_speed_rad_s | float | [-36, 36] | 2.0 | rad/s | `aaPS-20b` | `aaPSb` |
| `W` | sensor_sample_offset_percent | float | [0, 100] | 45.0 | % | `aaPW45b` | `aaPWb` |
| `L` | semantic_report_frequency_hz | uint | [1, 200] | 2 | Hz | `aaPL20b` | `aaPLb` |
| `H` | oscilloscope_report_frequency_hz | uint | [1, 200] | 100 | Hz | `aaPH100b` | `aaPHb` |
| `O` | oscilloscope_param_mask | uint | [0, 65535] | 779 (0x030B) | bitmask | `aaPO63b` | `aaPOb` |
| `C` | pid_current_kp | float | [0, 50] | 4.0 | - | `aaPC2.0b` | `aaPCb` |
| `I` | pid_current_ki | float | [0, 50] | 60.0 | - | `aaPI10b` | `aaPIb` |
| `J` | pid_current_kd | float | [0, 10] | 0.00 | - | `aaPJ0.01b` | `aaPJb` |
| `G` | pid_angle_kp | float | [0, 50] | 2.0 | - | `aaPG2.5b` | `aaPGb` |
| `K` | pid_angle_ki | float | [0, 50] | 0.8 | - | `aaPK0.9b` | `aaPKb` |
| `N` | pid_angle_kd | float | [0, 10] | 0.01 | - | `aaPN0.02b` | `aaPNb` |
| `P` | pid_speed_kp | float | [0, 50] | 0.8 | - | `aaPP1.5b` | `aaPPb` |
| `U` | pid_speed_ki | float | [0, 50] | 0.6 | - | `aaPU0.6b` | `aaPUb` |
| `V` | pid_speed_kd | float | [0, 10] | 0.005 | - | `aaPV0.005b` | `aaPVb` |
| `U`\* | cogging_comp_iq_limit_a | float | [0, 10] | 0.50（见 `FOC_COGGING_COMP_IQ_LIMIT_A`） | A | `aaPU1.0b` | `aaPUb` |
| `V`\* | cogging_comp_speed_gate_rad_s | float | [0, 36] | 12.0（见 `FOC_COGGING_COMP_SPEED_GATE_RAD_S`） | rad/s | `aaPV8.0b` | `aaPVb` |
| `M` | control_min_mech_angle_accum_delta_rad | float | >= 0 | 0.001 | rad | `aaPM0.002b` | `aaPMb` |
| `B` | control_angle_hold_integral_limit | float | >= 0 | 0.05 | - | `aaPB0.10b` | `aaPBb` |
| `E` | control_angle_hold_pid_deadband_rad | float | >= 0 | 0.005 | rad | `aaPE0.004b` | `aaPEb` |
| `F` | control_speed_angle_transition_start_rad | float | >= 0 | 0.60 | rad | `aaPF0.45b` | `aaPFb` |
| `T` | control_speed_angle_transition_end_rad | float | >= 0 | 1.00 | rad | `aaPT0.70b` | `aaPTb` |
| `D` | control_mode | uint | 0 或 1 | 0（完整构建默认） | - | `aaPD0b` | `aaPDb` |
| `Q` | current_soft_switch_mode | uint | 0/1/2 | 2 | - | `aaPQ2b` | `aaPQb` |
| `Z` | current_soft_switch_auto_open_iq_a | float | [0, 100] | 0.20 | A | `aaPZ0.5b` | `aaPZb` |
| `Y` | current_soft_switch_auto_closed_iq_a | float | [0, 100] 且 >= open | 0.50 | A | `aaPY1.0b` | `aaPYb` |
| `k` | cogging_calib_gain_k | float | >= 0 | 0.05 | - | `aaPk0.10b` | `aaPkb` |
| `X` | read_all 哨兵 | - | 只读 | - | - | 不可用 | `aaPXb` |

\* 齿槽补偿参数与速度 PID 调参共用 `P:U`/`P:V` 子命令字母（分别对应 `pid_speed_ki`、`pid_speed_kd`）。由于解析器仅支持大写字母的约束以及 `A-Z` 空间的限制，`FOC_PROTOCOL_ENABLE_COGGING_COMP` 和 `FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING` 在编译时**互斥**。若两者均设为 `ENABLE`，编译器会发出提示，仅一个代码路径生效（见 `foc_compile_limits.h`）。

### 4.2 控制模式值

- `0`：速度+角度模式
- `1`：纯速度模式

构建裁剪约束：

- 完整构建：允许 `0` 和 `1`
- 纯速度构建：仅接受 `1`
- 速度角度构建：仅接受 `0`

速度参数映射：

- `R`（`angle_position_speed_rad_s`）：速度+角度模式使用的速度参考（非负限幅）
- `S`（`speed_only_speed_rad_s`）：纯速度模式使用的速度参考（支持负方向）

### 4.3 电流环 PID 抗饱和算法

电流环 PID 使用**条件积分（Conditional Integration）**抗饱和策略：

```
如果（输出未饱和 OR 误差方向与饱和方向相反）
    正常积分；
否则
    冻结（回滚本次积分增量）；
```

并附加积分钳位安全网 `|积分| ≤ |输出最大 / ki|` 作为极端情况保护。

与 back-calculation 相比，条件积分允许积分器在瞬态响应中自由建立，更适合电流环小 KP + 大 KI 的参数风格。

### 4.4 电流软切换模式值

- `0`：开环（纯开环电流模型）
- `1`：闭环（纯电流 PID）
- `2`：自动（阈值+滞回，一阶混合）

混合时间常数当前使用编译期宏 `FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_DEFAULT_SEC`（本版本不暴露为运行时参数）。

### 4.5 示波参数掩码位

| 位 | 十六进制 | 字段 | 默认启用 |
|----|---------|------|---------|
| 0 | 0x0001 | current_a | 是 |
| 1 | 0x0002 | current_b | 是 |
| 2 | 0x0004 | current_c | 否 |
| 3 | 0x0008 | angle_filtered | 是 |
| 4 | 0x0010 | angle_accum | 否 |
| 5 | 0x0020 | execution_time_us | 否 |
| 6 | 0x0040 | cogging_iq | 否 |
| 7 | 0x0080 | vbus_voltage | 否 |
| 8 | 0x0100 | iq_target | 是 |
| 9 | 0x0200 | iq_measured | 是 |

默认掩码：0x030B（current_a + current_b + angle_filtered + iq_target + iq_measured）。

## 5. 状态子命令

状态通道使用命令 `S`，状态值为严格数字 `0` 或 `1`。

| 子命令 | 状态名 | 默认值 | 写示例（`S`） | 读示例（`S`） |
|--------|--------|--------|--------------|-------------|
| `M` | motor_enable | 1 | `aaSM1b` | `aaSMb` |
| `S` | semantic_report_enabled | 0 | `aaSS1b` | `aaSSb` |
| `O` | oscilloscope_report_enabled | 0 | `aaSO1b` | `aaSOb` |
| `C` | current_soft_switch_enabled | 1（宏默认，实际值取决于特性/协议开关） | `aaSC1b` | `aaSCb` |
| `G` | cogging_comp_enabled | 0（跟随 `FOC_COGGING_COMP_ENABLE`） | `aaSG1b` | `aaSGb` |
| `X` | read_all 哨兵 | - | 不可用 | `aaSXb` |

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
state.semantic_report_enabled=ENABLE
STATE SYS=1 COMM=1 REPORT=1 DIRTY=1 LAST=1 INIT=1 FAULT=NONE SENS_INV=0 PROTO_ERR=0 PARAM_ERR=0 CTRL_SKIP=0
FAULT_CTRL state=1 fault=NONE proto_err=0 param_err=0 ctrl_skip=0
```

格式化规则：

- 浮点参数：3 位小数
- 整数参数：无符号十进制文本
- 状态输出：`ENABLE` 或 `DISABLE`

故障状态行为：

- 在 FAULT 状态下，调试流定期语义/示波输出被抑制。
- 命令路径诊断和显式查询/清除命令仍可用。

## 7. 完整命令示例

### 7.1 核心命令

```text
aaPA3.14b   # 写入 target_angle_rad
aaPAb       # 读取 target_angle_rad
aaPXb       # 读取所有参数
aaSM1b      # motor_enable = ENABLE
aaSMb       # 读取 motor_enable
aaSXb       # 读取所有状态
aaYRb       # 读取运行时状态摘要
aaYCb       # 清除故障计数器 + 软诊断重初始化
aaYIb       # 运行时电机参数重初始化（方向/极对数/零位标定）
```

### 7.2 常用参数写入

```text
aaPD0b      # control_mode = 速度+角度
aaPD1b      # control_mode = 纯速度
aaPS-20b    # 纯速度速度参考 = -20rad/s
aaPR12b     # 速度+角度模式速度限制/参考 = 12rad/s
aaPW45b     # 传感器采样偏移百分比 = 45
aaPL20b     # 语义报告频率参数示例值 = 20
aaPH100b    # 示波报告频率参数示例值 = 100
aaPO63b     # 示波掩码位 0..5 全开
aaPP1.5b    # 速度 kp
aaPU0.6b    # 速度 ki
aaPV0.005b  # 速度 kd
aaPQ2b      # current_soft_switch_mode = 自动
aaPZ0.5b    # current_soft_switch_auto_open_iq_a = 0.5A
aaPY1.0b    # current_soft_switch_auto_closed_iq_a = 1.0A
aaPk0.10b   # cogging_calib_gain_k = 0.10
```

### 7.3 常用状态写入

```text
aaSM1b      # 电机使能
aaSM0b      # 电机关闭
aaSS1b      # 语义报告使能
aaSO1b      # 示波报告使能
aaSC1b      # 电流软切换使能
aaSG1b      # 齿槽补偿使能（仅当 FOC_PROTOCOL_ENABLE_COGGING_COMP=ENABLE）
aaYGb       # 启动运行时齿槽标定（仅当 FOC_COGGING_CALIB_ENABLE=ENABLE）
aaYDb       # 导出齿槽补偿表到串口（仅当 FOC_COGGING_CALIB_ENABLE=ENABLE）
aaYTb       # 以 C 代码形式导出齿槽补偿表到串口（仅当 FOC_COGGING_CALIB_ENABLE=ENABLE）
```

### 7.4 常用读取

```text
aaPDb       # 读取 control_mode
aaPSb       # 读取 speed_only_speed_rad_s
aaPRb       # 读取 angle_position_speed_rad_s
aaSMb       # 读取 motor_enable
aaPWb       # 读取 sensor_sample_offset_percent
aaPOb       # 读取 oscilloscope_param_mask
aaPPb       # 读取 pid_speed_kp
aaPMb       # 读取 min_mech_angle_accum_delta
aaSCb       # 读取 current_soft_switch_enabled
aaPkb       # 读取 cogging_calib_gain_k
```

## 8. 常见错误与原因

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

## 9. 推荐测试流程

1. 打开绑定到实例输入/输出通道的主机工具。
2. 发送 `aaPXb` 确认完整参数转储出现在输出通道。
3. 发送一个参数写入命令（例如 `aaPP1.5b`），验证：
   - 输出通道收到 `O`
   - 输出通道打印更新的参数行
4. 发送一个状态写入命令（例如 `aaSM1b`）并验证状态输出行。
5. 发送 `aaYRb` 检查运行时状态摘要。
6. 发送 `aaYCb` 确认故障计数器已清除。

实例特定的串口终端设置和通道布线文档见：
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`

最佳实践：

- 一次只发送一帧（不要在单次突发中拼接多帧）。
- 保持命令/子命令大写，帧分隔符小写（`a`、`b`）。
- 将写入值保持在有效范围内。