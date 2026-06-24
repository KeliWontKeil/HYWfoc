# 架构与结构总览（唯一结构说明）

## 文档定位

本文件是仓库结构、分层、依赖方向和控制链路的唯一事实源（SSOT）。

约束：

1. 不写死具体控制频率，频率以 `foc/include/LS_Config/foc_cfg_*.h` 为准。
2. 架构事实必须可映射到真实文件。
3. 代码与文档冲突时，以代码为准并同次修正文档。

## 仓库结构（当前 v1.8.1）

```text
FOC_VSCODE/
├── foc/
│   ├── include/
│   │   ├── LS_Config/       ← 符号定义、功能开关、类型定义、数据表
│   │   ├── L1_Orchestration/← 应用编排（精简）
│   │   ├── L2/
│   │   │   ├── Control/     ← 控制算法（8 模块）
│   │   │   ├── Protocol/    ← 协议帧接入、解析、命令执行、输出适配
│   │   │   └── Runtime/     ← 调度器、调试流（独立工具）
│   │   └── L3/              ← 基础服务：Math + PAL + sensor + SVPWM
│   └── src/
│       ├── L1_Orchestration/
│       └── L2/
│           ├── Control/
│           ├── Protocol/
│           └── Runtime/
├── examples/GD32F303_FOCExplore/
│   ├── hardware/
│   └── software/
├── docs/
└── .github/
```

## 分层模型（代码与职责）

| 层级 | 主要位置 | 职责 |
|---|---|---|
| `LS` 配置层 | `foc/include/LS_Config/` | 符号定义、功能开关、默认值、编译期约束、类型定义、数据表 |
| `L1` 运行编排层 | `foc/src/L1_Orchestration/foc_app.c` + `foc_service_handler.c` | 启动流程、实例化 `foc_motor_t`、服务任务编排（协议/配置同步/系统命令/重初始化）、指示器 |
| `L2/Control` | `foc_ctrl_*.c` 共 8 模块 | 控制算法（执行器/配置/初始化/外环/电流环/参数学习/补偿/执行输出） |
| `L2/Protocol` | `foc_protocol_handler.c`、`foc_protocol_output.c`、`foc_protocol_parser.c` | 协议帧接入（多源帧读取）、帧解析、命令执行（参数/状态读写）、输出适配 |
| `L2/Runtime` | `foc_task_scheduler.c`、`foc_debug_stream.c` | 调度器、调试流（独立工具，无运行时管线） |
| `L3` 基础服务层 | `foc/include/L3/`、`foc/src/L3/` | 数学变换、LUT、平台抽象API、**传感器采样（sensor）、SVPWM调制（svpwm）** |
| `L5` 板级驱动层 | `examples/.../software/Utilities/*`、`Firmware/*` | 外设驱动与芯片库实现 |

**关键变化（v1.8.1 阶段性调整）：**
- 删除 `foc_ctrl_entry.c/h`（入口点文件）和 `foc_ctrl_fast.c/h`
- 新增 `foc_ctrl_executor.c/h` — 控制执行器，合并 ISR 路径与外环调度
- `foc_svpwm.c/h`、`foc_sensor.c/h` 从 L2/Control 移入 L3 基础服务层
- 静态全局变量（速度外环状态、SVPWM 前置 LPF 状态、控制模式追踪等）移入 `foc_motor_t` 结构体
- `FOC_Control_Init` 函数删除，职责整合入 `FOC_ControlConfigResetDefault`
- `FOC_Control_ApplyConfig` 从入口文件移入 `foc_ctrl_cfg.c`

## 数据流设计

L1 (foc_app.c) 实例化一个核心数据结构：

```text
foc_motor_t g_motor;           ← 电机控制数据结构（包含控制参数/状态/PID/运行时状态
                                 /per-motor 外环状态）

数据流：
  协议块：FOC_Protocol_Process(&g_motor)  → 读帧、修改 g_motor.state / g_motor.cfg
  控制块：FOC_ControlExecutor_RunOuterLoop(&g_motor, &sensor, dt)
                                  → 读 g_motor.cfg 做算法，写 g_motor
  L1 编排：
    检测 g_motor.state.cfg_dirty  → 调用 FOC_Control_ApplyConfig(&g_motor)
    检测 g_motor.state.reinit_pending  → 调用 FOC_Service_ReInitMotor()
    检测 g_motor.state.pending_system_action → 转发到控制块
```

块间严禁直接函数调用，所有数据传递通过 `foc_motor_t` 结构体由 L1 统一协调。

## 当前主链路

### 协议处理链（Service Task 中执行）

协议处理：`FOC_Protocol_Process(motor, budget)`
1. 轮询多通讯源读取帧（FrameSource_TryReadReady）
2. 帧提取 + 协议解析（ProtocolCore_ExtractFrame + ProtocolCore_ParseFrame）
3. 命令派发：P通道（参数读写）/ S通道（状态读写）/ Y通道（系统命令）
4. 所有命令直接读写 `motor->cfg` 和 `motor->state`，设置 `cfg_dirty` 标志

### 控制运行链

1. **初始化链**：`FOC_MotorInit` → `FOC_ControlConfigResetDefault` → `FOC_ControlExecutor_Init` → `FOC_Control_ApplyConfig`
2. **运行外环**：`FOC_ControlExecutor_RunOuterLoop(motor, sensor, dt)` — 内部调度速度/角度外环 + 齿槽补偿
3. **运行内环（PWM ISR）**：`FOC_ControlExecutor_RunISR(motor)` — 采样 → e-cycle 漂移抑制 → 电流环 → SVPWM 执行
4. **控制循环**：`FOC_ControlExecutor_RunCycle(motor, dt)` — 传感器读取 → 故障检查 → 外环控制
5. **配置应用**：`FOC_Control_ApplyConfig(motor)` — 从 motor->cfg 读取PID参数和fine-tuning设置

### 调度

调度器位于 `L2/Runtime/foc_task_scheduler`，管理任务速率：
- 服务任务（中速）：指示灯、协议帧处理、参数同步
- 控制主循环（快速）：传感器读取、外环控制
- 监测任务（低速）：调试流报告

## 宏裁剪口径（与代码一致）

### 算法特性开关（LS）

定义位置：`foc/include/LS_Config/foc_cfg_feature_switches.h`

1. 电流环与软切换特性：`FOC_CURRENT_LOOP_PID_ENABLE`、`FOC_CURRENT_SOFT_SWITCH_ENABLE`
2. 齿槽补偿特性：`FOC_COGGING_COMP_ENABLE`（补偿使能）、`FOC_COGGING_CALIB_ENABLE`（运行时手动标定使能）
3. 采样滤波特性：`FOC_SENSOR_KALMAN_*`、`FOC_SENSOR_ANGLE_LPF_ENABLE`、`FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE`

### 协议裁剪开关

1. 定义位置：`foc/include/LS_Config/foc_cfg_feature_switches.h`
2. 固定最小集（不可裁剪）：`P:A/R/S/D`、`S:M`、`Y:R/C`
3. 可选组：`FOC_PROTOCOL_ENABLE_*`
4. `FOC_PROTOCOL_ENABLE_*` 系列宏仅控制协议命令的可见性与参数读写通道，不得用于保护控制算法中的逻辑分支。

### 编译期约束与提示策略

定义位置：`foc/include/LS_Config/foc_compile_limits.h`

1. 开关合法性与范围约束：使用 `#error` 阻断非法配置。
2. 跨开关冲突提示：使用编译提示（ARMCC5 下通过 `#warning` 分支）。
3. 典型硬约束：调度分频整除、PWM/ISR 频率整除、初始化标定关闭时默认方向/极对必须定义。

## 依赖方向约束（强制）

1. `L2` 各块访问硬件只能通过 `L3` 平台 API（`foc_pal.h`）。
2. 公共头文件不得暴露 `gd32f30x_*`。
3. `L5` 不得反向依赖 `foc/src/*` 业务逻辑。
4. 配置常量必须收敛在 `foc_cfg_*.h`，禁止在业务 `.c` 中散落默认值。
5. L2 各块间禁止跨块直接调用，数据通过 `foc_motor_t` 结构体由 L1 统一协调。
6. Protocol 块只修改 `motor` 结构体字段，不调用任何控制算法函数。
7. Control 块只读取 `motor` 结构体做控制算法，不调用任何协议/IO 函数。
8. L1 编排负责检测 dirty 标志、转发系统命令、管理初始化流程。

## 控制时序（抽象）

```text
控制节拍源
├── 调度器回调（服务任务、监测任务、控制主循环）
└── 控制主循环入口

PWM 更新中断源（高速路径）
├── SVPWM 插值
└── （可选）电流环快路径

采样触发源
└── 与 PWM 对齐的电流/角度采样
```

具体定时器映射与引脚归属由实例文档维护。

## 维护规则

1. 结构/依赖变化必须同步更新本文件。
2. 更新本文件后，同次检查：`docs/README.md`、`docs/development.md`。
3. 禁止新增并行结构文档作为"兼容跳转页"。