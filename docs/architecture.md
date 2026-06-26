# 架构与结构总览（唯一结构说明）

## 文档定位

本文件是仓库结构、分层、依赖方向和数据流的唯一事实源（SSOT）。

约束：

1. 不写死具体控制频率，频率以 `foc_core/include/LS_Config/foc_cfg_*.h` 为准。
2. 架构事实必须可映射到真实文件。
3. 代码与文档冲突时，以代码为准并同次修正文档。

## 仓库结构

```text
FOC_VSCODE/
├── foc_core/                         ← 平台无关可复用控制库
│   ├── include/
│   │   ├── LS_Config/           ← 符号定义、功能开关、默认值、编译期约束、类型定义、数据表
│   │   ├── L1_Orchestration/    ← 应用编排（主循环、输出管理器、service handler、monitor queue types）
│   │   ├── L2/
│   │   │   ├── Control/         ← 控制算法（10 模块）
│   │   │   ├── Protocol/        ← 协议帧解析、命令执行、输出适配
│   │   │   └── Runtime/         ← 调度器、环形队列、调试流生成器
│   │   └── L3/                  ← 数学变换、平台抽象API、传感器采样、SVPWM
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

## 分层模型

层级方向严格单向：`LS → L1 → L2 → L3 → L4`

| 层级 | 主要位置 | 职责 | 实例化职责 |
|---|---|---|---|
| `LS` 配置层 | `foc_core/include/LS_Config/` | 符号定义、功能开关、默认值、编译期约束、类型定义、数据表 | 无实例（纯宏与类型） |
| `L1` 编排层 | `foc_core/src/L1_Orchestration/` | 启动流程、实例化核心数据结构（`foc_motor_t`、`foc_system_t`）、主循环编排、实例化和持有所有队列（RX/TX FIFO、monitor_elem_q）、调度器/指示器管理 | **持有所有运行时实例**（系统结构体、队列缓冲区、调度器、调试流状态） |
| `L2/Control` | `foc_ctrl_*.c`（10 模块） | 控制算法：执行器/配置/初始化/外环/电流环/参数学习/补偿/**齿槽标定**/**非阻塞重初始**/执行输出。齿槽标定和重初始化以非阻塞状态机形式由 L1 控制任务路由调用。 | 不持实例，操作传入的 `foc_motor_t` 指针 |
| `L2/Protocol` | `foc_protocol_handler.c`、`foc_protocol_output.c`、`foc_protocol_parser.c` | **单帧处理**：解析一帧 → 修改 motor 字段 → 返回结果结构体。不读帧、不入队、不轮询 | 不持实例，工作所需指针由 L1 传入（遥测策略配置） |
| `L2/Runtime` | `foc_task_scheduler.c`、`foc_queue.c`、`foc_debug_stream.c` | 调度器（任务速率管理）；环形队列（**纯方法模块**，不持实例，调用者传入队列指针）；调试流生成器（提供 PollNextValue + 格式化接口，由 L1 双上下文调用） | 队列类型可实例化，但实例在 L1 分配；调度器/调试流实例由 L1 持有 |
| `L3` 基础服务层 | `foc_core/src/L3_Hal/` | 数学变换、LUT、平台抽象API、传感器采样、SVPWM | 无实例（纯函数或操作 motor 中的字段） |
| `L4` 板级驱动层 | `examples/.../software/Utilities/`、`Firmware/` | 外设驱动与芯片库实现 | 芯片固有实例 |

### 分层约束

1. L2 各块访问硬件只能通过 L3 平台 API（`foc_pal.h`）。
2. 公共头文件不得暴露 `gd32f30x_*` 设备头。
3. L4 不得反向依赖 `foc_core/src/*` 业务逻辑。
4. 配置常量必须收敛在 `foc_cfg_*.h`，禁止在业务 `.c` 中散落默认值。
5. L2 各块间禁止跨块直接调用，数据通过 `foc_motor_t` 由 L1 统一协调。
6. **L2 任何模块不得包含 `L1_Orchestration/` 头文件**。
7. **L2 任何模块不得持有队列实例**——队列存储由 L1 在 `foc_runtime_ctx_t` 中分配，L2 通过指针参数操作。
8. L1 编排负责检测 dirty 标志、转发系统命令、管理初始化流程。
9. **L1 不直接调用 Sensor_* / SVPWM_* 等 L3 硬件初始化方法**——硬件初始化统一通过 L2 的 `FOC_ControlPlatform_InitHardware()` 收口。平台管理类（`FOC_Platform_*`、回调注册）和输出封装（`FOC_OutputMgr_*`）仍由 L1 直调 L3。

## 核心数据结构

系统以两个顶层结构体为数据中枢：

- **`foc_motor_t`**（定义于 `foc_ctrl_types.h`）— 电机控制数据结构，包含控制参数、状态、PID、运行时状态、各外环状态等。L1 实例化，L2 各块通过指针读/写。
- **`foc_system_t`**（定义于 `foc_system_types.h`）— 系统级数据结构，包含：
  - `cfg`：系统配置（遥测策略、报告模式，不随 reinit 重置）
  - `runtime`：运行时状态（调度器、调试流、RX/TX 队列缓冲区、monitor_elem_q 队列、指示器状态）

## 数据流设计

### 总体架构

```
┌───────────────── ISR 上下文 ─────────────────┐
│  调度器回调触发 Service/Monitor/Control 任务    │
│  Service ISR: 读平台帧 → 入 RX 队列            │
│  Monitor ISR: 快照 sensor/motor 关键字段 →      │
│               DebugStream_PollNextValue 逐元素   │
│               入 monitor_elem_q                  │
│  Control ISR: 控制循环（传感器→算法→输出）      │
└──────────────────────────────────────────────┘
                       │
                       ▼
┌────────────── 主循环上下文 ───────────────────┐
│  FOC_App_Loop():                              │
│    1. Monitor 段: monitor_elem_q 出队 →       │
│       switch(tag): FormatSemanticLine/AppendOsc │
│       → FIFO_Enqueue(tx_fifo)                  │
│    2. Service 段: RX 队列 → 协议处理 → 编排    │
│       (needs_summary → monitor_elem_q 统一输出) │
│    3. TX 队列出队 → 平台发送                    │
└──────────────────────────────────────────────┘
```

### 协议数据流（输入 → 处理 → 输出）

```
平台 UART ISR
  │
  ▼ （Service 触发回调中）
读帧（FOC_Platform_CommSource*_ReadFrame）
  │
  ▼
FIFO_Enqueue(rx_fifo)       ← L2/Runtime 队列方法，操作 L1 的队列实例
  │
  ▼ （主循环 Service 段）
FIFO_Dequeue(rx_fifo)        ← L2/Runtime 队列方法
  │
  ▼
FOC_Protocol_ProcessSingle() ← L2/Protocol 单帧处理
  │   解析帧 → 修改 motor 字段 → 返回结果结构体
  │
  ├── [comm_active]  → 更新 LED 指示器
  ├── [needs_summary] → L1 生成 MONITOR_ELEM_PROTOCOL_SUMMARY →
  │                      FIFO_Enqueue(monitor_elem_q)（由 Monitor 段统一格式化输出）
  ├── [needs_status]  → 状态码已在协议内部直写（快路径）
  └── [param_changed] → L1 稍后检测 cfg_dirty → ApplyConfig
```

### 双输出路径

| 路径 | 机制 | 用途 | 执行位置 |
|------|------|------|---------|
| **快路径**（直写） | 直接调用 L3 平台 API（`FOC_Platform_Write*`） | 状态码、参数行、错误回报等短数据 | ISR 或协议处理函数内部 |
| **慢路径**（队列） | ISR: FIFO_Enqueue(monitor_elem_q) → 主循环: FIFO_Dequeue → 格式化(tag switch) → FIFO_Enqueue(tx_fifo) → 平台发送 | 语义遥测、示波器帧、协议摘要等多行数据 | 入队在 MonitorTrigger ISR（快照 + PollNextValue），格式化+入 TX 在主循环 Monitor 段，出 TX 由 L1 统一消费 |

**快路径的特点**：短小、可打断队列输出、不在乎阻塞（因为很短）。
**慢路径的特点**：大数据量、需要缓冲、通过队列解耦生产者与消费者。

### Monitor 元素队列机制

`monitor_elem_q` 是 L1 新增的轻量标记元素队列，统一所有慢输出路径：

```
模板：fifo_queue_t，元素 = monitor_element_t {tag, aux, value}
深度：FOC_MONITOR_ELEM_QUEUE_DEPTH（默认 32）

MonitorTrigger ISR:
  1. 快照 motor->sensor 关键字段到栈（~14 个赋值）
  2. Push MONITOR_ELEM_FRAME_START（帧隔离标记）
  3. while (DebugStream_PollNextValue → 元素) { Push 元素 }
  4. Set monitor_task_pending

主循环 Monitor 段:
  while (consumed < FOC_MONITOR_MAX_DEQUEUE_PER_CYCLE):
    Pop → switch(tag):
      FRAME_START → 丢弃上一帧残余，开始新帧
      SEMANTIC_0~7 → FormatSemanticLine → TX FIFO
      SEMANTIC_END → 帧结束
      OSC_VALUE → AppendOscValue 累积
      OSC_END → FormatOscLine（加头尾） → TX FIFO
      PROTOCOL_SUMMARY → 格式化摘要 → TX FIFO
```

帧隔离标记（`FRAME_START`）保证：即使主循环阻塞后恢复，队列中的帧也不会交错。

### 数据流核心规则

1. **L2 层只调方法，不持实例**。队列方法定义在 L2/Runtime，但实例在 L1 的 `foc_runtime_ctx_t` 中。
2. **L2 层不碰队列操作**。协议处理只返回结果结构体，调试流提供 ISR 安全的 `PollNextValue` 接口和主循环格式化函数，入队/出队由 L1 编排。
3. **L1 是唯一编排者**。ISR 读帧→入队、ISR 快照→入 monitor_elem_q、主循环出队→处理→入 TX 队列、TX 出队→发送，全由 L1 控制。
4. **DebugStream 双接口**：`PollNextValue`（ISR 上下文调用，跑 state machine 取值）和 `Format*` 函数（主循环上下文调用，格式化字符串），两者分离确保采样时机正确。

## 控制算法链

### 控制模块结构

L2/Control 按 `foc_ctrl_XX_name.c` 命名，模块划分：

| 编号 | 模块 | 职责 |
|------|------|------|
| C11 | `foc_ctrl_executor` | 算法入口：外环/内环/开环/补偿入口，ISR 路径与外环调度 |
| C12 | `foc_ctrl_init` | 初始化与标定 |
| C13 | `foc_ctrl_cfg` | 配置状态管理（软切换、齿槽补偿、PID 初始化、fine-tuning setter） |
| C21 | `foc_ctrl_outer_loop` | 速度/位置外环 |
| C22 | `foc_ctrl_current_loop` | 电流内环 |
| C23 | `foc_ctrl_param_learn` | 电机参数学习 |
| C24 | `foc_ctrl_compensation` | 齿槽补偿 |
| C25 | `foc_ctrl_cogging_calib` | 齿槽标定（非阻塞状态机，由 L1 通过 control_phase 路由调用） |
| C26 | `foc_ctrl_reinit` | 非阻塞重初始化（由 L1 通过 control_phase 路由调用） |
| C31 | `foc_ctrl_actuation` | 执行输出（SVPWM 驱动） |

### 控制运行链

```
初始化链：FOC_MotorInit → FOC_ControlConfigResetDefault
       → FOC_ControlExecutor_Init → FOC_Control_ApplyConfig

控制循环（ControlTrigger ISR）：
  L1 编排 —— Sensor_ReadEncoder（按 FOC_SENSOR_ANGLE_FAST_ENABLE 条件）
           → Sensor_ReadVBUS
           → Sensor_SyncCurrentSnapshot（将 ISR 电流值同步到 motor->sensor）
           → 安全检查 → FOC_ControlExecutor_RunCycle()
             → 故障检查 → 外环控制（速度/角度 PID）

PWM ISR（高速路径，电流采样唯一入口）：
  FOC_ControlExecutor_RunISR()
    → Sensor_ReadCurrent（ADC 读取 → motor->sensor_fast，独占总线）
    → Sensor_ReadEncoder（仅 FOC_SENSOR_ANGLE_FAST_ENABLE 使能时）
    → Sensor_AccumulateEcycle（参考角度按宏条件使用 fast snapshot 或 volatile 桥接）
    → FOC_CurrentControlStep → Clarke/Park → PID → motor->iq_measured
    → SVPWM 执行

配置应用：
  FOC_Control_ApplyConfig(motor)
    → 从 motor 结构体读取 PID 参数和 fine-tuning 设置
```

### 采样路径规则

1. **电流采样独占性**：ADC 电流读取全部归 PWM ISR 独占（`Sensor_ReadCurrent`），控制 ISR 不再直接读 ADC。
2. **编码器角度路径选择**：由 `FOC_SENSOR_ANGLE_FAST_ENABLE` 宏联动：
   - `DISABLE`（慢速编码器，如 I2C AS5600）：在 Control ISR 中读取（`Sensor_ReadEncoder`），通过 `motor->ecycle_ref_angle_rad` volatile 桥接供 PWM ISR 中的 e-cycle 累积使用。
   - `ENABLE`（快速编码器，如霍尔/QEI）：在 PWM ISR 中同步读取。
3. **控制 ISR 电流数据来源**：通过 `Sensor_SyncCurrentSnapshot` 从 `motor->sensor_fast` 复制到 `motor->sensor`，直接复制不作滤波处理。
4. **e-cycle 漂移抑制**：角度参考源与编码器路径一致——快速路径使用 fast snapshot 角度，慢速路径使用 volatile 桥接字段。
5. **L3 平台 API**：统一为单一 `FOC_Platform_ReadPhaseCurrent`，无 `Fast/Slow` 双入口。

## 调度模型

调度器位于 `L2/Runtime/foc_task_scheduler`，管理三种任务速率：

- **服务任务（中速）**：ISR 中读帧入 RX 队列 + 更新指示器，主循环中出队解析、参数同步
- **控制主循环（快速）**：传感器读取、外环控制
- **监测任务（低速）**：ISR 中快照 sensor 数据 → 入 monitor_elem_q，主循环中出队格式化输出

控制节拍源与 PWM 更新中断源分离：
- 控制节拍源驱动调度器回调
- PWM 更新中断源驱动高速电流环路径
- 采样触发与 PWM 对齐

## 宏裁剪口径

### 算法特性开关（LS）

定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`

1. 电流环与软切换特性
2. 齿槽补偿特性（补偿使能 + 运行时手动标定使能）
3. 采样滤波特性（Kalman、LPF、电气周期偏移补偿）

### 协议裁剪开关

1. 定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`
2. 固定最小集（不可裁剪）：`P:A/R/S/D`、`S:M`、`Y:R/C`
3. 可选组：`FOC_PROTOCOL_ENABLE_*`
4. **协议裁剪宏仅控制协议命令可见性与参数读写通道，不得用于保护控制算法的逻辑分支**

### 编译期约束

定义位置：`foc_core/include/LS_Config/foc_compile_limits.h`

1. 开关合法性与范围约束：`#error` 阻断非法配置
2. 跨开关冲突提示：编译警告
3. 典型硬约束：调度分频整除、PWM/ISR 频率整除、初始化标定关闭时默认方向/极对必须定义

## 维护规则

1. 结构/依赖变化必须同步更新本文件。
2. 更新本文件后，同次检查：`docs/README.md`、`docs/development.md`。
3. 禁止新增并行结构文档作为"兼容跳转页"。