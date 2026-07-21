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
│   │   │   ├── Control/         ← 控制算法（20 模块：含估计器/桥接/启动/过渡/有感标定）
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
| `L2/Control` | `foc_ctrl_*.c`（20 模块） | 控制算法：执行器/配置/初始化/外环/电流环/参数学习/补偿/估计器/桥接/启动策略/过渡管理/有感齿槽标定/有感重初始/执行输出 | 不持实例，操作传入的 `foc_motor_t` 指针 |
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
10. **传感器硬件存在性与算法使用分离**：`FOC_SENSOR_ENCODER_ENABLE` 控制编码器硬件层，`FOC_ESTIMATOR_ENCODER_ENABLE` 控制是否使用编码器作为反馈源。齿槽补偿/标定依赖 `FOC_SENSOR_ENCODER_ENABLE`。
11. **有效性检查收口单一检查点**：传感器有效性检查仅在 L1 `FOC_App_ControlTrigger` 中执行，L2 `RunCycle` 不再重复检查。

## 核心数据结构

系统以两个顶层结构体为数据中枢：

- **`foc_motor_t`**（定义于 `foc_ctrl_types.h`）— 电机控制数据结构，包含控制参数、状态、PID、估计器状态（`est_state`）、控制输入快照（`ctrl_input`）、各估计器私有状态、启动/过渡状态、各外环状态等。L1 实例化，L2 各块通过指针读/写。
- **`foc_system_t`**（定义于 `foc_system_types.h`）— 系统级数据结构，包含：
  - `cfg`：系统配置（遥测策略、报告模式，不随 reinit 重置）
  - `runtime`：运行时状态（调度器、调试流、RX/TX 队列缓冲区、monitor_elem_q 队列、指示器状态）

### 估计器数据流（采样与控制解耦）

```
┌─────────────────── 数据源层 ───────────────────┐
│  Sensor_ReadCurrent → sensor_fast (电流)        │
│  Sensor_ReadEncoder → sensor.mech_angle_rad     │
│  Sensor_ReadVBUS    → sensor.vbus_*             │
│                                                 │
│  foc_ctrl_estim_* → 读原始角度/电流             │
│     → 私有状态 → est_state.mech_angle_rad       │
└───────────────────┬─────────────────────────────┘
                    │
                    ▼
┌─────────────────── 桥接层 ──────────────────────┐
│  FOC_Bridge_CopyInput(motor)                    │
│    est_state → ctrl_input (瞬时角度 + 电流)     │
└───────────────────┬─────────────────────────────┘
                    │
                    ▼  (Control ISR)
┌─────────────────── 控制算法层 ──────────────────┐
│  外环: ctrl_input.mech_angle_rad                │
│        → 累积角度维护 → electrical_phase_angle  │
│  齿槽补偿: ctrl_input.mech_angle_rad            │
│  下一周期 PWM ISR: 电流环 + SVPWM               │
└─────────────────────────────────────────────────┘
```

**关键约束**：
- 控制算法只读 `ctrl_input`，不直接接触任何估计器的内部状态
- 估计器只写 `est_state`，桥接层统一拷贝到 `ctrl_input`
- `Sensor_ReadEncoder` 作为 L3 硬件抽象层函数保留不变

### 估计器体系

所有反馈源（编码器、SMO、HFI）通过统一接口写入 `motor->est_state`：

| 估计器 | 类型宏 | 需要启动策略 | 运行节拍 |
|--------|--------|------------|---------|
| ENCODER | `FOC_ESTIMATOR_TYPE_ENCODER` | 否 | PWM ISR (FAST) 或 Control ISR (SLOW) |
| SMO | `FOC_ESTIMATOR_TYPE_SMO` | 是（强拖或 HFI） | PWM ISR |
| HFI | `FOC_ESTIMATOR_TYPE_HFI` | 否 | PWM ISR |
| FLUX | `FOC_ESTIMATOR_TYPE_FLUX` | 预留 | — |

双估计器过渡机制：过渡期同时运行主/副估计器（`estimator_step_fn` / `estimator_step_fn_alt`），
桥接层按 `blend_factor` 加权混合两路角度。

## 数据流设计

### 总体架构

```
┌───────────────── ISR 上下文 ─────────────────┐
│  调度器回调触发 Service/Monitor/Control 任务    │
│  Service ISR: 读平台帧 → 入 RX 队列            │
│  Monitor ISR: 快照 sensor/motor 关键字段 →      │
│               DebugStream_PollNextValue 逐元素   │
│               入 monitor_elem_q                  │
│  Control ISR: 4 阶段（传感器→估计器→桥接→控制） │
│  PWM ISR: 4 阶段（插值采样→电流环→角度同步+     │
│           估计器→SVPWM）                         │
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
5. **DebugStream 数据源**：角度从 `ctrl_input.mech_angle_rad` 和 `motor->mech_angle_accum_rad` 读取，反映控制算法实际看到的状态，而非独立采集原始传感器数据。

## 控制算法链

### 控制模块结构

L2/Control 按 `foc_ctrl_XX_name.c` 命名，模块划分：

| 编号 | 模块 | 职责 |
|------|------|------|
| C11 | `foc_ctrl_executor` | 算法入口：外环/内环/开环/补偿入口，ISR 路径与外环调度 |
| C12 | `foc_ctrl_init` | 初始化与标定 |
| C13 | `foc_ctrl_cfg` | 配置状态管理（软切换、齿槽补偿、PID 初始化、fine-tuning setter） |
| C14 | `foc_ctrl_bridge` | 桥接层：est_state → ctrl_input 拷贝 |
| C15 | `foc_ctrl_estim` | 估计器选择/注册中心 |
| C16 | `foc_ctrl_estim_encoder` | 编码器估计器（从 sensor 读原始角度，写入 est_state） |
| C17 | `foc_ctrl_estim_smo` | SMO 估计器 |
| C18 | `foc_ctrl_estim_hfi` | HFI 估计器 |
| C19 | `foc_ctrl_estim_flux` | FLUX 估计器（预留） |
| C21 | `foc_ctrl_outer_loop` | 速度/位置外环 |
| C22 | `foc_ctrl_current_loop` | 电流内环 |
| C23 | `foc_ctrl_param_learn` | 电机参数学习 |
| C24 | `foc_ctrl_compensation` | 齿槽补偿 |
| C25 | `foc_ctrl_sens_cogging_calib` | 有感齿槽标定（非阻塞状态机，由 L1 通过 control_phase 路由调用） |
| C26 | `foc_ctrl_sens_reinit` | 有感非阻塞重初始化（由 L1 通过 control_phase 路由调用） |
| C27 | `foc_ctrl_startup` | 启动策略管理 |
| C28 | `foc_ctrl_startup_openloop` | 强拖启动 |
| C29 | `foc_ctrl_transition` | 估计器过渡管理（加权混合） |
| C31 | `foc_ctrl_actuation` | 执行输出（SVPWM 驱动） |

### 控制运行链

```
初始化链：FOC_MotorInit → FOC_ControlConfigResetDefault
       → FOC_EstimEncoder_Init → FOC_Estimator_Select
       → FOC_ControlExecutor_Init → FOC_Control_ApplyConfig

Control ISR（4 阶段，严格串行，不可调换）：
  阶段1：传感器读取
    → Sensor_ReadEncoder / Sensor_ReadVBUS
    → Sensor_SyncCurrentSnapshot（将 ISR 电流同步到 motor->sensor）
    → 有效性检查（adc_valid + [encoder] encoder_valid）
  阶段2：估计器更新
    → motor->estimator_step_fn(motor, &motor->est_state, dt)
  阶段3：桥接拷贝
    → FOC_Bridge_CopyInput(motor)：est_state → ctrl_input
  阶段4：控制执行
    → 按 control_phase 路由：STARTUP/NORMAL/COGGING_CALIB/REINIT
    → NORMAL: FOC_ControlExecutor_RunCycle → RunOuterLoop → 外环控制

PWM ISR（4 阶段，严格串行，不可调换）：
  阶段1：插值与采样
    → SVPWM_InterpolationISR → Sensor_ReadCurrent
    → [FAST] Sensor_ReadEncoder → Sensor_AccumulateEcycle
  阶段2：电流环
    → FOC_CurrentControlStep → Clarke/Park → PID → ud/uq
  阶段3：角度同步 + 估计器
    → [FAST] sensor_fast → sensor 角度同步（仅拷贝 raw_value）
    → motor->estimator_step_fn(motor, &motor->est_state, dt)
  阶段4：SVPWM 输出
    → FOC_ControlApplyElectricalAngleRuntime

配置应用：
  FOC_Control_ApplyConfig(motor)
    → 从 motor 结构体读取 PID 参数和 fine-tuning 设置
```

### 采样路径规则

1. **电流采样独占性**：ADC 电流读取全部归 PWM ISR 独占（`Sensor_ReadCurrent`），控制 ISR 不再直接读 ADC。
2. **编码器角度路径选择**：由 `FOC_SENSOR_ANGLE_FAST_ENABLE` 宏联动：
   - `DISABLE`（慢速编码器，如 I2C AS5600）：在 Control ISR 中读取（`Sensor_ReadEncoder`）。
   - `ENABLE`（快速编码器，如霍尔/QEI）：在 PWM ISR 中同步读取。
3. **控制 ISR 电流数据来源**：通过 `Sensor_SyncCurrentSnapshot` 从 `motor->sensor_fast` 复制到 `motor->sensor`。
4. **PWM ISR 角度同步**：电流环之后仅拷贝 `raw_value` 和有效性标志到 `motor->sensor`，不完整拷贝 `kalman_filter_t`。
5. **L3 平台 API**：统一为单一 `FOC_Platform_ReadPhaseCurrent`，无 `Fast/Slow` 双入口。

## 控制阶段枚举

```c
typedef enum {
    FOC_CONTROL_PHASE_NORMAL        = 0U,  // 正常控制
    FOC_CONTROL_PHASE_COGGING_CALIB = 1U,  // 有感齿槽标定
    FOC_CONTROL_PHASE_REINIT        = 2U,  // 有感重初始化
    FOC_CONTROL_PHASE_STARTUP       = 3U   // 无感启动策略
} foc_control_phase_t;
```

## 调度模型

调度器位于 `L2/Runtime/foc_task_scheduler`，管理三种任务速率：

- **服务任务（中速）**：ISR 中读帧入 RX 队列 + 更新指示器，主循环中出队解析、参数同步
- **控制主循环（快速）**：Control ISR 4 阶段（传感器→估计器→桥接→控制）
- **监测任务（低速）**：ISR 中快照 sensor 数据 → 入 monitor_elem_q，主循环中出队格式化输出

控制节拍源与 PWM 更新中断源分离：
- 控制节拍源驱动调度器回调
- PWM 更新中断源驱动高速电流环 + 估计器路径
- 采样触发与 PWM 对齐

## 宏裁剪口径

### 算法特性开关（LS）

定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`

1. 传感器硬件：`FOC_SENSOR_ENCODER_ENABLE`
2. 估计器：`FOC_ESTIMATOR_ENCODER_ENABLE`、`FOC_ESTIMATOR_SMO_ENABLE`、`FOC_ESTIMATOR_HFI_ENABLE`、`FOC_ESTIMATOR_FLUX_ENABLE`
3. 启动策略：`FOC_STARTUP_OPENLOOP_ENABLE`
4. 过渡管理：`FOC_TRANSITION_ENABLE`
5. 齿槽补偿特性（`FOC_COGGING_COMP_ENABLE` + `FOC_COGGING_CALIB_ENABLE`）
6. 采样滤波特性（Kalman、LPF、电气周期偏移补偿）

### 常见功能宏组合

| 场景 | 配置 |
|------|------|
| 有感 FOC | `SENSOR_ENCODER=ENABLE`, `ESTIMATOR_ENCODER=ENABLE`, `SMO/HFI=DISABLE`, `COGGING=ENABLE` |
| 强拖→SMO 无感 | `SENSOR_ENCODER=DISABLE`, `ESTIMATOR_ENCODER=DISABLE`, `SMO=ENABLE`, `STARTUP_OPENLOOP=ENABLE` |
| HFI→SMO 无感 | `SENSOR_ENCODER=DISABLE`, `SMO=ENABLE`, `HFI=ENABLE`, `TRANSITION=ENABLE` |

### 协议裁剪开关

1. 定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`
2. 固定最小集（不可裁剪）：`P:A/R/S/D`、`S:M`、`Y:R/C`
3. 可选组：`FOC_PROTOCOL_ENABLE_*`
4. **协议裁剪宏仅控制协议命令可见性与参数读写通道，不得用于保护控制算法的逻辑分支**

### 编译期约束

定义位置：`foc_core/include/LS_Config/foc_compile_limits.h`

1. 开关合法性与范围约束：`#error` 阻断非法配置
2. 跨开关冲突提示：编译警告
3. 典型硬约束：
   - 估计器依赖编码器硬件（`ESTIMATOR_ENCODER` → `SENSOR_ENCODER`）
   - 齿槽补偿/标定依赖编码器硬件
   - 无感估计器需电流采样
   - SMO 需至少一种启动策略（当编码器不可用时）
   - 过渡管理需至少两个估计器同时使能

## 滤波器子系统

滤波器子系统采用**编译时类型推导 + 按作用对象分组**的配置方式，支持每位置独立选择算法（Kalman/LPF1/None/Biquad）并通过类型推导宏自动绑定正确的结构体类型和门控函数。

### 文件角色

| 文件 | 层级 | 职责 |
|------|------|------|
| `LS_Config/foc_cfg_filter.h` | LS | 滤波器参数配置：按作用对象分组，每组内按算法子分类（Kalman/LPF1 参数） |
| `LS_Config/foc_symbol_defs.h` | LS | 类型值符号（`FOC_FILTER_TYPE_NONE/KALMAN/LPF1/BIQUAD`）和类型推导宏表（`FOC_FILTER_TYPEDEF_0/1/2/3`） |
| `L3_Hal/foc_filter_types.h` | L3 | 滤波器数据结构体（`foc_filter_kalman_t`、`foc_filter_lpf1_t`、`foc_filter_biquad_t`） |
| `L3_Hal/foc_filter_math.h/.c` | L3 | 纯数学滤波算法（无状态、可复用），如 `FOC_FilterMath_KalmanStep`、`FOC_FilterMath_Lpf1Step`、`FOC_FilterMath_Lpf1AngleStep`（带 0/2pi 环绕处理的 LPF1 变体） |
| `L3_Hal/foc_filter_gate.h` | L3 | 门控函数（static inline），在编译时通过 `#if` 选择正确的纯数学函数并传入默认 alpha |

### 参数组织方式（按作用对象分组）

`foc_cfg_filter.h` 中的参数按 **7 个滤波器位置**分组，每组名称前缀格式为 `FOC_FILTER_<POSITION>_<ALGORITHM>_<PARAM>`：

| 位置分组 | 前缀 | 包含参数 | 算法 |
|----------|------|----------|------|
| SENSOR_CURRENT_A | `FOC_FILTER_SENSOR_CURRENT_A_KALMAN_*` / `LPF_ALPHA` | MEAS_ERR, EST_ERR, PROC_NOISE, INIT, LPF_ALPHA | Kalman + LPF1 |
| SENSOR_CURRENT_B | `FOC_FILTER_SENSOR_CURRENT_B_KALMAN_*` / `LPF_ALPHA` | MEAS_ERR, EST_ERR, PROC_NOISE, INIT, LPF_ALPHA | Kalman + LPF1 |
| SENSOR_CURRENT_C | `FOC_FILTER_SENSOR_CURRENT_C_KALMAN_*` / `LPF_ALPHA` | MEAS_ERR, EST_ERR, PROC_NOISE, INIT, LPF_ALPHA | Kalman + LPF1 |
| SENSOR_ANGLE | `FOC_FILTER_SENSOR_ANGLE_KALMAN_*` / `LPF_ALPHA` | MEAS_ERR, EST_ERR, PROC_NOISE, INIT, LPF_ALPHA | Kalman + LPF1 |
| CURRENT_LOOP_IQ | `FOC_FILTER_CURRENT_LOOP_IQ_LPF_ALPHA` | alpha 系数 | LPF1 |
| SVPWM | `FOC_FILTER_SVPWM_LPF_ALPHA` | alpha 系数 | LPF1 |
| ENCODER_SPEED | `FOC_FILTER_ENCODER_SPEED_LPF_ALPHA` | alpha 系数 | LPF1 |

类型选择宏 `FOC_FILTER_SENSOR_CURRENT_A` 等定义于 `foc_cfg_feature_switches.h`，供用户选择每位置的滤波器类型。

### 类型推导宏表

`foc_symbol_defs.h` 中定义了 4 元推导表，将类型值（0/1/2/3）隐式映射为对应的 C 类型：

```c
#define FOC_FILTER_TYPEDEF_0    uint8_t              // NONE
#define FOC_FILTER_TYPEDEF_1    foc_filter_kalman_t   // KALMAN
#define FOC_FILTER_TYPEDEF_2    foc_filter_lpf1_t     // LPF1
#define FOC_FILTER_TYPEDEF_3    foc_filter_biquad_t   // BIQUAD
```

在 `foc_ctrl_types.h` 中，`sensor_data_t` 的每个滤波器字段通过 `FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_A)` 推导出正确的类型。

### 门控函数

`foc_filter_gate.h` 为 7 个位置各提供一个 `FOC_FilterGate_<Position>()` static inline 函数。这些函数在编译时根据位置选择宏的值展开对应的算法分支，消除运行时 dispatch 开销。LPF1 分支直接使用位置特定的 `FOC_FILTER_<POSITION>_LPF_ALPHA` 宏。其中 Angle 门的 LPF1 分支使用 `FOC_FilterMath_Lpf1AngleStep`（处理 0/2PI 环绕），其余位置使用标准 `FOC_FilterMath_Lpf1Step`。

## 维护规则

1. 结构/依赖变化必须同步更新本文件。
2. 更新本文件后，同次检查：`docs/README.md`、`docs/development.md`。
3. 禁止新增并行结构文档作为"兼容跳转页"。