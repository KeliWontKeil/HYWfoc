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
│   │   ├── L2_Core/
│   │   │   ├── Control/         ← 控制算法（Source Manager、Source、估计器、外环/电流环、特殊模式）
│   │   │   ├── Protocol/        ← 协议帧解析、命令执行、输出适配
│   │   │   └── Runtime/         ← 调度器、环形队列、调试流生成器
│   │   └── L3_Hal/              ← 数学变换、平台抽象API、传感器采样、SVPWM、滤波器数学
│   └── src/
│       ├── L1_Orchestration/
│       └── L2_Core/
│           ├── Control/
│           ├── Protocol/
│           └── Runtime/
│       └── L3_Hal/
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
| `L1` 编排层 | `foc_core/src/L1_Orchestration/` | 启动流程、实例化核心数据结构（`foc_motor_t`、`foc_system_t`）、主循环编排、实例化和持有所有队列（comm RX FIFO、output TX FIFO、monitor element FIFO）、调度器/指示器管理 | **持有所有运行时实例**（系统结构体、队列缓冲区、调度器、调试流状态） |
| `L2/Control` | `foc_ctrl_*.c` | 控制算法：Source Manager、OpenLoop angle source 与 low-speed policy、估计器体系（编码器/SMO/HFI/FLUX）、外环、电流环、参数学习、补偿、有感齿槽标定、有感重初始化、执行输出 | 不持实例，操作传入的 `foc_motor_t` 指针 |
| `L2/Protocol` | `foc_protocol_handler.c`、`foc_protocol_output.c`、`foc_protocol_parser.c` | **单帧处理**：解析一帧 → 修改 motor 字段 → 返回结果结构体。不读帧、不入队、不轮询 | 不持实例，工作所需指针由 L1 传入（系统 report 配置） |
| `L2/Runtime` | `foc_task_scheduler.c`、`foc_queue.c`、`foc_debug_stream.c` | 调度器（任务速率管理）；环形队列（**纯方法模块**，不持实例，调用者传入队列指针）；调试流生成器（提供 PollNextValue + 格式化接口，由 L1 双上下文调用） | 队列类型可实例化，但实例在 L1 分配；调度器/调试流实例由 L1 持有 |
| `L3` 基础服务层 | `foc_core/src/L3_Hal/` | 数学变换、LUT、平台抽象API、传感器采样、SVPWM、滤波器数学 | 无实例（纯函数或操作 motor 中的字段） |
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

- **`foc_motor_t`**（定义于 `foc_ctrl_types.h`）— 电机控制数据结构，包含控制参数、状态、PID、估计器状态（`estim_smo_state`、`estim_hfi_state`、`estim_encoder_state`）、Source Manager 状态（`active_source_state`、`source_mgr_state`、`source_switch_state`）、各 source 私有状态（如 `openloop_state`）和控制运行时（`ctrl`: `iq_target`、`electrical_angle_rad`、`ud/uq`）、外环状态等。L1 实例化，L2 各块通过指针读/写。
- **`foc_system_t`**（定义于 `foc_system_types.h`）— 系统级数据结构，包含：
  - `cfg.report`：系统 report 配置，不随 reinit 重置
  - `runtime.scheduler`：系统任务调度器
  - `runtime.tasks`：ISR → main loop 任务触发标志
  - `runtime.comm`：通信输入轮询状态与 RX FIFO
  - `runtime.output`：文本输出 TX FIFO
  - `runtime.monitor`：DebugStream 状态、monitor element FIFO、示波器行累积状态
  - `runtime.indicator`：LED/指示器运行状态

### Source/Control 数据流（采样、角度来源与控制量生成解耦）

```
 PWM ISR：Source / 电流快线
  电流采样：Sensor_ReadCurrent → sensor_fast
  Estimator 后台迭代：SMO/HFI 更新内部状态（无论是否 active）
  Source Manager：Select(决策) → Publish(发布)：
    Select：读取各 source 内部状态、速度阈值、收敛状态 → 决定 active source / control_region
    Publish：从 active source 私有状态读数据 → 填入 active_source_state → 派生电机角度
  电流环：消费 motor->ctrl.iq_target + motor->ctrl.electrical_angle_rad → SVPWM
                        │
                        ▼
 Control ISR：Control 慢线
  慢速 sensor/VBUS/current snapshot 同步
  读取 active_source_state：
    OpenLoop active → OpenLoopLowSpeedPolicy 生成 iq_target
    其他 source active → 外环读取 active_source_state 角度/速度 生成 iq_target
  齿槽补偿在允许时叠加 iq offset
  特殊 phase：推进状态机并记录 phase_output_state
```

**关键约束**：
- 控制算法读取 `active_source_state` 生成控制量，不直接接触任何 source 的私有状态
- Source Manager 在 PWM ISR 中分两步运行：`Select`（纯决策）→ `Publish`（纯发布），决策与数据拷贝分离
- Publish 阶段写入 `motor->ctrl.electrical_angle_rad`（电流环消费的快路径角度）
- Control ISR 将生成的 `iq_target` 写入 `motor->ctrl.iq_target`，由 PWM ISR 电流环消费
- 所有 Estimator 在每个 PWM ISR 周期都后台运行（预收敛），无论其是否为 active source

## 数据流设计

### 总体架构

```
 ISR 上下文
  调度器回调触发 Service/Monitor/Control 任务
  Service ISR: 读平台帧 → 入 RX 队列
  Monitor ISR: 快照 sensor/motor 关键字段 →
               DebugStream_PollNextValue 逐元素
               入 runtime.monitor.elem_fifo
  Control ISR: 传感器同步→source view 读取→控制策略/特殊状态机
  PWM ISR: phase 路由→采样/插值→Estimator 迭代→Source 选择发布→电流环→SVPWM
                        │
                        ▼
 主循环上下文
  FOC_App_Loop():
    1. Monitor 段: monitor element FIFO 出队 →
       switch(tag): FormatSemanticLine/AppendOsc
       → FIFO_Enqueue(runtime.output.tx_fifo)
    2. Service 段: RX 队列 → 协议处理 → 编排
       (needs_summary → output TX FIFO)
    3. TX 队列出队 → 平台发送
```

### 协议数据流（输入 → 处理 → 输出）

```
平台 UART ISR
  │
  ▼ （Service 触发回调中）
读帧（FOC_Platform_CommSource*_ReadFrame）
  │
  ▼
FIFO_Enqueue(runtime.comm.rx_fifo)   ← L2/Runtime 队列方法，操作 L1 的队列实例
  │
  ▼ （主循环 Service 段）
FIFO_Dequeue(runtime.comm.rx_fifo)   ← L2/Runtime 队列方法
  │
  ▼
FOC_Protocol_ProcessSingle() ← L2/Protocol 单帧处理
  │   解析帧 → 修改 motor 字段 → 返回结果结构体
  │
  ├── [comm_active]  → 更新 LED 指示器
  ├── [needs_summary] → L1 生成摘要文本 →
  │                      FIFO_Enqueue(runtime.output.tx_fifo)
  ├── [needs_status]  → 状态码已在协议内部直写（快路径）
  └── [param_changed] → L1 稍后检测 cfg_dirty → ApplyConfig
```

### 双输出路径

| 路径 | 机制 | 用途 | 执行位置 |
|------|------|------|---------|
| **快路径**（直写） | 直接调用 L3 平台 API（`FOC_Platform_Write*`） | 状态码、参数行、错误回报等短数据 | ISR 或协议处理函数内部 |
| **慢路径**（队列） | ISR: FIFO_Enqueue(runtime.monitor.elem_fifo) → 主循环: FIFO_Dequeue → 格式化(tag switch) → FIFO_Enqueue(runtime.output.tx_fifo) → 平台发送 | 语义遥测、示波器帧、协议摘要等多行数据 | 入队在 MonitorTrigger ISR（快照 + PollNextValue），格式化+入 TX 在主循环 Monitor 段，出 TX 由 L1 统一消费 |

**快路径的特点**：短小、可打断队列输出、不在乎阻塞（因为很短）。
**慢路径的特点**：大数据量、需要缓冲、通过队列解耦生产者与消费者。

### Monitor 元素队列机制

`runtime.monitor.elem_fifo` 是 L1 持有的轻量标记元素队列，统一 Monitor 慢输出路径：

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

1. **L2 层只调方法，不持实例**。队列方法定义在 L2/Runtime，但实例在 L1 的 `foc_runtime_ctx_t` 中，并按 `comm`、`output`、`monitor` 子结构归类。
2. **L2 层不碰队列操作**。协议处理只返回结果结构体，调试流提供 ISR 安全的 `PollNextValue` 接口和主循环格式化函数，入队/出队由 L1 编排。
3. **L1 是唯一编排者**。ISR 读帧→入 comm RX 队列、ISR 快照→入 monitor element 队列、主循环出队→处理→入 output TX 队列、TX 出队→发送，全由 L1 控制。
4. **DebugStream 双接口**：`PollNextValue`（ISR 上下文调用，跑 state machine 取值）和 `Format*` 函数（主循环上下文调用，格式化字符串），两者分离确保采样时机正确。
5. **DebugStream 数据源**：控制角度从 `active_source_state.mech_angle_rad` 读取，原始编码器角度仍从 `sensor.mech_angle_rad` 读取。

## 控制算法链

### 控制模块结构

L2/Control 按 `foc_ctrl_<name>.c/.h` 命名，模块划分：

| 文件名 | 职责 |
|------|------|
| `foc_ctrl_executor` | 算法入口：PWM ISR 与 Control ISR 路由，外环调度、控制模式切换 |
| `foc_ctrl_init` | 初始化与标定 |
| `foc_ctrl_cfg` | 配置状态管理（软切换、齿槽补偿、PID 初始化、fine-tuning setter） |
| `foc_ctrl_source_mgr` | Source Manager：Select（切换决策）+ Publish（发布 active source view） |
| `foc_ctrl_openloop` | OpenLoop angle source 实现 + OpenLoop low-speed policy |
| `foc_ctrl_estim` | 估计器选择/注册中心 |
| `foc_ctrl_estim_encoder` | Encoder source 实现（从 sensor 读原始角度，写入 source snapshot） |
| `foc_ctrl_estim_smo` | SMO source 实现 + 收敛跟踪 |
| `foc_ctrl_estim_hfi` | HFI source 实现 |
| `foc_ctrl_outer_loop` | 速度/位置外环 |
| `foc_ctrl_current_loop` | 电流内环 |
| `foc_ctrl_param_learn` | 电机参数学习 |
| `foc_ctrl_compensation` | 齿槽补偿 |
| `foc_ctrl_sens_cogging_calib` | 有感齿槽标定（非阻塞状态机，由 L1 通过 control_phase 路由调用） |
| `foc_ctrl_sens_reinit` | 有感非阻塞重初始化（由 L1 通过 control_phase 路由调用） |
| `foc_ctrl_actuation` | 执行输出（SVPWM 驱动） |

### 控制运行链

```
初始化链：FOC_MotorInit → FOC_ControlConfigResetDefault
       → FOC_Estim* / OpenLoop source/policy 初始化
       → FOC_SourceMgr_Init(LOW_SOURCE, HIGH_SOURCE)
       → FOC_ControlExecutor_Init → FOC_Control_ApplyConfig

Control ISR（低频控制线，严格不做 source 选择）：
  阶段0：系统守卫（L1）
    → system_fault 检查 → return
  阶段1：传感器读取
    → [SLOW] Sensor_ReadEncoder、Sensor_ReadVBUS
    → 有效性检查（adc_valid + [encoder] encoder_valid）
    → 欠压保护检查（FOC_FEATURE_UNDERVOLTAGE_PROTECTION）
  阶段2：按 control_phase 运行状态机
    → NORMAL：Control Policy 路由（FOC_ControlExecutor_RunCycle）
      → OpenLoop active → FOC_OpenLoop_RunStep（写 motor->ctrl.iq_target）
      → 其他 source active → FOC_ControlExecutor_RunOuterLoop：
        根据 control_mode 选择外环 → FOC_SpeedOuterLoopStep / FOC_SpeedAngleOuterLoopStep
        → 齿槽补偿（FOC_ControlApplyCoggingCompensation，使用 active_source_state.mech_angle_rad）
    → COGGING_CALIB/REINIT：特殊状态机记录 `phase_output_state`

PWM ISR（5 阶段管线，严格串行，不可调换）：
  [L1 系统守卫]
    → system_fault 检查 → SafeOutput + return
    → system_running / motor_enabled 检查 → return

  [公共前导] SVPWM_InterpolationISR
  [特殊 phase 路由]（管线内部行为）
    → COGGING_CALIB/REINIT：直接消费 phase_output_state → apply → return，跳过后续 NORMAL 流程

  [NORMAL 标准流程——电流分频控制]：
  阶段1：硬件采样
    → [FAST] Sensor_ReadEncoder → Sensor_AccumulateEcycle
    → Sensor_ReadCurrent（若 current_loop_ready）
  阶段2：Estimator 数值迭代（后台运行，所有启用的 Estimator 都执行）
    → FOC_EstimSMO_Step（更新内部 bemf、PLL、收敛计数器）
    → FOC_EstimHFI_Step（更新内部高频注入解调状态）
    → [注] 无论 active source 是否为 SMO/HFI，均执行迭代——预收敛机制
  阶段3：Source Manager
    → FOC_SourceMgr_Select：纯决策
      读取各 source 状态、速度、收敛度
      切换判据（阈值/收敛/消抖）决定 active/standby source、control_region
      不写 active_source_state、不改电机角度
    → FOC_SourceMgr_Publish：纯发布
      从 active source 私有状态读数据
      填 active_source_state（角度/速度/置信度等）
      派生写 motor->ctrl.electrical_angle_rad
      更新 encoder_services
  阶段4：NORMAL 电流环
    → FOC_CurrentControlStep（Clarke/Park → PID → ud/uq）
  阶段5：SVPWM 输出
    → FOC_ControlApplyElectricalAngleRuntime

配置应用：
  FOC_Control_ApplyConfig(motor)
    → 从 motor 结构体读取 PID 参数和 fine-tuning 设置
```

**关键执行顺序说明**：
- Estimator 迭代在 Source Manager 决策之前，确保切换判据使用**当前周期**的收敛状态
- Source Manager 的 Select 和 Publish 是两步分离的：Select 只做决策不拷贝数据，Publish 只拷贝数据不做决策
- 电流环在发布之后，消费已发布的 `motor->ctrl.electrical_angle_rad`
- 电流分频：`FOC_CURRENT_LOOP_ISR_DIVIDER` 控制每 N 个 PWM 周期执行一次完整电流环，中间的 PWM 周期只做插值和 Estimator 迭代

### 采样路径规则

1. **电流采样独占性**：ADC 电流读取全部归 PWM ISR 独占（`Sensor_ReadCurrent`），控制 ISR 不再直接读 ADC。
2. **编码器角度路径选择**：由 `FOC_SENSOR_ANGLE_FAST_ENABLE` 宏联动：
   - `DISABLE`（慢速编码器，如 I2C AS5600）：在 Control ISR 中读取（`Sensor_ReadEncoder`）。
   - `ENABLE`（快速编码器，如霍尔/QEI）：在 PWM ISR 中同步读取。
3. **控制 ISR 电流数据来源**：通过 `Sensor_SyncCurrentSnapshot` 从 `motor->sensor_fast` 复制到 `motor->sensor`。
4. **PWM ISR 角度同步**：电流环之后仅拷贝 `raw_value` 和有效性标志到 `motor->sensor`，不完整拷贝滤波器完整状态。
5. **L3 平台 API**：统一为单一 `FOC_Platform_ReadPhaseCurrent`，无 `Fast/Slow` 双入口。

## 控制阶段枚举与运行区域

### control_phase

```c
typedef enum {
    FOC_CONTROL_PHASE_NORMAL        = 0U,  // 正常控制
    FOC_CONTROL_PHASE_COGGING_CALIB = 1U,  // 有感齿槽标定
    FOC_CONTROL_PHASE_REINIT        = 2U   // 有感重初始化
} foc_control_phase_t;
```

`control_phase` 表示当前顶层控制模式，决定 Control ISR 的状态机入口和 PWM ISR 的输出流程路由。
低速/高速、OpenLoop/SMO/Encoder/HFI 切换不通过 `control_phase` 表示，只属于 NORMAL 标准流程内部的 source/control 状态。

### control_region

```c
typedef enum {
    FOC_CONTROL_REGION_LOW  = 0U,  // 低速域（初始或回退状态）
    FOC_CONTROL_REGION_HIGH = 1U,  // 高速域（已切到 high source）
    FOC_CONTROL_REGION_FULL = 2U   // 全域（low=high，无切换必要）
} foc_control_region_t;
```

`control_region` 是 Source Manager 发给 Control ISR 的运行区间提示，不替代 `control_phase`：

| region | 含义 |
|--------|------|
| `LOW` | 初始状态，或从 high 回退到 low。Control Policy 应使用低速策略（如 OpenLoop low-speed policy） |
| `HIGH` | 已成功切换到 high source。Control Policy 可使用普通速度/位置外环 |
| `FULL` | low=high（single source 场景），无需切换，外环始终可用 |

`control_region` 的赋值：
- 初始化时：若 high_source == low_source 或 high= NONE → `FULL`；否则 `LOW`
- 切换完成时：切到 high → `HIGH`；回退到 low → `LOW`

## Source Manager 体系

### Source Manager 两步分离设计

Source Manager 在 PWM ISR 中分两步运行，决策与数据发布解耦：

```
FOC_SourceMgr_Select(motor)
  └── 纯决策函数：
      ├── 读取低/高 source 配置、当前 active source、各 source 内部状态
      ├── 检查速度阈值、低速侧升域授权、候选高速源收敛状态
      ├── 如果 single source（low=high）→ 直接返回，control_region = FULL
      ├── 低→高判断：LowMotionAbove(high_th) + CandidateSpeedAbove(high_th) + high 可获取
      ├── 高速获取：使用 low_th 作为取消门限，避免 high_th 附近反复清零
      ├── 高→低判断：high 失效/发散或速度低于 low_th 后连续确认
      ├── 消抖窗口：条件连续满足 FOC_SOURCE_SWITCH_SETTLE_CYCLES 后才提交切换
      └── 切换提交：修改 active_source、standby_source、control_region，并同步外环/电流环状态

FOC_SourceMgr_Publish(motor)
  └── 纯发布函数：
      ├── 根据 active_source 类型读取对应 source 的私有状态
      │   ├── ENCODER → sensor.mech_angle_rad
      │   ├── SMO → estim_smo_state.pll_angle_rad / pll_speed_rad_s
      │   └── OPENLOOP → openloop_state.virtual_angle_rad / mech_speed_rad_s
      ├── 填入 motor->active_source_state（source、state、valid、confidence、角度）
      ├── 统一写 motor->ctrl.electrical_angle_rad（电流环快路径角度）
      └── 更新 motor->encoder_services（calib/reinit/comp 可用性）
```

### Source 体系与预收敛机制

角度/速度来源统一称为 Source。所有启用的 Estimator（SMO、HFI）在每个 PWM ISR 周期都执行迭代，无论其是否为当前 active source。这种**后台预收敛**机制确保 source 切换时目标源已准备好。

| Source | 类型宏 | 私有状态 | 迭代位置 | 收敛跟踪 |
|--------|--------|----------|----------|----------|
| OPENLOOP | `FOC_SOURCE_TYPE_OPENLOOP` | `openloop_state` | Control ISR 积分虚拟角度，PWM ISR Publish 读取 | 非 FAILED 即有效 |
| ENCODER | `FOC_SOURCE_TYPE_ENCODER` | `estim_encoder_state` | Control ISR 或 PWM ISR（由 FAST 宏控制） | 硬件有效即 LOCKED |
| SMO | `FOC_SOURCE_TYPE_SMO` | `estim_smo_state` | PWM ISR 阶段2（每周期迭代） | converge_counter / lock_counter 三段状态 |
| HFI | `FOC_SOURCE_TYPE_HFI` | `estim_hfi_state` | PWM ISR 阶段2（每周期迭代） | 预留 |
| FLUX | `FOC_SOURCE_TYPE_FLUX` | 预留 | 预留 | 预留 |

SMO 收敛状态定义：
- `converge_counter`：反电势幅值超过阈值时的连续计数
- `lock_counter`：锁相环误差持续低于门限时的连续计数
- `rot_dir_counter`：旋转方向一致性计数
- 状态映射：`converge_counter < CONVERGE_CONSECUTIVE(50)` → `INIT`
  - `converge_counter > CONVERGE(50)` → `CONVERGING`
  - `converge_counter > LOCK_CONSECUTIVE(100)` → `LOCKED`
  - `lock_counter > DIVERGE_CONSECUTIVE(200)` → `DIVERGED`

### Source Manager 切换状态机

Source Manager 在 `Select` 阶段维护显式 `region_state`。状态字段只描述速域/source 切换过程，不替代顶层 `control_phase`。

```
FULL_ACTIVE
  single-source 或非法切换配置；active=low，control_region=FULL

LOW_ACTIVE
  active=low，control_region=LOW
  -> HIGH_ACQUIRE 条件（四个全满足，使用 high_th）：
       LowMotionAbove(low, high_th)
       && CandidateSpeedAbove(high_speed, high_th)
       && high_state >= CONVERGING
       && high source valid

HIGH_ACQUIRE
  active 仍保持 low，switch_counter 连续累计
  -> LOW_ACTIVE 条件（任一满足即回退，使用 low_th）：
       LowMotionAbove(low, low_th) == false
       || CandidateSpeedAbove(high_speed, low_th) == false
       || high_state 不可获取
       || high source invalid
  -> HIGH_ACTIVE 条件（消抖达标后直接检查切入条件，无中间 HIGH_READY 状态）：
       switch_counter >= FOC_SOURCE_SWITCH_SETTLE_CYCLES
       && high_state == LOCKED
       && (active 是 OPENLOOP || active/high 电角度兼容)
     动作：CommitSwitch(high)
  （若消抖达标但切入条件不满足，保持在 HIGH_ACQUIRE 中等待）

HIGH_ACTIVE
  active=high，control_region=HIGH
  -> HIGH_SUSPECT 条件（降级嫌疑，任一满足即转入）：
       high_state == DIVERGED
       || high_state 不可保持
       || high source invalid
       || speed_abs < low_th

HIGH_SUSPECT
  降级消抖状态
  -> HIGH_ACTIVE 条件（嫌疑解除，使用 high_th 为恢复门限）：
       high_state != DIVERGED
       && high_state 可保持
       && high source valid
       && (speed 无效 || speed_abs >= high_th)
  -> LOW_RECOVERY 条件：
       high_state == DIVERGED（立即）
       || switch_counter >= FOC_SOURCE_SWITCH_SETTLE_CYCLES（消抖完成）

LOW_RECOVERY
  low source valid 或 low=OPENLOOP 时 CommitSwitch(low) 并回到 LOW_ACTIVE
```

低速侧升域授权规则：
- `low=OPENLOOP`：只认 `openloop_state.mech_speed_rad_s`，外部拖动 encoder 速度不能单独触发升域。
- `low` 有物理速度：使用该 source 的物理速度。
- `low` 无物理速度：使用 `outer_loop.ramped_speed_rad_s` 作为控制意图。

切换提交同步：
- 重基准 `outer_loop.accum_rad / prev_rad / prev_mech_signed_rad`。
- 清 `speed_err_accum_rad` 并同步 `speed_state_valid`。
- 使用新源或旧源物理速度同步 `outer_loop.ramped_speed_rad_s`。
- 按切换前 `iq_target/uq/iq_measured` 预置速度 PID 与电流 PID，避免闭环从零状态硬接管。

### 全局控制加速度与速域上限

速度斜坡由 `outer_loop.ramped_speed_rad_s` 统一承载：
- OpenLoop 低速源使用 `openloop_state.ramp_rate_rad_s2` 积分虚拟电角速度，并受低速域速度上限约束。
- 普通速度/位置外环通过 `FOC_Accel_ApplySpeedLimit()` 施加加速度斜率和 `control_region` 对应速度上限。
- Source Manager 切换提交时同步 ramped speed，保证低速/高速域切换不会把速度斜坡状态清零或反向跳变。

### 初始化配置

```c
FOC_SourceMgr_Init(motor, low_source, high_source):
  active_source = low_source
  standby_source = switchable ? high_source : NONE
  control_region = (high == NONE || high == low) ? FULL : LOW
  region_state = switchable ? LOW_ACTIVE : FULL_ACTIVE
  config_valid = switchable
  switch_state.low_source = low_source
  switch_state.high_source = high_source
  active_source_state 所有字段清零（source=low_source, state=INIT, valid=0）
  UpdateEncoderServices：根据 active_source 是否为 ENCODER 设 calib/reinit/comp 可用性
```

| 组合场景 | active | standby | control_region | 初始 Control Policy |
|----------|--------|---------|----------------|---------------------|
| Encoder 全速域（low=Encoder, high=NONE） | Encoder | NONE | FULL | 普通速度/位置外环 |
| OpenLoop→SMO（low=OpenLoop, high=SMO） | OpenLoop | SMO | LOW | OpenLoop low-speed policy |
| Encoder→SMO（low=Encoder, high=SMO） | Encoder | SMO | LOW | 普通速度/位置外环 |

### encoder_services 绑定

`foc_encoder_services_state_t` 记录了 Encoder 相关的专属服务可用性，唯一写入入口是 `SourceMgr_UpdateEncoderServices`（由 Publish 阶段内部调用）：

- active_source == ENCODER：`calib_available=1`、`reinit_available=1`、`comp_available` 由 cogging 表决定
- active_source != ENCODER：四个标志清零

标定(COGGING_CALIB)和重初始化(REINIT)的入口门控即 `calib_available` / `reinit_available`，
因此只能在 active source 为 ENCODER 时进入。

## 调度模型

调度器位于 `L2/Runtime/foc_task_scheduler`，管理三种任务速率：

- **服务任务（中速）**：ISR 中读帧入 RX 队列 + 更新指示器，主循环中出队解析、参数同步
- **控制主循环（快速）**：Control ISR 同步慢速数据、读取已发布 active source view、根据 source 类型和 control_region 选择 Control Policy、生成 `iq_target`
- **监测任务（低速）**：ISR 中快照 sensor 数据 → 入 runtime.monitor.elem_fifo，主循环中出队格式化输出

控制节拍源与 PWM 更新中断源分离：
- 控制节拍源驱动调度器回调（如 Control ISR）
- PWM 更新中断源驱动高速电流环、Estimator 迭代、Source Manager 和特殊输出路径
- 采样触发与 PWM 对齐

## 宏裁剪口径

### 算法特性开关（LS）

定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`

1. 传感器硬件：`FOC_SENSOR_ENCODER_ENABLE`
2. 估计器：`FOC_ESTIMATOR_ENCODER_ENABLE`、`FOC_ESTIMATOR_SMO_ENABLE`、`FOC_ESTIMATOR_HFI_ENABLE`、`FOC_ESTIMATOR_FLUX_ENABLE`
3. OpenLoop 角度源/低速策略：`FOC_OPENLOOP_SOURCE_ENABLE`
4. Source 切换：`FOC_SOURCE_SWITCH_ENABLE`
5. 齿槽补偿特性（`FOC_COGGING_COMP_ENABLE` + `FOC_COGGING_CALIB_ENABLE`）
6. 采样滤波特性（Kalman、LPF、电气周期偏移补偿）

### 常见功能宏组合

| 场景 | 配置 |
|------|------|
| 有感 FOC | `SENSOR_ENCODER=ENABLE`, `ESTIMATOR_ENCODER=ENABLE`, `SMO/HFI=DISABLE`, `COGGING=ENABLE` |
| 强拖→SMO 无感 | `SENSOR_ENCODER=DISABLE`, `ESTIMATOR_ENCODER=DISABLE`, `SMO=ENABLE`, `OPENLOOP_SOURCE=ENABLE` |
| HFI→SMO 无感 | `SENSOR_ENCODER=DISABLE`, `SMO=ENABLE`, `HFI=ENABLE`, `SOURCE_SWITCH=ENABLE` |

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
   - SMO 需至少一种低速角度 source（当编码器不可用时）
   - Source 切换配置需满足 low/high source 组合约束

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

`foc_cfg_filter.h` 中的参数按 **7 个滤波器位置**分组：

| 位置分组 | 前缀 | 包含参数 | 算法 |
|----------|------|----------|------|
| SENSOR_CURRENT_A | `FOC_FILTER_SENSOR_CURRENT_A_KALMAN_*` / `LPF_ALPHA` | MEAS_ERR, EST_ERR, PROC_NOISE, INIT, LPF_ALPHA | Kalman + LPF1 |
| SENSOR_CURRENT_B | `FOC_FILTER_SENSOR_CURRENT_B_KALMAN_*` / `LPF_ALPHA` | 同上 | Kalman + LPF1 |
| SENSOR_CURRENT_C | `FOC_FILTER_SENSOR_CURRENT_C_KALMAN_*` / `LPF_ALPHA` | 同上 | Kalman + LPF1 |
| SENSOR_ANGLE | `FOC_FILTER_SENSOR_ANGLE_KALMAN_*` / `LPF_ALPHA` | 同上 | Kalman + LPF1 |
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
