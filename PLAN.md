# 结构体变量重组与架构改造 — 详细评估与执行计划

> **类型**：备忘录 / 评估报告  
> **关联任务**：核心库变量/结构体重组 + NEXT_MISSION P1 协议队列化  
> **状态**：计划阶段，待实施

---

## 一、当前问题评估

### 1.1 全局变量分布问题

| 层级 | 文件 | 全局变量数 | 性质 | 问题 |
|------|------|-----------|------|------|
| L1 | `foc_app.c` | 6 | 系统标志+指示器 | 分散声明，缺乏统一管理 |
| L2 | `foc_task_scheduler.c` | 3 | 调度器状态 | 违背"L2 不应有全局变量"原则 |
| L2 | `foc_debug_stream.c` | 7 | 调试流状态 | 同上 |
| L2 | `foc_protocol_handler.c` | 2 | 遥测策略+报告模式 | 同上 |
| L3 | `foc_svpwm.c` | 11 | SVPWM 插值引擎 | 多电机不兼容 + 输出快照冗余 |

**违反架构原则**：L2 层出现了 4 个文件共 12 个 static 全局变量。这些变量本质上属于不同的系统模块（调度器、调试流、协议），应由 L1 层实例化后以参数传递。

### 1.2 结构体类型存储位置混乱

当前所有结构体类型定义堆积在 `LS_Config/` 目录下，违背层级归属原则：

| 类型 | 当前 | 应属层级 | 理由 |
|------|------|---------|------|
| `kalman_filter_t`, `foc_pid_t` | LS | **L3** | 数学/滤波器基础类型，被 sensor 和 control 共用 |
| `protocol_command_t`, 解析枚举 | LS | **L2/Protocol** | 协议层核心类型 |
| `telemetry_policy_snapshot_t` | LS | **L2/Protocol** | 遥测策略由协议管控 |
| `FOC_TaskRate_t` | LS | **L2/Runtime** | 调度器核心枚举 |
| `runtime_step_signal_t`, `init_check_t` | LS | **L2/Runtime** | 运行时信号类型 |
| `foc_motor_t` 及所有子结构体 | LS | **L2** | 控制算法核心数据结构，被 L2 各子模块交叉引用 |

*sensor_data_t 例外：与 motor 结构体紧耦合（内嵌 sensor/sensor_fast），为保持完整性暂不拆分。*

### 1.3 cfg/status 双重存储的设计缺陷

当前结构：
```
协议 WriteParam → motor->cfg (cfg_dirty=1)
                    ↓ Service 任务
                    FOC_Control_ApplyConfig → 同步到 status/PID
                                               ↓
                    控制算法从 status/PID 读取
```

**问题**：
1. **控制算法实际混合读取**：`motor->cfg.control_mode` 直接从 cfg 读，而软切换从 status 读，fine-tuning 从 `control_runtime_cfg` 读，PID 从 PID 对象读 — 三个不同的数据源
2. **缓冲未起作用**：WriteParam 到 ApplyConfig 之间的窗口期，ISR 可能读到新旧混杂的配置（如 cfg 已更新但 status 未同步）
3. **同步代码冗余**：`FOC_Control_ApplyConfig` 中 15+ 行纯同步代码，每次新增配置项都要同步修改两处

### 1.4 SVPWM 输出快照冗余

`foc_svpwm.c` 内部维护 `s_output` 全局变量，而 `foc_motor_t` 中也有 `duty_a/b/c/sector`。当函数写入了 motor 的 duty 后，SVPWM 内部又更新 s_output：

```
foc_ctrl_actuation.c:
  FOC_ControlApplyElectricalAngleCore
    → motor->duty_a = ...
    → SVPWM_UpdateRuntime(..., &motor->duty_a, &motor->sector)  // 写 motor
    → SVPWM_CalculateDuty 内部: s_output.duty_a = ...          // 又写 s_output
```

两个数据副本，路径不一致，存在时序风险。

### 1.5 协议输出链路问题

当前 Monitor 任务和服务任务直接调用 `FOC_Platform_WriteDebugText`，在主循环上下文中**阻塞式发送**：

```
Monitor 任务回调 (定时器):
  DebugStream_Process → snprintf → FOC_Platform_WriteDebugText → UART 发送
                          ↑                                ↑
                      读 sensor/motor               阻塞等待 TX 完成

Service 任务回调 (定时器):  
  FOC_Protocol_Process → 解析 → 执行 → snprintf → FOC_Platform_WriteDebugText
  
主循环:
  FOC_App_Loop → 无数据发送
```

问题：
1. 发送在定时器回调中执行，发送耗时可能影响下一次定时器触发
2. 示波器帧与参数回报混杂在同一条输出流中，无优先级区分
3. 大量文本输出（如 `Y:R` 运行时摘要、齿槽标定 Dump）会阻塞控制流程

---

## 二、L1 系统上下文设计

### 2.1 分离原则

**系统运行时上下文**（`foc_runtime_ctx_t`）与**系统配置**（`foc_system_cfg_t`）分开，原因：

| 方面 | 运行时上下文 | 系统配置 |
|------|-------------|---------|
| 生命周期 | 每次 reinit 重置 | 跨越 reinit 持久保持 |
| 内容 | 调度器 tick、LED 闪烁计数、任务标志、调试流计数器 | 遥测策略、报告模式 |
| 修改者 | 系统自身运行时变化 | 协议命令 **P/S** 通道修改 |
| 示例 | `g_sched_tick_counter`, `s_led_run_blink_counter` | `g_telemetry.semantic_report_freq_hz` |

如果合并在一个结构体中，`FOC_Service_ReInitMotor` 内部重置 motor 时可能会不小心重置遥测策略 — 违反"协议配置应持久保持"的设计。

### 2.2 顶层结构

```c
// foc/include/L1_Orchestration/foc_system_types.h

/* === 系统配置（不随 reinit 重置） === */
typedef struct {
    telemetry_policy_snapshot_t telemetry;  // 遥测策略（协议设置）
    uint8_t report_mode;                    // 报告模式
    /* P1 +: 队列配置 */
} foc_system_cfg_t;

/* === 系统运行时状态（每次 reinit 重置） === */
typedef struct {
    /* 调度器 */
    struct {
        uint16_t tick_counter;
        uint32_t execution_cycles;
        ControlScheduler_Callback_t callbacks[FOC_TASK_RATE_COUNT];
    } scheduler;
    
    /* 调试流 */
    debug_stream_state_t debug_stream;
    
    /* 任务触发标志 */
    volatile uint8_t service_task_pending;
    volatile uint8_t monitor_task_pending;
    
    /* 指示器 */
    struct {
        uint16_t comm_pulse_counter;
        uint8_t  led_run_on;
        uint16_t led_run_blink_counter;
    } indicator;
} foc_runtime_ctx_t;

/* === 系统顶层聚合 === */
typedef struct {
    foc_system_cfg_t cfg;        // 持久配置
    foc_runtime_ctx_t runtime;   // 运行时状态
    foc_motor_t motor;           // 电机实例
} foc_system_t;
```

### 2.3 API 传递路径

```
L1 (foc_app.c):
  static foc_system_t g_sys;
  → g_sys.cfg          传递给协议（持久侧）
  → g_sys.runtime      传递给调度器/调试流/指示器
  → g_sys.motor        传递给控制算法

L2 API 变化:
  ControlScheduler_Init(foc_runtime_ctx_t *ctx)
  DebugStream_Init(foc_runtime_ctx_t *ctx)
  DebugStream_Process(foc_runtime_ctx_t *ctx, sensor_data_t *sensor, foc_motor_t *motor)
  FOC_Protocol_Process(foc_motor_t *motor, foc_system_cfg_t *sys_cfg, uint8_t frame_budget)
  FOC_Protocol_GetTelemetry(const foc_system_cfg_t *sys_cfg)
```

---

## 三、Phase 详解

---

### Phase 0：类型定义重定位

**目标**：将类型定义从 `LS_Config/` 移到各自所属层级，使 `#include` 路径反映真实的层级依赖。

**理由**：当前 ls_Config 混放宏配置 + 类型定义 + 数据表，违反了"配置宏收敛在 LS、类型定义归属各自层级"的原则。移动后可实现：
- `foc_ctrl_types.h` 属于 L2，被 L2/Control/Protocol/Runtime 引用，路径 `#include "L2/foc_ctrl_types.h"` 明确表示这是 L2 层类型
- 编译期可验证层级依赖：L3 文件不应 `#include "L2/..."`，L2 文件不应 `#include "L1/..."`

#### 影响矩阵

| 源文件（LS） | 目标文件 | 影响引用数 | 主要被谁引用 |
|-------------|---------|-----------|------------|
| `foc_math_types.h` | `L3/foc_math_types.h` | ~8 | sensor.c, ctrl_*.c, protocol_*.c |
| `foc_motor_types.h` | `L2/foc_ctrl_types.h` | ~15 | 几乎所有 L2/L1 文件 |
| `foc_protocol_types.h` | `L2/Protocol/foc_protocol_types.h` | ~3 | protocol_handler/parser/output |
| `foc_snapshot_types.h` | `L2/Protocol/foc_snapshot_types.h` | ~4 | protocol, debug_stream |
| `foc_scheduler_types.h` | `L2/Runtime/foc_scheduler_types.h` | ~3 | task_scheduler |
| `foc_runtime_types.h` | `L2/Runtime/foc_runtime_types.h` | ~3 | protocol, service_handler |

#### sensor_data_t 不拆分的原因说明

`sensor_data_t` 定义在 `foc_motor_types.h` 中，与 `foc_motor_t` 同文件。它是 L3 传感器层的输出类型，但：
- `foc_motor_t` 内嵌了 `sensor_data_t sensor` 和 `sensor_data_t sensor_fast` 两个快照
- 如果拆分到 L3，则 `foc_ctrl_types.h` 需要 `#include "L3/foc_sensor_types.h"`，形成 L2→L3 依赖
- 当前 L2→L3 是允许的单向依赖，不违反架构

→ **暂不拆分**，整体随 `foc_motor_types.h` 移至 `L2/foc_ctrl_types.h`

---

### Phase 1：系统上下文引入 + L2 全局变量消除

**目标**：消除 L2 层所有 static 全局变量，改为 L1 实例化后传参。完成 L1 系统级状态统一管理。

**理由**：
- 架构要求"L2 层不应出现全局变量"——所有运行时状态应由 L1 管控
- 当前调度器/调试流/DebugStream 的全局变量使 L2 模块自持状态，无法支持多实例或单元测试
- 封装后 ReInitMotor 可安全重置 `foc_runtime_ctx_t.scheduler` 等而不会影响 `foc_system_cfg_t`

#### P1.1 定义系统上下文（3 个新头文件）

```
foc/include/L1_Orchestration/
├── foc_system_types.h    # foc_system_t, foc_system_cfg_t, foc_runtime_ctx_t
├── foc_scheduler.h       # control_scheduler_t + API 声明
└── foc_debug_stream.h    # debug_stream_state_t + API 声明（保留原文件合并）
```

#### P1.2 调度器重构

**当前**：
```c
// foc_task_scheduler.c — 3 static 全局
static volatile uint16_t g_sched_tick_counter;
static uint32_t g_sched_execution_cycles;
static ControlScheduler_Callback_t g_sched_callbacks[FOC_TASK_RATE_COUNT];
```

**目标**：
```c
// foc_task_scheduler.h
typedef struct {
    uint16_t tick_counter;
    uint32_t execution_cycles;
    ControlScheduler_Callback_t callbacks[FOC_TASK_RATE_COUNT];
} control_scheduler_t;

void ControlScheduler_Init(control_scheduler_t *sched);
void ControlScheduler_RunTick(control_scheduler_t *sched);
uint32_t ControlScheduler_GetExecutionCycles(const control_scheduler_t *sched);
```

#### P1.3 DebugStream 重构

**当前**：7 个 static 全局变量（计数器/频率/周期）

**目标**：封装为 `debug_stream_state_t`，API 接受 `foc_runtime_ctx_t *ctx`（通过 `&ctx->debug_stream` 访问）

#### P1.4 协议上下文

**当前**：`g_telemetry` 和 `g_report_mode` 为 static 全局

**目标**：`g_telemetry` 移至 `foc_system_cfg_t`，API 改为 `FOC_Protocol_Init(foc_system_cfg_t *sys_cfg)`

#### P1.5 L1 实例化

```c
// foc_app.c
static foc_system_t g_sys;

void FOC_App_Init(void)
{
    // 初始化系统配置
    g_sys.cfg.telemetry.semantic_report_enabled = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
    // ...
    
    // 初始化运行时
    ControlScheduler_Init(&g_sys.runtime.scheduler);
    DebugStream_Init(&g_sys.runtime.debug_stream);
    
    // 初始化协议
    FOC_Protocol_Init(&g_sys.cfg);
    
    // 初始化电机
    Sensor_InitSnapshot(&g_sys.motor.sensor);
    // ...
}
```

---

### Phase 2：SVPWM per-motor 化

**目标**：消除 `foc_svpwm.c` 的 11 个 static 全局变量，将插值引擎状态纳入 `foc_motor_t`。

**理由**：
1. **多电机兼容**：当前所有电机共享同一组插值状态 → 第二个电机会覆盖第一个电机的占空比
2. **消除数据冗余**：motora 中的 `duty_a/b/c` + `sector` 与 SVPWM 内部的 `s_output` 重复
3. **数据流清晰**：SVPWM 状态完全通过 motor 传递，不再有隐式全局状态

#### 状态结构体

```c
typedef struct {
    svpwm_output_t output;         // sector + duty_a/b/c（唯一输出副本）
    float duty_a_current;
    float duty_b_current;
    float duty_c_current;
    float duty_a_target;
    float duty_b_target;
    float duty_c_target;
    float duty_a_step;
    float duty_b_step;
    float duty_c_step;
    uint16_t interp_steps_total;
    uint16_t interp_step_index;
} svpwm_interp_state_t;
```

#### 数据流变化

```
当前：
  FOC_ControlApplyElectricalAngleCore
    → SVPWM_UpdateRuntime(..., &motor->duty_a, &motor->sector)  // 写 motor
    → SVPWM_CalculateDuty 又写 s_output (重复)

目标：
  FOC_ControlApplyElectricalAngleCore
    → SVPWM_UpdateRuntime(motor, ...)  // 直接写 motor->svpwm.output
    → 不再有 s_output
```

#### API 变更

| 当前签名 | 目标签名 |
|---------|---------|
| `SVPWM_Init(uint16_t freq_kHz, uint8_t deadtime_percent)` | `SVPWM_Init(foc_motor_t *motor, ...)` |
| `SVPWM_SetRuntimeDutyTarget(uint8_t sector, ...)` | `SVPWM_SetRuntimeDutyTarget(foc_motor_t *motor, ...)` |
| `SVPWM_UpdateRuntime(float phase_a, ..., uint8_t *sector_out, float *duty_a, ...)` | `SVPWM_UpdateRuntime(foc_motor_t *motor, float phase_a, ...)` |
| `SVPWM_InterpolationISR(void)` | `SVPWM_InterpolationISR(foc_motor_t *motor)` |
| `const svpwm_output_t* SVPWM_GetOutput(void)` | `const svpwm_output_t* SVPWM_GetOutput(const foc_motor_t *motor)` |

#### 移除冗余

`foc_motor_t` 原来有：
```c
float duty_a, duty_b, duty_c;  // 移除
uint8_t sector;                 // 移除
```
替换为：
```c
svpwm_interp_state_t svpwm;    // 通过 motor->svpwm.output.duty_a 访问
```

所有原引用 `motor->duty_a` 改为 `motor->svpwm.output.duty_a`，为了方便可以提供内联访问宏。

---

### Phase 3：cfg 冗余消除（Write-Through 方案A）

**目标**：移除 `foc_motor_cfg_t` 和 `foc_control_runtime_config_t`，实现单一配置源写穿透。

**理由**：
- 当前双重存储的**缓冲机制并未真正生效**（控制算法混合读取 cfg/status）
- 同步代码臃肿（`FOC_ControlConfigResetDefault` + `FOC_Control_ApplyConfig` 中共 30+ 行纯同步）
- 配置变更到生效之间存在窗口期，引入竞态可能性
- "写穿透"后协议参数立即生效，`cfg_dirty` 仅标记副作用需求

#### P3.1 字段去向总表

```
原 foc_motor_cfg_t 字段 → 目标位置
═══════════════════════════════════════════
target_angle_rad           → foc_motor_t 顶层 (motor->target_angle_rad)
angle_position_speed_rad_s → foc_motor_t 顶层
speed_only_rad_s           → foc_motor_t 顶层
sensor_sample_offset_percent → foc_motor_t 顶层
control_mode               → foc_motor_t 顶层 (与 state.control_mode 统一)
pid_current_kp/ki/kd       → motor->torque_current_pid
pid_angle_kp/ki/kd         → motor->angle_pid
pid_speed_kp/ki/kd         → motor->speed_pid
current_soft_switch_enable → motor->current_soft_switch_status.enabled
current_soft_switch_mode   → motor->current_soft_switch_status.configured_mode
current_soft_switch_auto_open/closed_iq_a → motor->current_soft_switch_status
cogging_comp_enable        → motor->cogging_comp_status.enabled
cogging_comp_iq_limit_a    → motor->cogging_comp_status.iq_limit_a
cogging_comp_speed_gate_rad_s → motor->cogging_comp_status.speed_gate_rad_s
cogging_calib_gain_k       → motor->cogging_comp_status.calib_gain_k
min_mech_angle_accum_delta_rad → foc_motor_t 顶层 (P3.1已处理)
angle_hold_integral_limit  → foc_motor_t 顶层 (P3.1已处理)
angle_hold_pid_deadband_rad → foc_motor_t 顶层 (P3.1已处理)
speed_angle_transition_start/end_rad → foc_motor_t 顶层 (P3.1已处理)

原 foc_control_runtime_config_t → 消除，5 个字段升为 foc_motor_t 顶层
```

#### P3.2 status 结构体精简

```c
// 原
typedef struct {
    uint8_t enabled;           // ← 由协议直接写，不变
    uint8_t configured_mode;   // ← 由协议直接写，不变
    uint8_t active_mode;       // ← 保留
    float blend_factor;        // ← 保留
    float auto_open_iq_a;      // ← 保留（协议直接写）
    float auto_closed_iq_a;    // ← 保留（协议直接写）
} foc_current_soft_switch_status_t;
// → 不变！因为这些都是运行时状态，不是 cfg 副本

// 原
typedef struct {
    uint8_t enabled;           // 保留，协议直接写
    uint8_t available;         // 保留
    uint8_t source;            // 保留
    uint16_t point_count;      // 保留
    float iq_lsb_a;            // 保留
    float iq_limit_a;          // 保留，协议直接写
    float speed_gate_rad_s;    // 保留，协议直接写
    float calib_gain_k;        // 保留，协议直接写
} foc_cogging_comp_status_t;
// → 不变！这些字段都被控制算法实时读取，本就是"单事实源"
```

**重新评估结论**：检查代码后发现，`foc_current_soft_switch_status_t` 和 `foc_cogging_comp_status_t` 的字段本来就是"运行时状态"——enabled 是在运行时可能被修改的（如齿槽标定时临时禁用软切换），不是从 cfg 同步的副本。**真正冗余的是 `foc_motor_cfg_t` 和 `foc_control_runtime_config_t`**。

所以 P3 实际消除的是：
- `foc_motor_cfg_t`（整个结构体）
- `foc_control_runtime_config_t`（整个结构体）
- 6 个 PID 配置宏字段直接写入 PID 对象

#### P3.3 默认值处理（注意点）

`foc_cfg_init_values.h` 中存在条件定义的默认值宏：

```c
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
#define COMMAND_MANAGER_DEFAULT_CONTROL_MODE COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE
#elif ...
```

这种条件宏如果未被包含或条件未命中则未定义。`FOC_ControlConfigResetDefault` 中使用这些宏赋值时可能出现**使用未定义宏**的编译错误。

**解决方案**：在 `FOC_ControlConfigResetDefault`（或 `FOC_MotorInit`）中改为：
```c
#if defined(COMMAND_MANAGER_DEFAULT_CONTROL_MODE)
    motor->control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
#else
    motor->control_mode = COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE;  // 安全默认值
#endif
```
不过更好的做法是确保编译配置完整性（在 `foc_compile_limits.h` 中检查宏是否已定义）。

#### P3.4 副作用执行器

消除 cfg 后，`FOC_Control_ApplyConfig` 改为：

```c
void FOC_Control_ApplyConfig(foc_motor_t *motor)
{
    float phase_res = (fabsf(motor->phase_resistance) > 1e-6f) 
                      ? fabsf(motor->phase_resistance) : 1e-6f;
    float i_max = motor->set_voltage / phase_res;
    
    // PID 限幅重新计算（依赖 set_voltage 和 phase_resistance）
    motor->torque_current_pid.out_min = -motor->set_voltage;
    motor->torque_current_pid.out_max = motor->set_voltage;
    motor->speed_pid.out_min = -i_max;
    motor->speed_pid.out_max = i_max;
    motor->angle_pid.out_min = -i_max;
    motor->angle_pid.out_max = i_max;
    
    // ADC 采样偏移
    Sensor_ADCSampleTimeOffset(motor->sensor_sample_offset_percent);
}
```

---

### Phase 4：输出链路统一 + 队列化

**目标**：实现双通道输出架构，直写通道保障即时反馈，队列通道承载批量数据。

**理由**：
- 当前 Monitor 任务和服务任务在定时器上下文中直接阻塞发送，影响控制时序
- 不同性质的输出混杂（故障诊断、示波器帧、参数回报），无法做优先级调度
- NEXT_MISSION P1 要求："主循环只消费队列并发送，避免直接读电机状态造成频率抖动"

#### 4.1 输出链路详细设计

```
                          L2 Control                    L2 Protocol               L2 Runtime
                     ┌─────────────────┐       ┌────────────────────┐     ┌──────────────────────┐
                     │ 故障诊断文本     │       │ 协议状态字节(status)│     │ 示波器帧              │
                     │ 标定过程日志     │       │ 命令响应文本(Y:R)  │     │ 语义遥测              │
                     │ 初始化日志       │       │ 参数回报(P:X/S:X)  │     │                      │
                     └────────┬────────┘       └────────┬───────────┘     └──────────┬───────────┘
                              │                         │                           │
                    ┌─────────▼─────────────────────────▼───────────────────────────▼──────────┐
                    │                         L1 输出管理器                                │
                    │                    FOC_OutputMgr_WriteText()                         │
                    └─────────────────────────┬─────────────────────────────────────────────┘
                                              │
                                 判断输出类型(调用时指定)
                                              │
                    ┌─────────────────────────┼─────────────────────────────┐
                    │                         │                             │
                    ▼                         ▼                             ▼
           ┌──────────────────┐    ┌────────────────────┐         ┌──────────────────┐
           │ 直写通道          │    │ 队列通道            │         │ 状态字节通道      │
           │ (立即输出)        │    │ (环形缓冲区)         │         │ (立即输出)        │
           ├──────────────────┤    ├────────────────────┤         ├──────────────────┤
           │ 故障文本         │    │ 示波器帧            │         │ 协议状态码        │
           │ 初始化日志       │    │ 参数回报文本         │         │ (单字节)          │
           │ 标定进度         │    │ Y:R 运行时摘要       │         │                  │
           │ 齿槽 Dump/Export │    │ OutputDiag 诊断文本   │         │                  │
           └────────┬─────────┘    └──────────┬─────────┘         └────────┬─────────┘
                    │                         │                           │
                    └─────────────────────────┼───────────────────────────┘
                                              │
                    ┌─────────────────────────▼──────────────────────────────┐
                    │             物理端口 (UART)                            │
                    │  直写通道优先级最高，可打断队列发送                       │
                    │  队列通道发送时，每条消息为一个原子单元                    │
                    └────────────────────────────────────────────────────────┘
```

#### 4.2 队列原子性保护

队列发送和直写共用同一 UART 端口，需要互斥机制：

```
队列通道发送流程:
  1. 获取锁 (cli / 关中断)
  2. 从队列取出一条消息
  3. 释放锁 (sei / 开中断)    ← 此时直写通道可插入
  4. 发送消息 (可能耗时)
  5. 循环直到达到 max_per_cycle 或队列空

直写通道发送流程:
  1. 直接写 UART 硬件
  (可在中断上下文中执行，队列发送必须在主循环)
```

**要点**：
- 队列发送**获取锁只用于取出消息**，发送本身不加锁（否则会阻塞直写通道）
- 直写通道在 ISR 中执行，通过硬件级的 UART 发送中断优先级保证
- 队列发送在主循环中执行，不关中断发送，自然可以被直写通道打断

#### 4.3 队列配置

```c
// foc/include/L1_Orchestration/foc_output.h

#define FOC_OUTPUT_QUEUE_DEPTH      8     // 队列深度，默认 8
#define FOC_OUTPUT_MAX_PER_CYCLE    4     // 主循环每次最大发送量

typedef struct {
    char buffer[FOC_OUTPUT_QUEUE_DEPTH][96];  // 每帧最大 96 字节
    uint8_t write_idx;
    uint8_t read_idx;
    uint8_t count;
    uint8_t overflow_count;                   // 满队列丢帧计数
} foc_output_queue_t;

void   FOC_OutputMgr_Init(foc_system_t *sys);
void   FOC_OutputMgr_WriteDirect(foc_system_t *sys, const char *text);  // 直写
void   FOC_OutputMgr_WriteQueue(foc_system_t *sys, const char *text);   // 入队
void   FOC_OutputMgr_WriteStatus(foc_system_t *sys, uint8_t status);    // 直写状态字节
void   FOC_OutputMgr_FlushQueue(foc_system_t *sys);                     // 主循环消费
uint8_t FOC_OutputMgr_GetOverflowCount(const foc_system_t *sys);        // 诊断
```

#### 4.4 各调用点输出通道分配

| 文件位置 | 调用内容 | 输出函数 | 通道 | 优先级 | 说明 |
|---------|---------|---------|------|-------|------|
| **L2/Control** | | | | | |
| `foc_ctrl_executor.c:144` | "sensor invalid threshold reached" | `WriteDirect` | 直写 | 高 | 故障诊断需即时上报 |
| `foc_ctrl_init.c:72` | "init.calib: zero-lock sampling failed" | `WriteDirect` | 直写 | 中 | 初始化一次性日志 |
| `foc_ctrl_init.c:104` | "init.calib: direction/pole-pairs estimation failed" | `WriteDirect` | 直写 | 中 | 同上 |
| `foc_ctrl_init.c:250` | 齿槽静态表装载状态 | `WriteDirect` | 直写 | 中 | 初始化状态 |
| `foc_ctrl_compensation.c:257` | "COGGING CALIB START..." | `WriteDirect` | 直写 | 中 | 长操作过程日志 |
| `foc_ctrl_compensation.c` | 标定过程百分比日志 | `WriteDirect` | 直写 | 低 | 可被更高优先级打断 |
| `foc_ctrl_compensation.c:806` | LUT Dump（纯数字列表，512行） | `WriteDirect` | 直写 | 低 | 数据量大但不适合入队 |
| `foc_ctrl_compensation.c:843` | LUT Export（C代码） | `WriteDirect` | 直写 | 低 | 同上 |
| **L2/Protocol** | | | | | |
| `foc_protocol_handler.c:678` | Y:R 运行时摘要文本 | `WriteQueue` | 队列 | 中 | 文本较大，可延迟 |
| `foc_protocol_handler.c` | 状态码 'O'/'E'/'P'/'I'/'T' | `WriteStatus` | 直写 | **最高** | **必须即时** |
| `foc_protocol_output.c:56` | OutputDiag 诊断文本 | `WriteQueue` | 队列 | 低 | 可延迟 |
| **L2/Runtime** | | | | | |
| `foc_debug_stream.c:137` | 示波器帧 | `WriteQueue` | 队列 | 中 | P1.0 核心改造 |
| `foc_debug_stream.c:282` | 语义遥测 | `WriteQueue` | 队列 | 低 | 可延迟 |
| **L1** | | | | | |
| `foc_app.c:137` | 启动信息 | `WriteDirect` | 直写 | 低 | 一次性 |
| `foc_service_handler.c:41` | "=== ReInitMotor started ===" | `WriteDirect` | 直写 | 中 | 初始化日志 |
| `foc_service_handler.c:65` | reinit 完成信息 | `WriteDirect` | 直写 | 中 | 同上 |
| `foc_service_handler.c:136` | "init: all checks passed" | `WriteDirect` | 直写 | 中 | 同上 |

---

## 四、互斥与风险分析

### 4.1 直写/队列互斥细化

```
主循环发送队列时：

  while (sent_count < FOC_OUTPUT_MAX_PER_CYCLE)
  {
      // Step 1: 取出消息（关中断保护临界区）
      EnterCritical();
      if (queue->count == 0)
      {
          ExitCritical();
          break;
      }
      char *msg = queue->buffer[queue->read_idx];
      queue->read_idx = (queue->read_idx + 1) % FOC_OUTPUT_QUEUE_DEPTH;
      queue->count--;
      ExitCritical();
      
      // Step 2: 发送消息（开中断，此时直写可打断）
      FOC_Platform_WriteDebugText(msg);
      sent_count++;
  }

直写通道（可在 ISR 上下文中调用）：

  void FOC_OutputMgr_WriteDirect(foc_system_t *sys, const char *text)
  {
      // 无需锁，因为 UART 发送缓冲区由硬件/FIFO 管理
      FOC_Platform_WriteDebugText(text);
  }
```

**风险**：
1. 直写通道在 ISR 中调用时，如果 UART 发送未完成就写入新数据，可能导致 UART 缓冲区溢出
2. 队列通道发送长消息时被直写打断，直写消息可能嵌入队列消息中间

**解决方案**：
1. 队列通道发送每条消息为一次完整的 `FOC_Platform_WriteDebugText` 调用（内部保证原子发送）
2. 直写通道的消息也是独立的 `WriteDebugText` 调用
3. 物理层（UART 驱动）应保证单次 `WriteDebugText` 调用不会被其他调用混合 — 这依赖于 L5 实现。如果 L5 的 `FOC_Platform_WriteDebugText` 不保证原子性，则需要在 L1 增加发送锁

### 4.2 时序影响

| 场景 | 当前 | 改造后 |
|------|------|--------|
| 示波器帧发送（100Hz, ~80字节） | 在 Monitor 回调中阻塞发送，影响下次回调 | Monitor 回调仅入队（~1μs），主循环发送 |
| 参数批量回报（P:X, ~30条） | 在 Service 回调中逐条发送，占用服务时间 | 入队后主循环分多次发送 |
| 状态字节响应 | 直接写 UART，即时 | 保持即时（直写通道） |
| 齿槽标定 Dump（512数字） | 一次性输出大量文本 | 保持直写，因为不可预期延迟无意义 |

---

## 五、构建与验证

### 5.1 影响文件统计

| Phase | 新增 | 修改 | 删除 | 主要风险 |
|-------|------|------|------|---------|
| P0 | 6 | ~18 | 6 | 大量 #include 路径更新，遗漏会导致编译失败 |
| P1 | 2 | ~8 | 0 | API 签名变更，需要全库同步 |
| P2 | 0 | ~10 | 0 | SVPWM API 签名变更 + 所有调用者更新 |
| P3 | 0 | ~10 | 0 | 控制算法读配置路径变更，高影响 |
| P4 | 2 | ~10 | 0 | 输出行为变化，调试时可能漏输 |

### 5.2 回退策略

每个 Phase 完成后需要：
1. `git add` 所有变更文件
2. `git commit -m "Phase N: ..."`
3. 构建验证 0 error
4. 若下一个 Phase 失败，可通过 `git checkout` 回退到上一个 commit

### 5.3 构建命令

```powershell
.\tools\build_gd32f303.ps1
```

验收标准：**0 error，不新增 warning**。