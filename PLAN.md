# 架构重构执行计划

## 目标架构

### 核心思想

消除冗余的 C1→C2→C3→C4→C5 运行时管线，将运行时状态和控制配置并入 `foc_motor_t`，恢复"L1 编排 + L2 协议/控制 独立块"的简单架构。

### 块间关系

```
主循环/Service Task (L1):
  FOC_Protocol_Process(&motor)   ← 协议块：读帧→解析→改motor字段(参数/状态)
  if (motor.state.cfg_dirty)
      FOC_Control_ApplyConfig(&motor)  ← 控制块：从motor读取参数应用到PID

控制任务/Control_MainLoop (L1):
  Sensor_ReadAll()
  if (!motor.state.system_fault && motor.state.motor_enabled)
      FOC_Control_Run(&motor, &sensor, dt)

PWM ISR/电流环 (L1 -> Control):
  FOC_Control_CurrentLoop(&motor, &current_pid, &sensor, angle, dt)
```

### 数据流

```
                         ┌─────────────────────┐
                         │    foc_motor_t        │
                         │  (包含所有电机参数、   │
                         │   PID对象、运行状态、   │
                         │   控制配置、运行时状态)  │
                         └──────────┬───────────┘
                               ▲    │
             协议块写入/读取    │    │   控制算法读写
                         ┌────┴────┴────┐
                         │   L1 编排     │
                         │  (foc_app.c)  │
                         └──────────────┘
```

---

## 阶段1 — 数据结构重构

### 1a. 扩展 `foc_motor_t`

将以下内容并入 `foc_motor_t`：

**运行时状态**（原 `runtime_state_snapshot_t` → 放入 motor）
```c
    /* === 运行时状态（per-motor） === */
    struct {
        uint8_t system_running;       /* 1=正常运行,0=非运行 */
        uint8_t system_fault;         /* 1=故障状态 */
        uint8_t reinit_pending;       /* 1=重初始化请求 */
        uint8_t last_fault_code;      /* 故障码 */
        uint8_t cfg_dirty;            /* 1=配置已变更，需要L1应用 */
        uint8_t motor_enabled;        /* 1=电机使能（原control_cfg.motor_enabled） */
        uint8_t control_mode;         /* 控制模式 */
        uint8_t pending_system_action;/* 待处理的系统命令（由协议设置，L1消费） */
        uint16_t init_check_mask;     /* 已执行的初始化检查掩码 */
        uint16_t init_fail_mask;      /* 失败的初始化检查掩码 */
        uint16_t sensor_invalid_consecutive; /* 连续无效传感器计数 */
        uint32_t protocol_error_count;
        uint32_t param_error_count;
        uint32_t control_skip_count;
    } state;
```

**控制配置**（原 `control_config_snapshot_t` → 直接放入 motor）
```c
    /* === 控制配置参数 === */
    struct {
        float target_angle_rad;
        float angle_position_speed_rad_s;
        float speed_only_rad_s;
        float sensor_sample_offset_percent;
        // fine-tuning
        float min_mech_angle_accum_delta_rad;
        float angle_hold_integral_limit;
        float angle_hold_pid_deadband_rad;
        float speed_angle_transition_start_rad;
        float speed_angle_transition_end_rad;
        // 软切换
        uint8_t current_soft_switch_enable;
        uint8_t current_soft_switch_mode;
        float current_soft_switch_auto_open_iq_a;
        float current_soft_switch_auto_closed_iq_a;
        // 齿槽补偿
        uint8_t cogging_comp_enable;
        float cogging_comp_iq_limit_a;
        float cogging_comp_speed_gate_rad_s;
        float cogging_calib_gain_k;
    } cfg;
```

**PID 对象**（原独立全局变量 → 放入 motor）
```c
    /* === PID 控制器对象 === */
    foc_pid_t torque_current_pid;
    foc_pid_t speed_pid;
    foc_pid_t angle_pid;
```

**已有字段全部保留不变**（electrical_phase_angle, ud/uq, iq_target, cogging相关等）。

### 1b. 精简 `foc_snapshot_types.h`

删除 `runtime_c4_params_view_t`、`runtime_c4_states_view_t`、`runtime_c4_runtime_view_t`（不再需要内部存储+外部快照的双重结构）。

保留 `telemetry_policy_snapshot_t` 作为系统级配置（非 per-motor）。

`runtime_snapshot_t` 和 `control_config_snapshot_t` 可删除，因为不再需要快照传递机制。

保留 `runtime_step_signal_t`、`runtime_init_check_t`、`runtime_fault_code_t`（但可能改名，放入 `foc_motor_types.h`）。

### 1c. 删除/整合冗余类型文件

- `foc_runtime_types.h` → 保留但精简（`runtime_step_signal_t` 等仍需要），或者把类型定义移入 `foc_motor_types.h` 后删除此文件
- `foc_snapshot_types.h` → 大幅精简，只保留 telemetry 相关
- `foc_runtime_snapshot.h` → 删除
- `foc_shared_types.h` → 检查是否还有引用

### 1d. 新增类型 `foc_motor_state_t` 和 `foc_motor_cfg_t`

或者直接在 `foc_motor_t` 中以嵌套结构体形式嵌入（如上所示）。

---

## 阶段2 — 协议块重构

### 2a. 新建文件

| 文件 | 内容 |
|------|------|
| `foc/src/L2/Protocol/foc_protocol_handler.c` | 协议主入口：帧读取+解析+派发+存储 |
| `foc/include/L2/Protocol/foc_protocol_handler.h` | 上述的公开接口 |
| `foc/src/L2/Protocol/foc_protocol_output.c` | 输出适配（原 `foc_runtime_output.c`） |
| `foc/include/L2/Protocol/foc_protocol_output.h` | 上述的公开接口 |

### 2b. 对外接口

```c
/* foc_protocol_handler.h */

/* 处理一帧通讯（从通讯源读取帧→解析→执行→写入motor）
 * 返回 1=有通讯活动，0=无帧
 */
uint8_t FOC_Protocol_Process(foc_motor_t *motor, uint8_t frame_budget);

/* 初始化协议内部存储 */
void FOC_Protocol_Init(void);

/* 清除配置脏标志（L1应用配置后调用） */
void FOC_Protocol_Commit(foc_motor_t *motor);
```

### 2c. 内部工作

- 吸收原 `foc_runtime_protocol.c` 的 FrameSource_* 帧读取逻辑
- 吸收原 `foc_runtime_fsm.c` 的 `RuntimeC3_HandleCommand`（现在改名为 `FOC_Protocol_HandleCommand`，直接操作 `motor` 结构体）
- 吸收原 `foc_runtime_store.c` 参数读写命令执行
- 吸收原 `foc_runtime_output.c` 输出适配

**关键变化**：不再有 C4 内部 static 的 params/states/runtime 三件套。参数存储直接写入 `motor->cfg` 和 `motor->state`。状态读写直接操作 `motor` 字段。

### 2d. 删除文件

| 删除文件 | 原因 |
|----------|------|
| `foc/src/L2/Runtime/foc_runtime_entry.c` | 功能并入 L1 和 Protocol |
| `foc/include/L2/Runtime/foc_runtime_entry.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_protocol.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_protocol.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_fsm.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_fsm.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_store.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_store.h` | 同上 |

### 2e. 改名移入的文件

| 旧文件 | 新文件 |
|--------|--------|
| `foc/src/L2/Runtime/foc_runtime_output.c` | `foc/src/L2/Protocol/foc_protocol_output.c` |
| `foc/include/L2/Runtime/foc_runtime_output.h` | `foc/include/L2/Protocol/foc_protocol_output.h` |

### 2f. 保留的文件（仍在 Runtime 目录）

- `foc/src/L2/Runtime/foc_debug_stream.c` — 调试流
- `foc/include/L2/Runtime/foc_debug_stream.h`
- `foc/src/L2/Runtime/foc_task_scheduler.c` — 调度器
- `foc/include/L2/Runtime/foc_task_scheduler.h`

---

## 阶段3 — 控制块简化

### 3a. 简化的对外接口

```c
/* foc_ctrl_entry.h */

/* 运行一帧控制外环（速度/角度模式）。内部读motor->cfg.control_mode */
void FOC_Control_Run(foc_motor_t *motor, const sensor_data_t *sensor, float dt_sec);

/* 电流环步进（在PWM ISR中调用） */
void FOC_Control_CurrentLoop(foc_motor_t *motor, const sensor_data_t *sensor, 
                             float electrical_angle, float dt_sec);

/* 应用配置：从motor->cfg读取PID参数和fine-tuning设置，重算PID限幅 */
void FOC_Control_ApplyConfig(foc_motor_t *motor);

/* 初始化控制算法状态 */
void FOC_Control_Init(foc_motor_t *motor);

/* 开环步进 */
void FOC_Control_OpenLoopStep(foc_motor_t *motor, float voltage, float turn_speed);

/* 齿槽补偿标定 */
uint8_t FOC_Control_CoggingCalibIsBusy(const foc_motor_t *motor);
uint8_t FOC_Control_CoggingCalibSampleStep(foc_motor_t *motor, const sensor_data_t *sensor, float dt_sec);
void FOC_Control_CoggingCalibRequestStart(foc_motor_t *motor);
void FOC_Control_CoggingCalibDumpTable(const foc_motor_t *motor);
void FOC_Control_CoggingCalibExportTable(const foc_motor_t *motor);
```

### 3b. 函数签名变化示例

| 旧函数 | 新函数 |
|--------|--------|
| `FOC_ControlOuterLoopStep(motor, current_pid, speed_pid, angle_pid, sensor, mode, ...)` | `FOC_Control_Run(motor, sensor, dt)` — 内部使用 `motor->torque_current_pid` 等 |
| `FOC_ControlCurrentLoopStep(motor, current_pid, sensor, angle, dt)` | `FOC_Control_CurrentLoop(motor, sensor, angle, dt)` — 内部使用 motor->torque_current_pid |
| 无统一 Apply 函数 | `FOC_Control_ApplyConfig(motor)` — 封装原 foc_app.c 中的 PID init helper |

### 3c. 删除或重命名的内部函数

- `foc_ctrl_entry.c` 中的 `FOC_ControlCurrentLoopRequiresSample` → 内联到调用点或改名
- `FOC_ControlCompensationStep` → 内联到 `FOC_Control_Run` 内部
- `FOC_ControlOpenLoopStep` → 保留但签名不变（已经接受 motor 指针）

### 3d. `foc_ctrl_cfg.c/h` 接口精简

当前的 setter 函数（`FOC_ControlSetMinMechAngleAccumDeltaRad` 等）是协议写入时调用的。新架构中协议直接写 `motor->cfg`，但 setter 可以保留用于额外校验：

```c
// 建议保留但简化：
void FOC_Control_ApplyCfgMinMechAngleAccumDeltaRad(foc_motor_t *motor, float value);
// 或者直接通过默认宏初始化，协议块直接写 motor->cfg 字段
```

考虑：协议块直接写 `motor->cfg.min_mech_angle_accum_delta_rad = new_value` 设置 `motor->state.cfg_dirty = 1`，由 L1 在检测到 dirty 后调用 `FOC_Control_ApplyConfig(motor)` 批量应用所有配置。这样协议块不需要知道任何 setter 函数。

---

## 阶段4 — L1 编排重构（foc_app.c）

### 4a. 全局变量变化

**删除的全局变量：**
```c
static runtime_snapshot_t g_StateSnapshot;           // 不再需要
static foc_pid_t g_torque_current_pid;               // 移入 g_motor
static foc_pid_t g_angle_pid;                        // 移入 g_motor
static foc_pid_t g_speed_pid;                        // 移入 g_motor
```

**保留的全局变量：**
```c
static foc_motor_t g_motor;                          // 扩展后包含所有
static sensor_data_t g_sensor_snapshot;              // 传感器快照
static sensor_data_t g_fast_current_sensor_snapshot; // 电流环快照

// 电流环快速路径变量（保持独立）
static volatile uint8_t g_fast_current_loop_enabled;
static volatile uint8_t g_fast_current_loop_div_counter;
static volatile float g_fast_current_loop_iq_target;
static volatile float g_fast_current_loop_electrical_angle;

// 系统级
static volatile uint8_t g_service_task_pending;
static volatile uint8_t g_monitor_task_pending;
static volatile uint8_t g_reinit_in_progress;
static uint16_t g_led_comm_pulse_counter;
```

### 4b. Init 流程变化

```c
void FOC_App_Init(void)
{
    FOC_Platform_RuntimeInit();
    FOC_Platform_IndicatorInit();
    // LED 闪烁表示启动中...

    // 初始化调度器
    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();
    FOC_Platform_SetControlTickCallback(ControlScheduler_RunTick);
    ControlScheduler_SetCallback(FOC_TASK_RATE_SERVICE, Service_Task_Trigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, Motor_Control_Loop);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, Monitor_Task_Trigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    // 通讯初始化
    FOC_Platform_CommInit();

    // 协议初始化
    FOC_Protocol_Init();

    // 控制初始化
    FOC_Control_Init(&g_motor);

    // 传感器初始化
    Sensor_Init(...);
    Sensor_ReadAll();
    Sensor_CopyData(&g_sensor_snapshot);

    // PWM 初始化
    SVPWM_Init(...);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);

    // 初始化电机（含PID）
    FOC_MotorInit(&g_motor, ...);

    // 设置 init_check 等状态到 motor.state
    // ...

    FOC_Platform_WriteDebugText("\r\n=== FOC System Started ===\r\n");
    FOC_App_UpdateIndicators();
}
```

### 4c. Service Task 变化

```c
static void Service_Task_Trigger(void)
{
    FOC_App_UpdateIndicators();
    g_service_task_pending = 1U;
}

// 在主循环中:
if (g_service_task_pending)
{
    g_service_task_pending = 0U;

    // 重初始化处理
    if (g_motor.state.reinit_pending)
        FOC_App_ReInitMotor();

    // 协议处理：接收帧→解析→修改 motor
    if (FOC_Protocol_Process(&g_motor, FOC_APP_COMM_FRAMES_PER_STEP))
        g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;

    // 配置脏检查：如果协议修改了参数
    if (g_motor.state.cfg_dirty)
    {
        FOC_Control_ApplyConfig(&g_motor);
        Sensor_ADCSampleTimeOffset(g_motor.cfg.sensor_sample_offset_percent);
        FOC_Protocol_Commit(&g_motor);  // 清除 dirty
    }

    // 系统命令处理（Y通道：齿槽标定）
    if (g_motor.state.pending_system_action != FOC_SYSACTION_NONE)
    {
        switch (g_motor.state.pending_system_action)
        {
        case FOC_SYSACTION_COGGING_START:
            FOC_Control_CoggingCalibRequestStart(&g_motor);
            break;
        case FOC_SYSACTION_COGGING_DUMP:
            g_motor.cogging_calib_state.request_dump = 1U;
            break;
        case FOC_SYSACTION_COGGING_EXPORT:
            g_motor.cogging_calib_state.request_export = 1U;
            break;
        }
        g_motor.state.pending_system_action = FOC_SYSACTION_NONE;
    }

    // 齿槽标定数据导出
    if (g_motor.cogging_calib_state.request_dump) { ... }
    if (g_motor.cogging_calib_state.request_export) { ... }
}
```

### 4d. 控制主循环变化

```c
static void Motor_Control_Loop(void)
{
    if (g_reinit_in_progress) return;

    if (g_motor.state.system_fault)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    Sensor_ReadAll();
    Sensor_CopyData(&g_sensor_snapshot);

    // 传感器有效性检查 → 更新故障状态
    if (!g_sensor_snapshot.adc_valid || !g_sensor_snapshot.encoder_valid)
    {
        // ...更新 motor.state.fault_code 等
    }

    if (!g_motor.state.motor_enabled)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    // 齿槽标定模式
    if (FOC_Control_CoggingCalibIsBusy(&g_motor))
    {
        // ...
    }
    else
    {
        // 正常控制
        FOC_Control_Run(&g_motor, &g_sensor_snapshot, FOC_CONTROL_DT_SEC);
    }

    // 更新电流环快速路径变量
    g_fast_current_loop_iq_target = g_motor.iq_target;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_enabled = 1U;
}
```

### 4e. PWM ISR 变化

```c
static void FOC_App_OnPwmUpdateISR(void)
{
    // ... 同前
    FOC_Control_CurrentLoop(&g_motor, &g_fast_current_sensor_snapshot,
                            g_fast_current_loop_electrical_angle,
                            current_loop_dt_sec);
}
```

### 4f. 简化后的 ReInit / EnterSafeOutput

`FOC_App_ReInitMotor` → 移除 `Runtime_ClearReinit()` 调用，改为 `g_motor.state.reinit_pending = 0U;`

`FOC_App_EnterSafeOutputState` → 移除 `Runtime_UpdateSignals` 调用，改为直接设置 `motor` 字段。

---

## 阶段5 — 文件名/API 重命名清单

### 5a. 新增文件

| 文件 | 说明 |
|------|------|
| `foc/src/L2/Protocol/foc_protocol_handler.c` | 协议主入口 |
| `foc/include/L2/Protocol/foc_protocol_handler.h` | 协议主入口接口 |
| `foc/src/L2/Protocol/foc_protocol_output.c` | 输出适配（从 Runtime 改名移入） |
| `foc/include/L2/Protocol/foc_protocol_output.h` | 输出适配接口 |

### 5b. 删除文件

| 文件 | 说明 |
|------|------|
| `foc/src/L2/Runtime/foc_runtime_entry.c` | 功能并入 L1 + Protocol |
| `foc/include/L2/Runtime/foc_runtime_entry.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_protocol.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_protocol.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_fsm.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_fsm.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_store.c` | 功能并入 protocol_handler |
| `foc/include/L2/Runtime/foc_runtime_store.h` | 同上 |
| `foc/src/L2/Runtime/foc_runtime_output.c` | 改名移入 Protocol |
| `foc/include/L2/Runtime/foc_runtime_output.h` | 改名移入 Protocol |
| `foc/include/L2/Runtime/foc_runtime_snapshot.h` | 冗余间接头文件 |

### 5c. 保留的文件

| 文件 | 说明 |
|------|------|
| `foc/src/L2/Runtime/foc_debug_stream.c` | 调试流（保留） |
| `foc/include/L2/Runtime/foc_debug_stream.h` | 同上 |
| `foc/src/L2/Runtime/foc_task_scheduler.c` | 调度器（保留） |
| `foc/include/L2/Runtime/foc_task_scheduler.h` | 同上 |

### 5d. 需要大改的文件

| 文件 | 改动 |
|------|------|
| `foc/include/LS_Config/foc_motor_types.h` | 大幅扩展：加入 state/cfg/PID |
| `foc/include/LS_Config/foc_snapshot_types.h` | 大幅精简：只保留 telemetry |
| `foc/src/L1_Orchestration/foc_app.c` | 彻底重写（删除快照，改用新接口） |
| `foc/include/L1_Orchestration/foc_app.h` | 接口精简（可能） |
| `foc/src/L2/Control/foc_ctrl_entry.c` | 简化接口 + 使用 motor->pid |
| `foc/include/L2/Control/foc_ctrl_entry.h` | 接口重命名 |
| `foc/src/L2/Control/foc_ctrl_cfg.c` | 可能需要适配新结构体路径 |
| `foc/include/L2/Control/foc_ctrl_cfg.h` | 可能需要适配 |
| `foc/src/L2/Control/foc_ctrl_init.c` | 使用 motor->pid 初始化 |
| `foc/include/L2/Control/foc_ctrl_init.h` | 可能不变 |

### 5e. 需要小幅修改的文件

| 文件 | 改动 |
|------|------|
| `foc/include/L2/Runtime/foc_debug_stream.h` | 修改函数签名（去掉 runtime 参数改为 motor） |
| `foc/src/L2/Runtime/foc_debug_stream.c` | 同上 |
| `examples/GD32F303_FOCExplore/.../builder.params` | 更新 sourceList（删除旧 runtime，新增 protocol_handler） |
| `examples/GD32F303_FOCExplore/.../foc_platform_api.c` | 如果引用了被删类型 |
| `docs/architecture.md` | 同步架构描述 |
| `CHANGELOG.md` | 记录变更 |

---

## 阶段6 — 具体函数重命名映射

### 6a. Runtime → Protocol 函数

| 旧函数 | 新函数 | 位置 |
|--------|--------|------|
| `Runtime_Init()` | `FOC_Protocol_Init()` | L1 调用点 |
| `Runtime_FrameRunStep(frame_budget)` | `FOC_Protocol_Process(motor, frame_budget)` | L1 调用点 |
| `Runtime_UpdateSignals(signals)` | **删除**（直接操作 motor.state） | |
| `Runtime_GetSnapshot(snapshot)` | **删除**（直接读 motor 字段） | |
| `Runtime_Commit()` | `FOC_Protocol_Commit(motor)` | L1 调用点 |
| `Runtime_ClearReinit()` | 改为 `motor.state.reinit_pending = 0U` | L1 调用点 |
| `RuntimeC2_ProcessOneFrame()` | **内联**到 `FOC_Protocol_Process` | Protocol 内部 |
| `RuntimeC2_Init()` | **内联**到 `FOC_Protocol_Init` | Protocol 内部 |
| `RuntimeC2_UpdateSignals(signal)` | **删除** | |
| `RuntimeC2_BuildSnapshot(snapshot)` | **删除** | |
| `RuntimeC2_Commit()` | **内联** | |
| `RuntimeC2_ClearReinit()` | **内联** | |
| `RuntimeC3_UpdateSignals(signal)` | **全局改为 motor 直接赋值** | Protocol 内部 |
| `RuntimeC3_HandleCommand(cmd)` | `FOC_Protocol_HandleCommand(motor, cmd)` | Protocol 内部 |
| `RuntimeC3_Init()` | **内联**到 `FOC_Protocol_Init` | |
| `RuntimeC3_ReportFrameError()` | **内联**到 handler | |
| `RuntimeC3_BuildSnapshot(snapshot)` | **删除** | |
| `RuntimeC3_Commit()` | **内联** | |
| `RuntimeC3_ClearReinit()` | **内联** | |
| `RuntimeC4_*` / `RuntimeC4Store_*` | 全部变为 Protocol 内部 static 函数 | Protocol 内部 |
| `RuntimeC5_*` | 全部变为协议输出工具 `FOC_Protocol_Output*` | |

### 6b. Control 函数

| 旧函数 | 新函数 | 说明 |
|--------|--------|------|
| `FOC_ControlOuterLoopStep(motor, current_pid, speed_pid, angle_pid, sensor, ...)` | `FOC_Control_Run(motor, sensor, dt)` | 内部使用 motor->pid |
| `FOC_ControlCurrentLoopStep(motor, current_pid, sensor, angle, dt)` | `FOC_Control_CurrentLoop(motor, sensor, angle, dt)` | 同上 |
| `FOC_ControlCompensationStep(motor, sensor)` | **内联**到 `FOC_Control_Run` | |
| `FOC_ControlOpenLoopStep(motor, volt, speed)` | 保留（签名不变） | |
| — (不存在) | `FOC_Control_ApplyConfig(motor)` | 封装原有的 PID init 和 fine-tuning 应用 |
| — (不存在) | `FOC_Control_Init(motor)` | 控制算法初始化 |
| `FOC_ControlCoggingCalibIsBusy(motor)` | `FOC_Control_CoggingCalibIsBusy(motor)` | 改命名风格 |
| `FOC_ControlCoggingCalibSampleStep(...)` | `FOC_Control_CoggingCalibSampleStep(...)` | 改命名风格 |
| `FOC_ControlCoggingCalibRequestStart(...)` | `FOC_Control_CoggingCalibRequestStart(...)` | 改命名风格 |
| `FOC_ControlCoggingCalibDumpTable(...)` | `FOC_Control_CoggingCalibDumpTable(...)` | 改命名风格 |
| `FOC_ControlCoggingCalibExportTable(...)` | `FOC_Control_CoggingCalibExportTable(...)` | 改命名风格 |
| `FOC_ControlCurrentLoopRequiresSample()` | 保留或内联 | |

### 6c. 输出工具函数

| 旧函数 | 新函数 |
|--------|--------|
| `RuntimeC5_WriteText(text)` | `FOC_Protocol_WriteText(text)` |
| `RuntimeC5_WriteStatusByte(status)` | `FOC_Protocol_WriteStatus(status)` |
| `RuntimeC5_GetFaultName(code)` | `FOC_Protocol_GetFaultName(code)` |
| `RuntimeC5_OutputDiag(level, module, detail)` | `FOC_Protocol_OutputDiag(level, module, detail)` |
| `RuntimeC5_OutputParam(subcmd, value)` | `FOC_Protocol_OutputParam(subcmd, value)` |
| `RuntimeC5_OutputState(subcmd, value)` | `FOC_Protocol_OutputState(subcmd, value)` |

---

## 阶段7 — 类型定义迁移

### 原 `foc_snapshot_types.h` 中的类型

| 原类型 | 处理 |
|--------|------|
| `control_config_snapshot_t` | **删除** — 改为 `foc_motor_t` 的 `.cfg` 嵌套结构体 |
| `telemetry_policy_snapshot_t` | **保留** — 作为系统级全局配置 |
| `runtime_state_snapshot_t` | **删除** — 改为 `foc_motor_t` 的 `.state` 嵌套结构体 |
| `runtime_snapshot_t` | **删除** — 不再需要 |
| `runtime_c4_exec_result_t` | **删除** — 内部枚举不对外暴露 |
| `RUNTIME_SYSACTION_*` | **改名为** `FOC_SYSACTION_*` |
| `runtime_c4_runtime_view_t` | **删除** |
| `runtime_c4_params_view_t` | **删除** |
| `runtime_c4_states_view_t` | **删除** |

### 原 `foc_runtime_types.h` 中的类型

| 原类型 | 处理 |
|--------|------|
| `runtime_step_signal_t` | **保留**（L1 可能仍需要它传递传感器状态到 init） |
| `runtime_init_check_t` | **保留**或移入 `foc_motor_types.h` |
| `runtime_fault_code_t` | **改名为** `foc_fault_code_t` |

### `foc_motor_types.h` 中新增的嵌套结构体

```c
/* === 运行时状态（per-motor） === */
typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t reinit_pending;
    uint8_t last_fault_code;
    uint8_t cfg_dirty;
    uint8_t motor_enabled;
    uint8_t control_mode;
    uint8_t pending_system_action;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
} foc_motor_state_t;

/* === 控制配置参数 === */
typedef struct {
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
    uint8_t current_soft_switch_enable;
    uint8_t current_soft_switch_mode;
    float current_soft_switch_auto_open_iq_a;
    float current_soft_switch_auto_closed_iq_a;
    uint8_t cogging_comp_enable;
    float cogging_comp_iq_limit_a;
    float cogging_comp_speed_gate_rad_s;
    float cogging_calib_gain_k;
} foc_motor_cfg_t;

/* === 系统动作枚举 === */
#define FOC_SYSACTION_NONE           0U
#define FOC_SYSACTION_COGGING_START  1U
#define FOC_SYSACTION_COGGING_DUMP   2U
#define FOC_SYSACTION_COGGING_EXPORT 3U
```

---

## 阶段8 — 构建配置更新

### builder.params changes

**删除的 source entry：**
```
"../../../foc/src/L2/Runtime/foc_runtime_entry.c"
"../../../foc/src/L2/Runtime/foc_runtime_fsm.c"
"../../../foc/src/L2/Runtime/foc_runtime_protocol.c"
"../../../foc/src/L2/Runtime/foc_runtime_store.c"
```

**改名的 source entry：**
```
"../../../foc/src/L2/Runtime/foc_runtime_output.c"
  → "../../../foc/src/L2/Protocol/foc_protocol_output.c"
```

**新增的 source entry：**
```
"../../../foc/src/L2/Protocol/foc_protocol_handler.c"
```

**保留的 Runtime source：**
```
"../../../foc/src/L2/Runtime/foc_debug_stream.c"
"../../../foc/src/L2/Runtime/foc_task_scheduler.c"
```

---

## 阶段9 — 回归验证清单

构建前检查：
- [ ] `foc_motor_types.h` 中新增的 `state` 和 `cfg` 嵌套结构体定义完整
- [ ] 所有对 `g_torque_current_pid`、`g_speed_pid`、`g_angle_pid` 的引用已改为 `g_motor.torque_current_pid` 等
- [ ] 所有对 `g_StateSnapshot.*` 的引用已删除或改为 `g_motor.*`
- [ ] 所有对 `Runtime_*` 函数的调用已改为新接口
- [ ] 所有 `#include` 不再引用被删除的头文件
- [ ] Protocol 块不包含任何控制算法头文件
- [ ] Control 块不包含任何协议/运行时头文件（除 `foc_motor_types.h` 等LS类型）
- [ ] `builder.params` sourceList 更新正确

构建验证：
- [ ] 0 error
- [ ] 无新增 warning
- [ ] `"undefined symbol"` 链接错误 = 0

---

## 实施步骤（切换为ACT模式后执行）

### Step A — 数据结构先行
- [ ] A1: 修改 `foc_motor_types.h`：加入 `foc_motor_state_t`、`foc_motor_cfg_t`、`foc_fault_code_t` 类型
- [ ] A2: 扩展 `foc_motor_t`：加入 `.state`、`.cfg`、三个 PID 对象字段
- [ ] A3: 精简 `foc_snapshot_types.h`：只保留 `telemetry_policy_snapshot_t`
- [ ] A4: 删除 `foc_runtime_snapshot.h`
- [ ] A5: 更新 `foc_runtime_types.h`：`runtime_fault_code_t` → 改为使用 `foc_fault_code_t`

### Step B — 创建 Protocol 块新文件
- [ ] B1: 创建 `foc_protocol_handler.c/.h`（吸收 Runtime 协议功能）
- [ ] B2: 创建 `foc_protocol_output.c/.h`（从 runtime_output 改名移入）

### Step C — 简化 Control 块
- [ ] C1: 修改 `foc_ctrl_entry.c/.h`：简化接口，内部使用 motor->pid
- [ ] C2: 修改 `foc_ctrl_cfg.c/.h`：适配新结构体路径
- [ ] C3: 修改 `foc_ctrl_init.c`：初始化 motor->pid

### Step D — 重写 L1 编排
- [ ] D1: 重写 `foc_app.c`：删除快照和 Runtime 调用，使用新接口

### Step E — 更新所有其他引用点
- [ ] E1: 更新 `foc_debug_stream.c/.h`（如引用旧类型）
- [ ] E2: 更新 `foc_platform_api.c`（如引用旧类型）
- [ ] E3: 更新 `builder.params`
- [ ] E4: 编译验证

### Step F — 文档同步
- [ ] F1: 更新 `docs/architecture.md`
- [ ] F2: 更新 `CHANGELOG.md`
- [ ] F3: 更新 `docs/protocol-parameters-bilingual.md`（如协议参数引用变化）
- [ ] F4: 更新此 PLAN.md 为完成状态

---

## 注意事项

1. **不中断功能**：所有对外行为（协议格式、控制行为）不变，只变内部组织
2. **不留兼容层**：旧函数名/类型名全部删除或改名，不保留 `#define` 兼容桥
3. **单步可编译**：尽量将大改拆分为多个独立提交，每步可编译，完成后继续连续进行下一步而不是停下来抛一个假的任务完成。
4. **文档同步**：代码变更完成后立即同步更新文档

---

*本文件是重构过程的唯一执行参照，实施过程中需同步更新本文件的状态。*