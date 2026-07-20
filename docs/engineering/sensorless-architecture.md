# 无感 FOC 架构设计文档（草案）

> 本文档定义无感 FOC 的架构设计方案，聚焦模块划分、接口契约、数据流与编译期组合方式。
> 当前状态：**草案** ｜ 目标基线：v1.11.x → v1.12.x 规划
>
> **⚠️ 本文件为设计过程文档，代码落地前需转为正式架构更新**
> 任何不确定点均需要查阅该文档

---

## 目录

1. [范围与目标](#1-范围与目标)
2. [核心设计原则](#2-核心设计原则)
3. [数据结构设计](#3-数据结构设计)
4. [数据流设计](#4-数据流设计)
5. [估计器体系](#5-估计器体系)
6. [桥接层](#6-桥接层)
7. [启动策略模块](#7-启动策略模块)
8. [过渡管理](#8-过渡管理)
9. [文件组织方案](#9-文件组织方案)
10. [配置宏方案](#10-配置宏方案)
11. [L1 编排层改动](#11-l1-编排层改动)
12. [编码器有效性检查策略](#12-编码器有效性检查策略)
13. [依赖关系与组合约束](#13-依赖关系与组合约束)
14. [实施顺序](#14-实施顺序)
15. [运行时框架适配](#15-运行时框架适配)

---

## 1. 范围与目标

### 1.1 范围

- 从**当前仅支持编码器反馈**扩展到**多源估计器统一框架**，覆盖：
  - 编码器（有感）
  - SMO（中高速无感）
  - HFI（零速/低速无感）
  - 预留后续估计器扩展
- 启动策略作为独立模块（强拖等）
- 估计器动态切换机制（过渡管理）

### 1.2 非目标

- 不涉及具体估计算法的数学推导或实现步骤
- 不涉及 L3 数学库的修改（Clarke/Park/SVPWM 不变）
- 不新增协议命令（故障名、遥测标签、DebugStream 数据源为必要适配）
- 不限定具体估计器的个数、启动策略的种类

### 1.3 设计原则

1. **有感路径零回归**：所有修改不得破坏有感 FOC 的现有功能
2. **分层单向依赖不变**：L1 编排 → L2 控制 → L3 平台，L2/L3 不持实例，所有实例在 L1 分配，下层通过指针参数操作
3. **采样与控制解耦**：数据源（传感器/估计器）各自维护私有处理数据，控制算法只读不可变输入快照，通过桥接层在控制周期边界进行一次数据拷贝来选择来源
4. **估计器即反馈源**：编码器与 SMO/HFI 通过同一接口写入系统状态，无区别对待
5. **数据结构风格统一**：新增字段遵循现有 `foc_motor_t` 的子结构体组织规律，所有私有状态作为 `foc_motor_t` 的条件编译子结构体
6. **空框架优先**：架构阶段只定义接口和空桩，算法逐个独立实现
7. **编译期裁剪**：未选中的模块完全不编译
8. **改动彻底**：不接受兼容性签名转发，每次改动一次性完成

---

## 2. 核心设计原则

### 2.1 采样与控制解耦

采样路径（传感器/估计器）与控制路径（外环/电流环/SVPWM）完全解耦，仅通过拷贝/传值连接：

- **数据源层**：传感器驱动（L3）和估计器（L2）各自维护私有处理状态，写入各自的输出结构体
- **桥接层**（新增 L2 模块）：在控制周期边界，根据当前选中的估计器类型，从对应数据源拷贝值到统一的控制输入快照
- **控制算法层**：只读控制输入快照，不直接接触任何数据源的内部状态

**关键约束**：
- 控制算法**只读**控制输入快照，**绝不**回写到数据源
- 数据源**只写**自己的私有空间，**绝不**直接写控制输入快照
- `Sensor_ReadEncoder` 作为 L3 硬件抽象层函数**保留不变**
- 编码器估计器**调用** `Sensor_ReadEncoder` 获取原始值，在自身私有状态中做滤波处理

### 2.2 实例化原则

遵循现有分层约束：**L2/L3 不持实例，所有实例在 L1 分配**。

- 编码器估计器的私有 Kalman/LPF 状态 → `foc_motor_t` 的条件编译子结构体
- SMO/HFI 估计器的私有状态 → `foc_motor_t` 的条件编译子结构体
- 启动策略状态 → `foc_motor_t` 的条件编译子结构体
- 过渡管理状态 → `foc_motor_t` 的条件编译子结构体
- 桥接层 → 纯方法模块，不持实例，操作传入的 `foc_motor_t` 指针

### 2.3 现有数据结构组织规律

`foc_motor_t` 的当前组织规律：

```
foc_motor_t
├── 顶层标量（配置参数 + 核心运行时值）
├── 子结构体（无条件，按功能块聚合）
├── 子结构体（条件编译）
└── 辅助标量（条件编译）
```

**规律总结**：
- 按功能块聚合为子结构体，不散落零散字段
- 子结构体在 `foc_motor_t` 中聚合，而非定义为独立全局变量
- 条件编译的子结构体放在一起
- PID 控制器（`foc_pid_t`）作为顶层字段直接内嵌

---

## 3. 数据结构设计

### 3.1 `foc_est_state_t` — 估计器输出快照

```c
/* foc_ctrl_types.h — 估计器输出快照 */
typedef struct {
    uint8_t  source;              /* FOC_ESTIMATOR_TYPE_* */
    uint8_t  state;               /* FOC_ESTIMATOR_STATE_* */
    uint8_t  valid;
    float    confidence;          /* 0.0~1.0 */

    /* 电气域（估计器内部计算中间变量 & 调试用途，不进入 ctrl_input） */
    float    elec_angle_rad;
    float    elec_speed_rad_s;

    /* 机械域 */
    float    mech_angle_rad;
    float    mech_angle_accum_rad;  /* 调试/遥测用途，不进入控制链路 */
    float    mech_speed_rad_s;      /* 调试/遥测用途，不进入控制链路 */
} foc_est_state_t;
```

**不含电流字段**。电流由 `sensor_fast`／`sensor` 统一维护，不进入 `est_state`。

### 3.2 `foc_control_input_t` — 控制算法统一输入

```c
/* foc_ctrl_types.h — 控制算法输入快照（不可变，由桥接层填充） */

/* 累积角度和速度由外环从 mech_angle_rad 一阶差分自行维护
 * （FOC_UpdateAccumulatedMechanicalAngle / FOC_UpdateSpeedAngleError），
 * 不使用估计器预计算值，确保控制链路单一事实源。 */
typedef struct {
    uint8_t  valid;
    uint8_t  source;               /* 来源：FOC_ESTIMATOR_TYPE_* */

    /* 角度 */
    float    mech_angle_rad;       /* 瞬时机械角度（唯一数据源） */

    /* 电流 */
    float    current_a;
    float    current_b;
    float    current_c;
} foc_control_input_t;
```

### 3.3 `foc_estim_encoder_state_t` — 编码器估计器私有状态

```c
/* foc_ctrl_types.h — 编码器估计器私有滤波状态 */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    kalman_filter_t mech_angle_kalman;
    uint8_t         lpf_valid;
    float           lpf_state;
} foc_estim_encoder_state_t;
#endif
```

### 3.4 SMO/HFI 估计器私有状态

各估计器如需要私有状态（如 SMO 的滑模面、HFI 的解调状态），按相同模式定义：

```c
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    /* SMO 私有状态字段 */
    float    ialpha_est;
    float    ibeta_est;
    float    bemf_alpha;
    float    bemf_beta;
} foc_estim_smo_state_t;
#endif

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    /* HFI 私有状态字段 */
    float    hf_sin_demod;
    float    hf_cos_demod;
} foc_estim_hfi_state_t;
#endif
```

### 3.5 `foc_motor_t` 新增字段

```c
typedef struct {
    /* ... 所有现有字段保持不变 ... */

    /* ====== 估计器输出 ====== */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_SMO_ENABLE   == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_HFI_ENABLE   == FOC_CFG_ENABLE)
    foc_est_state_t est_state;
    foc_est_state_t est_state_alt;  /* 过渡期副估计器输出 */

    void (*estimator_step_fn)(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
    void (*estimator_step_fn_alt)(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
#endif

    /* ====== 编码器估计器私有状态 ====== */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    foc_estim_encoder_state_t estim_encoder_state;
#endif

    /* ====== SMO 估计器私有状态 ====== */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    foc_estim_smo_state_t estim_smo_state;
#endif

    /* ====== HFI 估计器私有状态 ====== */
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    foc_estim_hfi_state_t estim_hfi_state;
#endif

    /* ====== 过渡管理私有状态 ====== */
#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)
    foc_transition_state_t transition_state;
#endif

    /* ====== 启动策略私有状态 ====== */
#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)
    foc_startup_openloop_state_t startup_openloop_state;
#endif

    /* ====== 控制输入快照（无条件存在）====== */
    foc_control_input_t ctrl_input;

    /* ... 现有子结构体继续 ... */
} foc_motor_t;
```

### 3.6 `electrical_phase_angle` — 保留

保留 `motor->electrical_phase_angle` 作为顶层字段。其来源改为从 `ctrl_input` 换算（外环末尾写入），PWM ISR 电流环仍然直接读取。

### 3.7 `sensor_data_t` — 保持现有结构不变

**不剥离** `kalman_filter_t` 字段。`sensor_data_t` 仍是 L3 传感器层的输出结构。

```c
typedef struct {
    kalman_filter_t current_a;
    kalman_filter_t current_b;
    kalman_filter_t current_c;
    kalman_filter_t mech_angle_rad;  /* 保留，供编码器估计器读取原始值 */
    uint8_t adc_valid;
    uint8_t encoder_valid;
    float vbus_voltage_raw;
    float vbus_voltage_filtered;
    uint8_t vbus_valid;
    float current_a_raw;
    float current_b_raw;
} sensor_data_t;
```

---

## 4. 数据流设计

### 4.1 总览

```
┌─────────────────── 数据源层 ───────────────────┐
│                                                 │
│  Sensor_ReadCurrent → sensor_fast (电流Kalman)  │
│  Sensor_ReadEncoder → sensor*.mech_angle_rad    │
│  Sensor_ReadVBUS    → sensor.vbus_*             │
│                                                 │
│  foc_ctrl_estim_encoder → 读原始角度             │
│     → 私有 Kalman/LPF → est_state.mech_angle_rad│
│                                                 │
│  foc_ctrl_estim_smo → 读 sensor_fast 电流        │
│     → SMO → est_state.mech_angle_rad            │
└───────────────────┬─────────────────────────────┘
                    │
                    ▼
┌─────────────────── 桥接层 ──────────────────────┐
│                                                 │
│  FOC_Bridge_CopyInput(motor)                    │
│    est_state → ctrl_input (仅瞬时角度 + 电流)    │
└───────────────────┬─────────────────────────────┘
                    │
                    ▼  (Control ISR)
┌─────────────────── 控制算法层 ──────────────────┐
│                                                 │
│  外环: ctrl_input.mech_angle_rad                │
│        → FOC_UpdateAccumulatedMechanicalAngle   │
│        → FOC_UpdateSpeedAngleError              │
│        → motor->mech_angle_accum_rad (控制SRC)  │
│        → electrical_phase_angle                 │
│                                                 │
│  齿槽补偿 (只读 ctrl_input.mech_angle_rad)       │
│                                                 │
│  → 下一周期 PWM ISR（电流环 + SVPWM，            │
│     读 electrical_phase_angle + sensor_fast）    │
└─────────────────────────────────────────────────┘
```

### 4.2 PWM ISR 路径

PWM ISR 执行顺序严格分为 4 个阶段，**不可调换**：

```
FOC_ControlExecutor_RunISR:
  │
  ├── // === 阶段 1：插值与采样 ===
  ├── SVPWM_InterpolationISR(motor)                ← 优先输出上一周期的 SVPWM 插值结果
  ├── [phase != NORMAL] return                     ← STARTUP/COGGING/REINIT 阶段只做插值
  │
  ├── Sensor_ReadCurrent(motor)              → sensor_fast.current_*.raw_value
  ├── [FAST] Sensor_ReadEncoder(motor, &motor->sensor_fast)  → sensor_fast.mech_angle_rad
  ├── Sensor_AccumulateEcycle(motor, &motor->sensor_fast)
  │
  ├── // === 阶段 2：电流环计算（消费 sensor_fast 电流 + electrical_phase_angle，产出 ud/uq）===
  ├── FOC_CurrentControlStep(motor, &motor->sensor_fast,
  │        motor->electrical_phase_angle, dt)
  │     数据流: sensor_fast 电流 → Clarke/Park → PID → ud/uq
  │
  ├── // === 阶段 3：角度同步 + 估计器更新（消费 sensor_fast 原始角度，产出 est_state）===
  ├── [FAST] 角度同步: motor->sensor.mech_angle_rad.raw_value ← motor->sensor_fast.mech_angle_rad.raw_value
  │                    motor->sensor.encoder_valid ← motor->sensor_fast.encoder_valid
  │   (仅拷贝 raw_value 和有效性标志，不完整拷贝 kalman_filter_t)
  │
  ├── [estimator enabled] motor->estimator_step_fn(motor, &motor->est_state, dt)
  │     数据流: sensor.mech_angle_rad.raw_value → 编码器估计器 → est_state
  │             sensor_fast 电流 → SMO → est_state
  │     用途: 下一周期 Control ISR 的桥接层消费
  │
  ├── [alt enabled] motor->estimator_step_fn_alt(motor, &motor->est_state_alt, dt)
  │     数据流: 过渡期副估计器 → est_state_alt
  │
  ├── // === 阶段 4：SVPWM 输出（ISR 最后一步）===
  └── FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle)
       数据流: ud/uq + electrical_phase_angle → SVPWM 调制 → PWM 占空比
```

**执行顺序约束**：

| 阶段 | 产出 | 消费方 | 可调换？ |
|------|------|--------|---------|
| 1. 插值与采样 | `sensor_fast`（电流 + 角度原始值） | 阶段 2（电流环）、阶段 3（估计器） | 必须在阶段 2 之前 |
| 2. 电流环 | `ud/uq` | 阶段 4（SVPWM） | 必须在阶段 1 之后、阶段 4 之前 |
| 3. 角度同步+估计器 | `est_state`、`sensor` 同步 | 下一周期 Control ISR 阶段 3（桥接） | 必须在阶段 2 之后（读 ud/uq）、阶段 4 之前（不阻塞 SVPWM） |
| 4. SVPWM | PWM 占空比 | 硬件定时器 | 必须在阶段 2 之后（ISR 最后一步，硬件立即生效） |

**说明**：阶段 3（估计器）虽然不直接为阶段 2 提供数据（电流环读 `electrical_phase_angle` 来自上一周期 Control ISR），但可以利用阶段 2 计算的 ud/uq 更新自身——SMO 等估计器依赖电压量。Control ISR 在任意时刻可抢占，但不会修改 `sensor_fast`，不影响已完成的角度同步。

### 4.3 Control ISR 路径

Control ISR 执行顺序严格分为 4 个阶段，**不可调换**：

```
FOC_App_ControlTrigger:
  │
  ├── // === 阶段 1：传感器读取 ===
  ├── [SLOW] Sensor_ReadEncoder(&motor, &motor->sensor)     → sensor.mech_angle_rad.raw_value
  ├── [FAST] motor->sensor.mech_angle_rad ← motor->sensor_fast.mech_angle_rad
  ├── Sensor_ReadVBUS(&motor->sensor)                       → sensor.vbus_*
  ├── Sensor_SyncCurrentSnapshot(motor)                     → sensor.current_*.output_value
  │
  ├── 有效性检查 (ADC + [encoder] encoder)
  │
  ├── // === 阶段 2：估计器更新（消费 sensor 原始值，产出 est_state）===
  ├── [estimator enabled] motor->estimator_step_fn(&motor, &motor.est_state, FOC_CONTROL_DT_SEC)
  │     数据流: sensor.mech_angle_rad.raw_value → 估计器 → est_state.mech_angle_rad
  │
  ├── // === 阶段 3：桥接拷贝（消费 est_state + sensor 电流，填充 ctrl_input）===
  ├── FOC_Bridge_CopyInput(motor)                          ← 新增
  │     数据流: est_state.mech_angle_rad → ctrl_input.mech_angle_rad
  │            sensor.current_*.output_value → ctrl_input.current_*
  │
  ├── // === 阶段 4：控制执行（消费 ctrl_input）===
  ├── [system_fault] return
  │
  ├── switch (control_phase):
  │     STARTUP → FOC_Startup_RunStep(motor, dt)  (Bridge 不运行)
  │     NORMAL  → FOC_ControlExecutor_RunCycle(motor, dt)
  │                └── FOC_ControlExecutor_RunOuterLoop(motor, dt)
  │                     只读 ctrl_input.mech_angle_rad
  │                     维护 motor->mech_angle_accum_rad (控制单一事实源)
  │     COGGING_CALIB / REINIT → 不变
```

**执行顺序约束**：

| 阶段 | 产出 | 消费方 | 可调换？ |
|------|------|--------|---------|
| 1. 传感器 | `sensor_data_t` 原始值 | 阶段 2（估计器）、阶段 3（桥接电流） | 必须在阶段 2 之前 |
| 2. 估计器 | `est_state.mech_angle_rad` | 阶段 3（桥接） | 必须在阶段 1 之后、阶段 3 之前 |
| 3. 桥接 | `ctrl_input.mech_angle_rad` + `current_*` | 阶段 4（外环） | 必须在阶段 2 之后、阶段 4 之前 |
| 4. 外环 | `electrical_phase_angle` | 下一周期 PWM ISR（电流环+SVPWM） | 必须在阶段 3 之后 |

**结论**：4 个阶段按 1→2→3→4 严格串行，任意调换都会导致本周期控制输出滞后一拍。PWM ISR 在任意时刻可抢占，但仅修改 `sensor_fast` 和 `est_state`（Control ISR 阶段 1 之后已完成拷贝和桥接，不受影响）。

### 4.4 估计器运行节拍表

| 估计器 | FAST ISR | SLOW ISR | 需要启动策略 | 备注 |
|--------|---------|---------|------------|------|
| ENCODER | PWM ISR（FOC_SENSOR_ANGLE_FAST=1） | Control ISR（FAST=0） | 否 | 零速即可输出有效角度 |
| SMO | PWM ISR | — | 是（强拖或 HFI） | 需要电流 + 电压 |
| HFI | PWM ISR | — | 否（零速可用） | 注入高频信号 |

### 4.5 数据源→消费者完整映射

| 数据 | 生产者 | 写入位置 | 消费者 | 读取路径 |
|------|--------|---------|--------|---------|
| 电流（PWM ISR） | `Sensor_ReadCurrent` | `sensor_fast.current_*.output_value` | 电流环 PID | 直接读 `sensor_fast` |
| 电流（Control ISR） | `Sensor_SyncCurrentSnapshot` | `sensor.current_*.output_value` | 桥接层 → `ctrl_input` | `ctrl_input.current_*` |
| 机械角度（瞬时） | `foc_ctrl_estim_*` | `est_state.mech_angle_rad` | 桥接层 → `ctrl_input` | `ctrl_input.mech_angle_rad` |
| 机械角度累积 | 外环 `FOC_UpdateAccumulatedMechanicalAngle` | `motor->mech_angle_accum_rad` | 外环位置控制 (SpeedAngle) | 直接读 `motor->mech_angle_accum_rad` |
| 机械角速度 | 外环 `FOC_UpdateSpeedAngleError` | `motor->outer_loop_state` | 外环速度控制 | 直接读 `outer_loop_state` |
| 电气角度 | 外环（从 `ctrl_input` 换算） | `electrical_phase_angle` | 电流环 (PWM ISR) | 直接读 `electrical_phase_angle` |
| VBUS | `Sensor_ReadVBUS` | `sensor.vbus_voltage_filtered` | 低电压保护 | 直接读 `sensor` |
| DebugStream 快照 | DebugStream (Monitor ISR) | — | 主循环 | 从 `ctrl_input` + `motor->mech_angle_accum_rad` + `sensor` 读取 |

**说明**：估计器的 `est_state.mech_angle_accum_rad` / `mech_speed_rad_s` 仅用于调试/遥测，不进入控制链路。控制链路的累积角度和速度由外环从 `ctrl_input.mech_angle_rad` 一阶差分自行维护，确保单一事实源。

### 4.6 跨 ISR 安全

在裸机单核环境下，Bridge 在 Control ISR 入口立即从 `est_state` / `est_state_alt` 读取并拷贝到 `ctrl_input`，拷贝完成后后续 PWM ISR 抢占修改 `est_state` 不影响本周期控制输入。无需 shadow 双缓冲。

**时序保证**：PWM ISR 中的角度同步（`sensor_fast → sensor`，仅拷贝 raw_value）在电流环之后、估计器 Step 之前执行。Control ISR 中的 `Sensor_ReadEncoder` 直接写入 `motor->sensor`，无需额外同步。

---

## 5. 估计器体系

### 5.1 核心思想

**编码器也是一个估计器**。它接收 `Sensor_ReadEncoder` 输出的原始机械角度，在私有 Kalman/LPF 状态中完成算法处理，与其他估计器（SMO/HFI）按相同格式产出 `foc_est_state_t`——包含机电域角度和速度的状态结构体。

所有反馈源（编码器、SMO、HFI）通过相同的接口写入 `motor->est_state`。控制算法不直接读 `est_state`——桥接层（第 6 章）负责将其汇聚为 `ctrl_input`，控制算法只读 `ctrl_input`。

### 5.2 统一接口

```c
/* foc_ctrl_estim.h — 位于 foc_core/include/L2_Core/Control/ */

/* 估计器类型（功能裁剪宏，定义在 foc_cfg_feature_switches.h） */
#define FOC_ESTIMATOR_TYPE_NONE     0U
#define FOC_ESTIMATOR_TYPE_ENCODER  1U
#define FOC_ESTIMATOR_TYPE_SMO      2U
#define FOC_ESTIMATOR_TYPE_HFI      3U
#define FOC_ESTIMATOR_TYPE_FLUX     4U  /* 预留 */
#define FOC_ESTIMATOR_TYPE_BLEND    5U  /* 过渡混合输出 */

/* 估计器状态（接口定义宏） */
#define FOC_ESTIMATOR_STATE_INIT        0U
#define FOC_ESTIMATOR_STATE_CONVERGING  1U
#define FOC_ESTIMATOR_STATE_LOCKED      2U
#define FOC_ESTIMATOR_STATE_DIVERGED    3U

/* 函数指针类型：out 参数接收输出槽（&motor->est_state 或 &motor->est_state_alt） */
typedef void (*foc_estimator_step_t)(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
typedef void (*foc_estimator_init_t)(foc_motor_t *motor);

/* 选择/注册 */
void FOC_Estimator_Select(foc_motor_t *motor, uint8_t estimator_type);

/* 编码器估计器 */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimEncoder_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimEncoder_Init(foc_motor_t *motor);
#endif

/* SMO */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimSMO_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimSMO_Init(foc_motor_t *motor);
#endif

/* HFI */
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimHFI_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimHFI_Init(foc_motor_t *motor);
#endif

/* FLUX（预留桩） */
#if (FOC_ESTIMATOR_FLUX_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimFlux_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimFlux_Init(foc_motor_t *motor);
#endif
```

### 5.3 `FOC_Estimator_Select` 实现

放在 `foc_ctrl_estim.c`（选择/注册中心）：

```c
void FOC_Estimator_Select(foc_motor_t *motor, uint8_t estimator_type)
{
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_ENCODER)
    {
        motor->estimator_step_fn = FOC_EstimEncoder_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_ENCODER;
        return;
    }
#endif
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_SMO)
    {
        motor->estimator_step_fn = FOC_EstimSMO_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_SMO;
        return;
    }
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    if (estimator_type == FOC_ESTIMATOR_TYPE_HFI)
    {
        motor->estimator_step_fn = FOC_EstimHFI_Step;
        motor->est_state.source = FOC_ESTIMATOR_TYPE_HFI;
        return;
    }
#endif
    motor->estimator_step_fn = 0;
    motor->est_state.source = FOC_ESTIMATOR_TYPE_NONE;
}
```

### 5.4 各估计器文件

| 文件 | 估计器 | 备注 |
|------|--------|------|
| `foc_ctrl_estim.c` | 选择/注册中心 | 实现 `FOC_Estimator_Select` |
| `foc_ctrl_estim_encoder.c` | ENCODER | 从 `motor->sensor` 读原始角度，私有 Kalman/LPF |
| `foc_ctrl_estim_smo.c` | SMO | 空桩，读 `sensor_fast` 电流 |
| `foc_ctrl_estim_hfi.c` | HFI | 空桩 |
| `foc_ctrl_estim_flux.c` | FLUX（预留） | 空桩 |

### 5.5 编码器估计器实现要点

`foc_ctrl_estim_encoder.c`：

- **统一从 `motor->sensor.mech_angle_rad.raw_value` 读取原始角度**，不分支处理
- 在 `motor->estim_encoder_state`（`foc_estim_encoder_state_t`）中做 Kalman/LPF 滤波
- 通过 `out` 参数将处理后的 `mech_angle_rad` 写入目标 `est_state`
- 计算 `mech_angle_accum_rad`、`mech_speed_rad_s`、`elec_angle_rad`、`elec_speed_rad_s`（调试/遥测用）

**读数源保证**：
- PWM ISR 路径：电流环之后同步 `raw_value`→`sensor`（仅拷贝必要字段，不完整拷贝）
- Control ISR 路径：`Sensor_ReadEncoder` 直接写入 `motor->sensor`，无需额外同步

**不替换 `Sensor_ReadEncoder`**——编码器估计器是 L2 模块，`Sensor_ReadEncoder` 是 L3 硬件抽象层函数。

### 5.6 初始化

```c
/* L1 / foc_init.c */

/* 初始估计器选择由功能宏组合决定 */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimEncoder_Init(&motor);
    FOC_Estimator_Select(&motor, FOC_ESTIMATOR_TYPE_ENCODER);
#elif (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE) || \
      (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    /* 纯无感路径：初始无可用估计器，estimator_step_fn = 0，
     * Bridge 被 ctrl_input.valid = 0 阻塞，
     * Control ISR 进入 STARTUP 阶段。
     * 启动完成后由启动策略调用 FOC_Estimator_Select 切换到目标估计器。
     */
    motor->est_state.source = FOC_ESTIMATOR_TYPE_NONE;
#else
    motor->est_state.source = FOC_ESTIMATOR_TYPE_NONE;
#endif
```

### 5.7 可扩展性

新增估计器：
1. 添加 `FOC_ESTIMATOR_TYPE_*` 枚举值 → `foc_cfg_feature_switches.h`
2. 定义私有状态类型 → `foc_ctrl_types.h`
3. 在 `foc_motor_t` 中新增条件编译子结构体字段
4. 添加 `FOC_ESTIMATOR_XXX_ENABLE` 宏 → `foc_cfg_feature_switches.h`
5. 新建 `foc_ctrl_estim_xxx.c/h`
6. 在 `FOC_Estimator_Select` 中添加 case
无需修改现有控制算法。

---

## 6. 桥接层

### 6.1 接口

```c
/* foc_ctrl_bridge.h — 位于 foc_core/include/L2_Core/Control/ */

/* 在 Control ISR 中，传感器检查通过后、外环执行前调用。
 * 从 est_state 拷贝瞬时角度到 ctrl_input，并复制电流快照。
 * 过渡期 (source == BLEND) 时按权重混合两路瞬时角度。
 * 不拷贝累积角度和速度——这些由外环从瞬时角度自行维护。
 */
void FOC_Bridge_CopyInput(foc_motor_t *motor);
```

### 6.2 实现

```c
/* foc_ctrl_bridge.c */

void FOC_Bridge_CopyInput(foc_motor_t *motor)
{
    foc_est_state_t *est = &motor->est_state;

    motor->ctrl_input.source = est->source;

    if (est->valid == 0U)
    {
        motor->ctrl_input.valid = 0U;
        return;
    }

    motor->ctrl_input.valid = 1U;

    /* 瞬时角度：过渡期加权混合 */
    if (est->source == FOC_ESTIMATOR_TYPE_BLEND)
    {
        foc_est_state_t *alt = &motor->est_state_alt;
        float w = motor->transition_state.blend_factor;
        motor->ctrl_input.mech_angle_rad = (1.0f - w) * est->mech_angle_rad + w * alt->mech_angle_rad;
    }
    else
    {
        motor->ctrl_input.mech_angle_rad = est->mech_angle_rad;
    }

    /* 电流从 sensor 快照复制 */
    motor->ctrl_input.current_a = motor->sensor.current_a.output_value;
    motor->ctrl_input.current_b = motor->sensor.current_b.output_value;
    motor->ctrl_input.current_c = motor->sensor.current_c.output_value;
}
```

---

## 7. 启动策略模块

### 7.1 设计动机

某些估计器（如 SMO）在零速时无法输出有效位置。启动策略提供从零速到估计器可靠工作区间的过渡方式。

### 7.2 统一接口

```c
/* foc_ctrl_startup.h */

#define FOC_STARTUP_PHASE_IDLE      0U
#define FOC_STARTUP_PHASE_RUNNING   1U
#define FOC_STARTUP_PHASE_DONE      2U
#define FOC_STARTUP_PHASE_FAILED    3U

void FOC_Startup_Init(foc_motor_t *motor);
void FOC_Startup_RunStep(foc_motor_t *motor, float dt_sec);
uint8_t FOC_Startup_IsComplete(const foc_motor_t *motor);
void FOC_Startup_Abort(foc_motor_t *motor);
```

### 7.3 各估计器的启动需求

| 估计器 | 需要启动策略？ | 原因 |
|--------|-------------|------|
| ENCODER | 否 | 零速即有有效位置 |
| HFI | 否 | 零速即可闭环 |
| SMO | 是 | 需要反电动势，零速发散 |

### 7.4 启动策略私有状态

```c
/* foc_ctrl_types.h — 强拖启动私有状态 */
#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    uint8_t  phase;            /* FOC_STARTUP_PHASE_* */
    float    virtual_angle_rad;
    float    current_ref_a;
    float    ramp_rate_rad_s2;
    float    target_speed_rad_s;
} foc_startup_openloop_state_t;
#endif
```

### 7.5 文件

| 文件 | 对应策略 |
|------|---------|
| `foc_ctrl_startup.c` | 启动策略管理（状态机路由） |
| `foc_ctrl_startup.h` | 接口 |
| `foc_ctrl_startup_openloop.c` | 强拖（V/f 或 I/F），空桩 |

### 7.6 启动策略路径与切换条件

**SVPWM 路径**：PWM ISR 在 `phase != NORMAL` 时跳过电流环只保留 SVPWM 插值 ISR。启动策略在 Control ISR 的 `STARTUP` 分支中：
1. 直接写 `motor->electrical_phase_angle`（虚角度递增）
2. 直接写 `motor->iq_target`（开环电流参考值）或 `motor->uq`（通过直接电压控制）
3. Bridge **不在** STARTUP 阶段运行——控制输入由启动策略直接填充

**切换到 NORMAL 的条件**：
- 虚角度达到目标速度且估计器状态变为 `LOCKED` 或 `CONVERGING`
- 到 NORMAL 后 Bridge 才开始运行（从 `est_state` 读位置反馈）

```c
/* Control ISR (foc_app.c) */
case FOC_CONTROL_PHASE_STARTUP:
    FOC_Startup_RunStep(&motor, dt);
    if (FOC_Startup_IsComplete(&motor))
    {
        motor.state.control_phase = FOC_CONTROL_PHASE_NORMAL;
        FOC_Estimator_Select(&motor, FOC_ESTIMATOR_TYPE_SMO);
    }
    break;
```

---

## 8. 过渡管理

### 8.1 切换场景

| 切换 | 需要混合过渡 |
|------|------------|
| ENCODER ↔ SMO | 是（避免角度突变） |
| HFI ↔ SMO | 是 |
| 同估计器 reinit | 否 |

### 8.2 模块

```c
/* foc_ctrl_transition.h */
void FOC_Transition_Init(foc_motor_t *motor);
void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec);
uint8_t FOC_Transition_Request(foc_motor_t *motor, uint8_t target_estimator_type);
```

### 8.3 双估计器同时运行机制

过渡期两个估计器需同时输出以做加权混合。采用**双函数指针**：

```c
motor->estimator_step_fn     = /* 当前估计器 */
motor->estimator_step_fn_alt = /* 目标估计器 */
motor->est_state.source      = FOC_ESTIMATOR_TYPE_BLEND;
```

PWM ISR 末尾两者都运行（通过 `out` 参数区分写入 `est_state` 还是 `est_state_alt`）。桥接层在 `source == BLEND` 时仅混合瞬时角度 `mech_angle_rad`。

**过渡触发条件**：`FOC_Transition_Request` 检查目标估计器宏使能且 `est_state_alt` 有效性满足门限（`valid && state != DIVERGED`），然后初始化 `estimator_step_fn_alt` 和 `blend_factor = 0.0`。

**过渡完成条件**：`blend_factor >= 1.0` → 卸载副估计器：

```c
FOC_Estimator_Select(motor, target_type);
motor->estimator_step_fn_alt = 0;
memset(&motor->est_state_alt, 0, sizeof(foc_est_state_t));
```

### 8.4 过渡管理私有状态

```c
/* foc_ctrl_types.h — 过渡管理状态 */
#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    float    blend_factor;      /* 0.0~1.0，0=全主估计器 */
    float    blend_rate;        /* 过渡速率 */
    uint8_t  active;
    uint8_t  target_source;
} foc_transition_state_t;
#endif
```

---

## 9. 文件组织方案

### 9.1 文件列表与模块编号

```
foc_core/src/L2_Core/Control/
├── foc_ctrl_executor.c                  C11 [修改] 算法入口
├── foc_ctrl_init.c                      C12 [修改]
├── foc_ctrl_cfg.c                       C13 配置管理
├── foc_ctrl_bridge.c                    C14 [新] 桥接层
├── foc_ctrl_estim.c                     C15 [新] 估计器选择/注册
├── foc_ctrl_estim_encoder.c             C16 [新] 编码器估计器
├── foc_ctrl_estim_smo.c                 C17 [新] SMO (空桩)
├── foc_ctrl_estim_hfi.c                 C18 [新] HFI (空桩)
├── foc_ctrl_estim_flux.c                C19 [新] FLUX (预留桩)
├── foc_ctrl_outer_loop.c                C21 [修改-签名] 外环
├── foc_ctrl_current_loop.c              C22 电流环（不变）
├── foc_ctrl_param_learn.c               C23 参数学习（不变）
├── foc_ctrl_compensation.c              C24 [修改-调用方] 齿槽补偿
├── foc_ctrl_cogging_calib.c             C25 齿槽标定
├── foc_ctrl_reinit.c                    C26 重初始化
├── foc_ctrl_startup.c                   C27 [新] 启动策略管理
├── foc_ctrl_startup_openloop.c          C28 [新] 强拖启动 (空桩)
├── foc_ctrl_transition.c                C29 [新] 过渡管理
└── foc_ctrl_actuation.c                 C31 SVPWM 执行（不变）
```

### 9.2 头文件路径

```
foc_core/include/L2_Core/Control/
├── foc_ctrl_estim.h
├── foc_ctrl_bridge.h
├── foc_ctrl_startup.h
├── foc_ctrl_transition.h
├── foc_ctrl_estim_encoder.h
├── foc_ctrl_estim_smo.h
├── foc_ctrl_estim_hfi.h
├── foc_ctrl_estim_flux.h
├── foc_ctrl_startup_openloop.h
└── (现有头文件不变)
```

### 9.3 命名约定

- 估计器：`foc_ctrl_estim_xxx.c/h`
- 启动策略：`foc_ctrl_startup_xxx.c/h`
- 桥接：`foc_ctrl_bridge.c/h`

### 9.4 有感模块重命名

架构调整完成后立即执行（算法实现前），为无感模块清出命名空间：

- `foc_ctrl_cogging_calib.c` → `foc_ctrl_sens_cogging_calib.c`
- `foc_ctrl_reinit.c` → `foc_ctrl_sens_reinit.c`
- 新增 `foc_ctrl_sens_calib.c`（从 `foc_ctrl_init.c` 拆分有感标定部分）

---

## 10. 配置宏方案

### 10.1 功能开关

定义位置：`foc_core/include/LS_Config/foc_cfg_feature_switches.h`

```c
/* ── 传感器硬件使能 ── */
#define FOC_SENSOR_ENCODER_ENABLE         FOC_CFG_ENABLE
/* 编码器硬件使能：有感=ENABLE，无感=DISABLE */

/* ── 估计器 ── */
#define FOC_ESTIMATOR_ENCODER_ENABLE   FOC_CFG_ENABLE
#define FOC_ESTIMATOR_SMO_ENABLE       FOC_CFG_DISABLE
#define FOC_ESTIMATOR_HFI_ENABLE       FOC_CFG_DISABLE
#define FOC_ESTIMATOR_FLUX_ENABLE      FOC_CFG_DISABLE

/* ── 启动策略 ── */
#define FOC_STARTUP_OPENLOOP_ENABLE    FOC_CFG_DISABLE

/* ── 过渡管理 ── */
#define FOC_TRANSITION_ENABLE          FOC_CFG_DISABLE
```

### 10.2 编译期约束

定义位置：`foc_core/include/LS_Config/foc_compile_limits.h`

```c
/* 编码器估计器依赖编码器硬件 */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) && \
    (FOC_SENSOR_ENCODER_ENABLE != FOC_CFG_ENABLE)
#error "FOC_ESTIMATOR_ENCODER_ENABLE requires FOC_SENSOR_ENCODER_ENABLE"
#endif

/* 齿槽补偿依赖编码器硬件 */
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE) && \
    (FOC_SENSOR_ENCODER_ENABLE != FOC_CFG_ENABLE)
#error "FOC_COGGING_COMP_ENABLE requires FOC_SENSOR_ENCODER_ENABLE"
#endif

/* 齿槽标定依赖编码器硬件 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE) && \
    (FOC_SENSOR_ENCODER_ENABLE != FOC_CFG_ENABLE)
#error "FOC_COGGING_CALIB_ENABLE requires FOC_SENSOR_ENCODER_ENABLE"
#endif

/* 新增宏合法性检查 */
#if ((FOC_SENSOR_ENCODER_ENABLE != FOC_CFG_DISABLE) && (FOC_SENSOR_ENCODER_ENABLE != FOC_CFG_ENABLE))
#error "FOC_SENSOR_ENCODER_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_ESTIMATOR_ENCODER_ENABLE != FOC_CFG_DISABLE) && (FOC_ESTIMATOR_ENCODER_ENABLE != FOC_CFG_ENABLE))
#error "FOC_ESTIMATOR_ENCODER_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_ESTIMATOR_SMO_ENABLE != FOC_CFG_DISABLE) && (FOC_ESTIMATOR_SMO_ENABLE != FOC_CFG_ENABLE))
#error "FOC_ESTIMATOR_SMO_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_ESTIMATOR_HFI_ENABLE != FOC_CFG_DISABLE) && (FOC_ESTIMATOR_HFI_ENABLE != FOC_CFG_ENABLE))
#error "FOC_ESTIMATOR_HFI_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_ESTIMATOR_FLUX_ENABLE != FOC_CFG_DISABLE) && (FOC_ESTIMATOR_FLUX_ENABLE != FOC_CFG_ENABLE))
#error "FOC_ESTIMATOR_FLUX_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_STARTUP_OPENLOOP_ENABLE != FOC_CFG_DISABLE) && (FOC_STARTUP_OPENLOOP_ENABLE != FOC_CFG_ENABLE))
#error "FOC_STARTUP_OPENLOOP_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif
#if ((FOC_TRANSITION_ENABLE != FOC_CFG_DISABLE) && (FOC_TRANSITION_ENABLE != FOC_CFG_ENABLE))
#error "FOC_TRANSITION_ENABLE must be FOC_CFG_ENABLE or FOC_CFG_DISABLE"
#endif

/* 无感估计器需电流采样 */
#if ((FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE) || \
     (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)) && \
    (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
#error "Sensorless estimators require current sensing"
#endif

/* SMO 需要至少一种启动策略（当编码器不可用时） */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE) && \
    (FOC_ESTIMATOR_ENCODER_ENABLE != FOC_CFG_ENABLE) && \
    (FOC_STARTUP_OPENLOOP_ENABLE != FOC_CFG_ENABLE) && \
    (FOC_ESTIMATOR_HFI_ENABLE != FOC_CFG_ENABLE)
#error "SMO requires at least one startup strategy (HFI or OPENLOOP) when encoder not available"
#endif

/* 过渡管理启用条件：至少两个估计器同时使能 */
#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)
#if ((FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) + \
     (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE) + \
     (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE) < 2)
#error "FOC_TRANSITION_ENABLE requires at least two estimators enabled"
#endif
```

### 10.3 默认值

定义位置：`foc_core/include/LS_Config/foc_cfg_init_values.h`（新增）

```c
/* ── 估计器默认参数 ── */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
#define FOC_ESTIM_SMO_K_SLIDE_DEFAULT     1.5f
#define FOC_ESTIM_SMO_PLL_KP_DEFAULT      0.5f
#define FOC_ESTIM_SMO_PLL_KI_DEFAULT      0.01f
#endif

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
#define FOC_ESTIM_HFI_INJ_AMPLITUDE_V     10.0f
#define FOC_ESTIM_HFI_INJ_FREQ_HZ         500.0f
#endif

/* ── 启动策略默认参数 ── */
#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)
#define FOC_STARTUP_OPENLOOP_RAMP_RATE_RAD_S2  10.0f
#define FOC_STARTUP_OPENLOOP_CURRENT_A         0.5f
#endif

/* ── 过渡管理默认参数 ── */
#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)
#define FOC_TRANSITION_BLEND_RATE_DEFAULT  0.05f
#endif
```

### 10.4 常见组合

| 场景 | 宏配置 |
|------|--------|
| 有感 FOC（当前） | `SENSOR_ENCODER=ENABLE, ESTIMATOR_ENCODER=ENABLE, SMO/HFI=DISABLE` |
| 强拖→SMO 无感 | `SENSOR_ENCODER=DISABLE, ESTIMATOR_ENCODER=DISABLE, SMO=ENABLE, STARTUP_OPENLOOP=ENABLE` |
| HFI→SMO 无感 | `SENSOR_ENCODER=DISABLE, SMO=ENABLE, HFI=ENABLE, TRANSITION=ENABLE` |
| HFI 纯无感 | `SENSOR_ENCODER=DISABLE, HFI=ENABLE` |

---

## 11. L1 编排层改动

### 11.1 `foc_control_phase_t` 扩展

```c
/* foc_ctrl_types.h */
typedef enum {
    FOC_CONTROL_PHASE_NORMAL           = 0U,
    FOC_CONTROL_PHASE_COGGING_CALIB    = 1U,
    FOC_CONTROL_PHASE_REINIT           = 2U,
    FOC_CONTROL_PHASE_STARTUP          = 3U,  /* 新增：强拖/对齐等启动策略 */
} foc_control_phase_t;
```

### 11.2 `FOC_App_ControlTrigger` 修改（概念）

```c
void FOC_App_ControlTrigger(void)
{
    /* SLOW encoder 读取 */
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    Sensor_ReadEncoder(&motor, &motor.sensor);
#else
    motor.sensor.mech_angle_rad = motor.sensor_fast.mech_angle_rad;
    motor.sensor.encoder_valid = motor.sensor_fast.encoder_valid;
#endif

    Sensor_ReadVBUS(&motor.sensor);
    Sensor_SyncCurrentSnapshot(&motor);

    /* 传感器有效性检查 */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((motor.sensor.adc_valid == 0U) || (motor.sensor.encoder_valid == 0U))
#else
    if (motor.sensor.adc_valid == 0U)
#endif
    {
        /* fault handling */
        return;
    }

    /* 慢速估计器：统一从 motor->sensor 读取原始角度 */
    if (motor.estimator_step_fn != 0)
        motor.estimator_step_fn(&motor, &motor.est_state, FOC_CONTROL_DT_SEC);

    /* 桥接 */
    FOC_Bridge_CopyInput(&motor);

    /* UVLO 等检查不变 */

    switch (phase)
    {
    case FOC_CONTROL_PHASE_NORMAL:
        cycle_result = FOC_ControlExecutor_RunCycle(&motor, FOC_CONTROL_DT_SEC);
        break;
    case FOC_CONTROL_PHASE_STARTUP:
        FOC_Startup_RunStep(&motor, FOC_CONTROL_DT_SEC);
        break;
    /* COGGING_CALIB / REINIT 不变 */
    }
}
```

### 11.3 `FOC_ControlExecutor_RunCycle` 签名

```c
/* 修改前 */
uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor,
                                     const sensor_data_t *sensor,
                                     float dt_sec);

/* 修改后 */
uint8_t FOC_ControlExecutor_RunCycle(foc_motor_t *motor, float dt_sec);
```

传感器有效性检查已由 L1 `FOC_App_ControlTrigger` 完成，`RunCycle` 内移除重复检查。

`FOC_ControlExecutor_RunOuterLoop` 签名同步修改：

```c
/* 修改前 */
void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor,
                                      const sensor_data_t *sensor,
                                      float dt_sec);

/* 修改后 */
void FOC_ControlExecutor_RunOuterLoop(foc_motor_t *motor, float dt_sec);
```

`RunOuterLoop` 内部所有 `sensor->mech_angle_rad.output_value` 替换为 `motor->ctrl_input.mech_angle_rad`。

### 11.4 外环函数签名

```c
/* 修改前 */
void FOC_SpeedOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                            float speed_ref_rad_s, const sensor_data_t *sensor, float dt_sec);
void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad, float angle_position_speed_rad_s,
                                 const sensor_data_t *sensor, float dt_sec);

/* 修改后 */
void FOC_SpeedOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                            float speed_ref_rad_s, float dt_sec);
void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor, foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad, float angle_position_speed_rad_s,
                                 float dt_sec);
```

函数内部 `sensor->mech_angle_rad.output_value` 全部替换为 `motor->ctrl_input.mech_angle_rad`。

**累积角度维护说明**：外环使用已有的 `FOC_UpdateAccumulatedMechanicalAngle` 和 `FOC_UpdateSpeedAngleError` 从 `ctrl_input.mech_angle_rad` 维护累积角度和速度，不使用估计器预计算值。`est_state.mech_angle_accum_rad` / `mech_speed_rad_s` 仅用于调试/遥测。此设计确保控制链路单一事实源（`motor->mech_angle_accum_rad`）。

### 11.5 `FOC_ControlExecutor_RunISR` 修改

PWM ISR 执行顺序：电流环(计算ud/uq) → 角度同步 → 估计器 → SVPWM(输出)。估计器在 SVPWM 之前运行，可利用本周期电流环计算的 ud/uq 更新估计。

```c
void FOC_ControlExecutor_RunISR(foc_motor_t *motor)
{
    /* ... 采样、e-cycle 等现有逻辑 ... */

    /* 电流环：计算 ud/uq，不执行 SVPWM */
    FOC_CurrentControlStep(motor, &motor->sensor_fast,
                           motor->electrical_phase_angle,
                           current_loop_dt_sec);

    /* 新增：角度同步（电流环后、估计器前），
     * 仅拷贝 raw_value 和有效性标志，不完整拷贝 kalman_filter_t */
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_ENABLE)
    motor->sensor.mech_angle_rad.raw_value = motor->sensor_fast.mech_angle_rad.raw_value;
    motor->sensor.encoder_valid = motor->sensor_fast.encoder_valid;
#endif

    /* 新增：运行估计器（与电流环同频，使用 current_loop_dt_sec） */
#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_SMO_ENABLE   == FOC_CFG_ENABLE) || \
    (FOC_ESTIMATOR_HFI_ENABLE   == FOC_CFG_ENABLE)
    if (motor->estimator_step_fn != 0)
        motor->estimator_step_fn(motor, &motor->est_state, current_loop_dt_sec);
    if (motor->estimator_step_fn_alt != 0)
        motor->estimator_step_fn_alt(motor, &motor->est_state_alt, current_loop_dt_sec);
#endif

    /* SVPWM 执行：ISR 最后一步，将 ud/uq 转换为 PWM 占空比输出 */
    FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
}
```

**无电流传感器分支（`FOC_ControlRequiresCurrentSample() == 0`）**：估计器 Step 同样需要运行（编码器估计器不依赖电流）。在 `else` 分支中按上述相同顺序：`FOC_CurrentControlStep` → 角度同步 → 估计器 Step → `FOC_ControlApplyElectricalAngleRuntime`。

---

## 12. 编码器有效性检查策略

### 12.1 硬件宏 vs 估计器宏

新增独立的硬件层宏 `FOC_SENSOR_ENCODER_ENABLE`，与估计器宏 `FOC_ESTIMATOR_ENCODER_ENABLE` 分离：

- `FOC_SENSOR_ENCODER_ENABLE` — 硬件层面是否连接编码器
- `FOC_ESTIMATOR_ENCODER_ENABLE` — 算法层面是否使用编码器作为反馈源

无感路径：`FOC_SENSOR_ENCODER_ENABLE = DISABLE`
有感路径：`FOC_SENSOR_ENCODER_ENABLE = ENABLE` + `FOC_ESTIMATOR_ENCODER_ENABLE = ENABLE`

### 12.2 有效性检查宏

```c
/* foc_app.c — FOC_App_ControlTrigger */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((motor.sensor.adc_valid == 0U) || (motor.sensor.encoder_valid == 0U))
#else
    if (motor.sensor.adc_valid == 0U)
#endif
    {
        motor.state.sensor_invalid_consecutive++;
        /* ... fault handling ... */
    }
```

### 12.3 Init_Verify 初始化检查适配

`foc_init.c — FOC_Init_Verify` 当前强依赖 `encoder_valid`。无感配置需宏裁剪：

```c
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((sensor->adc_valid != 0U) && (sensor->encoder_valid != 0U))
#else
    if (sensor->adc_valid != 0U)
#endif
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
```

无感配置下，`RUNTIME_INIT_CHECK_MOTOR` 的检查条件放宽（启动策略可提供方向初始化）。

### 12.4 估计器有效性故障码与协议故障名

```c
/* foc_ctrl_types.h */
typedef enum {
    FOC_FAULT_NONE = 0U,
    FOC_FAULT_SENSOR_ADC_INVALID = 1U,
    FOC_FAULT_SENSOR_ENCODER_INVALID = 2U,
    FOC_FAULT_UNDERVOLTAGE = 3U,
    FOC_FAULT_PROTOCOL_FRAME = 4U,
    FOC_FAULT_PARAM_INVALID = 5U,
    FOC_FAULT_INIT_FAILED = 6U,
    FOC_FAULT_ESTIMATOR_INVALID = 7U   /* 新增：估计器发散保护 */
} foc_fault_code_t;

/* 协议故障名同步：FOC_Protocol_GetFaultName 需新增 case 7 映射 "estimator diverged" */
```

---

## 13. 依赖关系与组合约束

### 13.1 功能依赖

| 功能 | 依赖 |
|------|------|
| 编码器估计器 | `FOC_SENSOR_ENCODER_ENABLE` + 编码器硬件 |
| SMO | `FOC_CURRENT_SENSE_PHASES != NONE` |
| HFI | `FOC_CURRENT_SENSE_PHASES != NONE` + 凸极电机 |
| 强拖 | 无（可无电流，但带电流更好） |
| 过渡管理 | 至少两个估计器同时使能 |
| 齿槽补偿 | 编码器 + 位置精度 |
| 齿槽标定 | 编码器 |

### 13.2 控制算法依赖路径

控制算法不直接依赖任何估计器——只依赖 `ctrl_input`：

```
外环 → ctrl_input.mech_angle_rad
       → FOC_UpdateAccumulatedMechanicalAngle → motor->mech_angle_accum_rad
       → FOC_UpdateSpeedAngleError → motor->outer_loop_state

电流环 → sensor_fast (PWM ISR 直接读)
       → electrical_phase_angle (外环写入)

齿槽补偿 → ctrl_input.mech_angle_rad
```

---

## 14. 实施顺序

每步通过编译验证，有感功能不退化。

| 步骤 | 内容 | 验证 |
|------|------|------|
| 1 | `foc_ctrl_types.h`：新增 `foc_control_input_t`（仅含 `mech_angle_rad` + 电流）、`foc_est_state_t`、`foc_estim_encoder_state_t` 等类型，`foc_motor_t` 新增对应字段 + 函数指针 + `ctrl_input` | 编译通过 |
| 2 | `foc_cfg_feature_switches.h`：新增 `FOC_SENSOR_ENCODER_ENABLE` + 估计器/启动/过渡宏。`foc_compile_limits.h`：新增约束（含齿槽补偿/标定依赖）。`foc_cfg_init_values.h`：新增默认值 | 编译通过 |
| 3 | `foc_control_phase_t` 新增 `STARTUP`，故障码新增 `ESTIMATOR_INVALID` | 编译通过 |
| 4 | `foc_ctrl_estim.h/c` (选择/注册) + 各估计器空桩 `.c` | 编译通过 |
| 5 | `foc_ctrl_bridge.h/c` 实现（仅拷贝瞬时角度 + 电流） | 编译通过 |
| 6 | `foc_ctrl_startup.h/c` + `foc_ctrl_startup_openloop.c` 空桩 | 编译通过 |
| 7 | `foc_ctrl_transition.h/c` 空桩 | 编译通过 |
| 8 | `foc_ctrl_estim_encoder.c` 实现（从 `motor->sensor` 读原始角度，私有 Kalman/LPF，写入 `est_state`） | 编译 + 有感功能回归 |
| 9 | `foc_ctrl_outer_loop.c` 签名修改：去 `sensor_data_t *`，改用 `ctrl_input`。同步修改 `RunOuterLoop` 签名。外环从 `ctrl_input.mech_angle_rad` 维护累积角度 | 编译通过 |
| 10 | `foc_ctrl_executor.c` 修改：角度同步（仅拷贝 raw_value）+ 估计器 Step + `RunCycle` 签名 + `RunOuterLoop` 调用参数 | 编译通过 |
| 11 | `foc_ctrl_compensation.c`：调用方 `RunOuterLoop` 参数修改（函数签名不变） | 编译通过 |
| 12 | `foc_app.c` 修改：`STARTUP` phase + bridge 调用 + encoder_valid 检查宏裁剪 + `RunCycle` 签名 | 编译 + 有感功能回归 |
| 13 | DebugStream 快照路径适配（角度从 `ctrl_input` + `motor->mech_angle_accum_rad` 读取） | 编译通过 |
| 14 | `builder.params` 更新：新增 `foc_ctrl_estim/bridge/startup/transition` 等文件 | 编译通过 |
| 15 | 有感模块重命名（架构调整收尾：加 `sens_` 前缀，拆分 `sens_calib`） | 编译通过 |
| 16 | 逐个实现 SMO / HFI / 启动策略 / 过渡管理 | 模块测试 |

### builder.params 更新清单

**新增文件**：
- `foc_core/src/L2_Core/Control/foc_ctrl_bridge.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_estim.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_estim_encoder.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_estim_smo.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_estim_hfi.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_estim_flux.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_startup.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_startup_openloop.c`
- `foc_core/src/L2_Core/Control/foc_ctrl_transition.c`

---

## 15. 运行时框架适配

### 15.1 概述

现有 `foc_debug_stream.c`（L2/Runtime）在语义遥测和示波器数据采集中**硬编码了 `encoder_valid` 门控和 `sensor->mech_angle_rad.output_value` 数据源**。扩展至无感 FOC 后，需要修改 DebugStream 使其通过 `ctrl_input` 和 `motor->mech_angle_accum_rad` 读取角度数据，从而与桥接层和控制算法保持一致。

**核心原则**：Monitor ISR 的调试目的不是独立采集，而是反映控制算法实际看到的状态。因此 DebugStream 不引入新的快照结构体，直接从 `motor->ctrl_input`（桥接层输出）、`motor->mech_angle_accum_rad`（外环维护的控制视角累积角度）和 `motor->sensor`（电流数据）读取。

**时序保证**：Monitor ISR 由调度器在 Control ISR 之后触发，此时 `ctrl_input` 已经过桥接层填充完成，`mech_angle_accum_rad` 已由外环更新。

### 15.2 冲突点分析

#### 冲突 A：示波器被完全阻塞（`DebugStream_IsOscInputValid`）

`foc_debug_stream.c:48-52`：
```c
static uint8_t DebugStream_IsOscInputValid(const foc_motor_t *motor)
{
    if (motor == 0) return 0U;
    if ((motor->sensor.adc_valid == 0U) || (motor->sensor.encoder_valid == 0U)) return 0U;
    return 1U;
}
```
无感路径 `encoder_valid` 永远为 0，示波器永不触发。

#### 冲突 B：语义遥测角度行门控 encoder

`PollSemantic` case 3/4 仅在 `encoder_valid` 为 1 时输出角度 raw/filtered 值。无感路径两行永远无效。

#### 冲突 C：示波器角度读 `sensor->mech_angle_rad.output_value`

OSC bit 3 读的是 `sensor_data_t` 中的 Kalman 输出，与控制算法读的 `ctrl_input.mech_angle_rad`（估计器处理后值）是不同来源，调试输出与控制实际情况不同步。

### 15.3 解决方案：DebugStream 直接从 `ctrl_input` 读取

**不引入新的快照结构体**。`ctrl_input` 已存在于 `foc_motor_t` 中，Monitor ISR 在 Control ISR 之后执行，时序安全。

#### 修改清单

| 位置 | 当前代码 | 修改为 |
|------|---------|--------|
| `DebugStream_IsOscInputValid` | `!adc_valid \|\| !encoder_valid` | `!adc_valid \|\| !motor->ctrl_input.valid` |
| `PollSemantic` case 3 (angle raw) | `motor->sensor.mech_angle_rad.raw_value` + `encoder_valid` 门控 | `motor->sensor.mech_angle_rad.raw_value` + `ctrl_input.valid` 门控 |
| `PollSemantic` case 4 (angle filtered) | `motor->sensor.mech_angle_rad.output_value` + `encoder_valid` 门控 | `motor->ctrl_input.mech_angle_rad` + `ctrl_input.valid` 门控 |
| OSC case 3 (bit 3 / angle filtered) | `motor->sensor.mech_angle_rad.output_value` | `motor->ctrl_input.mech_angle_rad` |
| OSC case 4 (bit 4 / angle accum) | `motor->mech_angle_accum_rad` | `motor->mech_angle_accum_rad`（不变，但语义改为控制视角） |

**电流路径不变**：`PollSemantic` case 0/1/2 和 OSC case 0/1/2/9/10 继续从 `motor->sensor.current_*` 读取。

#### 语义行标签更新

当前第 3/4 行标签：
```
"measurement.encoder_angle_raw_rad"
"measurement.encoder_angle_filtered_rad"
```
修改为通用标签：
```
"measurement.mech_angle_raw_rad"
"measurement.mech_angle_filtered_rad"
```
标签始终保持通用，不依赖反馈源类型。

#### 实施步骤拆分

将原有步骤 13（DebugStream 适配）拆分为：

| 步骤 | 内容 | 验证 |
|------|------|------|
| 13a | `foc_debug_stream.c`：修改 `IsOscInputValid` 门控条件 | 无感配置下 OSC 不再被阻塞 |
| 13b | `foc_debug_stream.c`：修改 `PollSemantic` case 3/4 数据源和门控 | 语义遥测角度行无感/有感下均正常输出 |
| 13c | `foc_debug_stream.c`：修改 OSC case 3/4 数据源 | 示波器与控制算法看同一份角度 |
| 13d | `foc_debug_stream.c`：修改 `FormatSemanticLine` case 3/4 标签 | 串口输出标签不含 "encoder" 字样 |
| 13e | `foc_debug_stream.c`：修改 `FormatOscLine` 对应标签（若存在） | 示波器标签同步更新 |

---

> **文档版本**：v0.13-draft
> **最后更新**：2026-07-18
> **修正历程**：
> - v0.1 → v0.2：修正强拖/HFI 串行假设为正交组合
> - v0.2 → v0.3：引入 `foc_state_estimate_t` 唯一数据源，编码器作为估计器
> - v0.3 → v0.4：重新评估 `foc_motor_t` 组织规律
> - v0.4 → v0.5：**全面重新设计数据流**——桥接层 `foc_control_input_t`、双函数指针过渡机制、`FOC_SENSOR_ENCODER_ENABLE`
> - v0.5 → v0.6：**新增运行时框架适配**——DebugStream 从 `ctrl_input` 读取角度
> - v0.6 → v0.7：删除 shadow 双缓冲；修正估计器/桥接职责描述
> - v0.7 → v0.8：**外部反馈查漏补缺**——估计器 Step 签名加 `foc_est_state_t *out`；启动策略 SVPWM 路径/切换条件；Init_Verify 宏裁剪；过渡管理触发/完成条件；估计器运行节拍表；协议故障名同步；有感重命名降级
> - v0.8 → v0.9：**文档审查修正**——编码器估计器统一从 `motor->sensor` 读数（PWM ISR 增加角度同步步骤）；`RunOuterLoop` 签名同步修改；增加多估计器初始化策略和齿槽补偿/标定编译期约束；`foc_ctrl_compensation.c` 标注从"修改-签名"改为"修改-调用方"；PWM ISR `else` 分支增加估计器 Step
> - v0.9 → v0.10：**数据流精简**——`ctrl_input` 移除 `mech_angle_accum_rad`/`mech_speed_rad_s`（仅保留瞬时 `mech_angle_rad` + 电流）；累积角度和速度由外环自行维护（单一事实源）；桥接层简化（仅拷贝瞬时角度）；PWM ISR 角度同步改为仅拷贝 `raw_value`（不完整拷贝 `kalman_filter_t`）；`est_state` 中非控制字段标注为调试/遥测用途；过渡混合段仅混合瞬时角度
> - v0.10 → v0.11：**PWM ISR 时序修正**——4.1 总览图移除放错层的"电流环→SVPWM"，改为箭头标注 PWM ISR 消费；4.2 PWM ISR 路径拆分电流环(计算ud/uq)和SVPWM(输出)，估计器置于两者之间；11.5 RunISR 代码片段显式列出四步顺序
> - v0.11 → v0.12：**Control ISR 时序明确**——4.3 重写为 4 阶段（传感器→估计器→桥接→控制），标注不可调换约束；新增执行顺序约束表
> - v0.12 → v0.13：**PWM ISR 时序标准化**——4.2 升级为与 4.3 一致的 4 阶段格式（插值与采样→电流环→角度同步+估计器→SVPWM），新增约束表和阶段说明，明确每阶段的产出/消费者/可调换性
