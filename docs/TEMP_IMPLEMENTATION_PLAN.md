# 无感FOC开发实现计划（临时文档）

> **状态：架构设计阶段，待审阅**
> **创建日期：2026-07-22**
> **最后修订：2026-07-24（v13：明确 SMO 时序、特殊输出状态与初始化 ISR 边界）**
>
> 此文档用于锁定实现计划，防止执行过程中的漂移。实施前需经审核确认。

注意：当前项目状态为部分实现，控制算法部分和相关的变量组织、文件组织均存在耦合问题。本文档描述的是重构后的目标架构，不代表当前代码已经完全满足。

---

## 一、目标

1. 在当前分层架构基础上实现无感 FOC。
2. 建立通用低速到高速切换策略，由用户配置低速/高速 source 对。
3. 明确区分 **Source（角度/速度数据来源）** 与 **Control Policy（控制量生成策略）**，避免把数据来源和 `iq_target` / `electrical_phase_angle` 的控制职责混在同一模块中。
4. 取消旧 `STARTUP phase` 的特殊 L1 路由。强拖不是独立顶层 phase，而是低速域的一种 source + control policy 组合。
5. 将齿槽标定与重初始化定义为 `control_phase` 下的 PWM ISR 特殊输出流程，而不是 NORMAL 标准双线流程外的补丁。
6. 明确 LS 层编译期约束与运行时初始化流程的边界，按不同 source 组合初始化 Source Manager 和 Control Policy。
7. 明确初始化阶段的传感器采样、电流零偏采样、电角度/方向/极对数标定和初始化校验门控，保证不同 source 组合下初始化行为可判定。

---

## 二、核心术语与边界

### 2.1 Source 与 Estimator

`Source` 是控制系统使用的角度/速度数据来源。它输出机械角、电角度、机械速度、电速度、有效性、收敛状态和置信度。

`Estimator` 是 Source 的一种实现方式，用于描述由传感器或观测算法得到角度/速度的模块。Encoder、SMO、HFI 可以保留 `estim_*` 命名；OpenLoop 不叫 estimator，而叫 **OpenLoop Angle Source**。

| Source | 实现类型 | 数据来源 | 典型速度域 |
|--------|----------|----------|------------|
| Encoder | Estimator / Sensor source | 物理编码器角度 | 全速域或低速域 |
| SMO | Estimator | 电流、电压、电机模型、PLL | 中高速 |
| HFI | Estimator（暂不实现具体算法） | 高频注入响应 | 零速/低速 |
| OpenLoop | Angle source | 虚拟角度积分器 | 零速/低速强拖 |
| Flux | Estimator（预留） | 磁链观测 | 中高速 |

### 2.2 Control Policy

`Control Policy` 负责产生控制量，尤其是 `iq_target` 和必要的控制状态。它只消费已发布的 source 数据，不决定 source 从哪里来。

典型 policy：

| Policy | 职责 |
|--------|------|
| Speed outer-loop policy | 根据目标速度与 active source 角度/速度生成 `iq_target` |
| Speed-angle outer-loop policy | 根据目标角度、累积角、速度过渡生成 `iq_target` |
| OpenLoop low-speed policy | 强拖低速域专用，按自身 ramp/current 策略生成 `iq_target` |
| Cogging compensation policy | 在 Encoder source 活跃且服务允许时，对 `iq_target` 叠加补偿 |
| Current-loop policy | PWM ISR 内根据 `iq_target`、电流采样和电角度生成 `ud/uq` |

**边界规则**：

- Source 只产生角度/速度数据，不产生 `iq_target`。
- Control Policy 只产生控制量，不选择 active source。
- Source Manager / 数据选择器只在 PWM ISR 的 NORMAL 标准流程中运行，负责选择和发布当前唯一 source，不实现速度环/位置环/强拖电流策略。
- NORMAL 下，数据选择器输出的是唯一 active source view，不是单个角度变量。`electrical_phase_angle` 只是该 view 面向 PWM 快线的派生快路径字段。
- NORMAL 下，电流环和 SVPWM 只消费已发布的唯一 active source view / `electrical_phase_angle`，不自行选择角度来源。

### 2.3 Active Source View

数据选择器必须向后级发布一个结构体级视图，而不是只发布 `electrical_phase_angle`。原因：

- 外环需要机械角、机械速度、有效性和 source id。
- 速度/位置控制需要累积角或可用于累积角维护的历史输入。
- Source Manager 需要 source state、confidence、收敛状态和切换状态。
- 遥测和调试需要同时观察 source 数据与 control policy 输出。
- 后续滤波、相位补偿、速度估计或状态检测可能需要历史值和参数，不能把 source 输出压扁成单个角度。

建议的 active source view 语义如下，具体字段可在实现时按现有 `foc_est_state_t` 兼容演进：

```c
typedef struct {
    uint8_t source;
    uint8_t valid;
    uint8_t state;
    float confidence;

    float mech_angle_rad;
    float mech_angle_accum_rad;
    float mech_speed_rad_s;

    float elec_angle_rad;
    float elec_speed_rad_s;

    /* 可选：用于 source 级滤波/补偿/诊断的历史值、时间戳或质量指标 */
} foc_active_source_state_t;
```

`motor->electrical_phase_angle` 可以继续存在。NORMAL 标准流程中，它只是 `active_source_state.elec_angle_rad` 的快速消费镜像；特殊输出流程中，它属于对应特殊算法的临时输出角度，不代表 active source。

### 2.4 PWM 输出流程状态

`control_phase` 不只影响 Control ISR 的 L1 路由，也决定 PWM ISR 当前执行的输出算法流程：

| control_phase | PWM ISR 输出流程 | Source Manager | 普通电流环 | 输出写入来源 |
|---------------|------------------|----------------|------------|--------------|
| NORMAL | 标准 FOC 快线流程 | 运行并发布 active source view | 运行 | Source Manager 发布角度，Current Loop 写 `ud/uq` |
| COGGING_CALIB | 齿槽标定特殊流程 | 不发布 active source view | 不运行 | 齿槽标定状态机产生特殊开环输出状态，由 PWM 输出流程应用 |
| REINIT | 重初始化特殊流程 | 不发布 active source view | 不运行 | 重初始化状态机产生对齐/零输出/方向扫描状态，由 PWM 输出流程应用 |

因此，“数据选择器只在 PWM ISR 中运行”更精确地说是：只在 `control_phase == NORMAL` 的 PWM ISR 标准流程中运行并发布 active source view。特殊输出流程同样属于 PWM ISR 架构，但不允许标准 Source Manager、普通外环或普通电流环覆盖它们的输出。

特殊模式可以在 Control ISR 中推进低频状态机，但 Control ISR 不应直接成为最终 PWM 仲裁点。目标架构中，Control ISR 只更新特殊模式私有状态或特殊输出状态；PWM ISR 根据 `control_phase` 选择对应输出流程，并在该流程内应用输出。

---

## 三、总体架构

### 3.1 双线三层模型

```
                       ┌───────────────────────────────┐
                       │        foc_motor_t 共享状态      │
                       │ active_source_state             │
                       │ electrical_phase_angle           │
                       │ iq_target / ud / uq              │
                       │ sensor / sensor_fast             │
                       │ source/control private states    │
                       └───────────────────────────────┘

PWM ISR 快线：
  control_phase 路由输出流程
    NORMAL 标准流程：
      Source Layer
        运行高频 source：SMO/HFI/OpenLoopAngle/[FAST Encoder]
      Manager / Selector Layer
        读取 source 状态、速度阈值、收敛状态、滞回消抖
        决定 active source / control region
        同步 encoder_services
        发布 active_source_state
        同步 electrical_phase_angle 快路径字段
      Control Layer
        电流环读取 iq_target + electrical_phase_angle + sensor_fast
        输出 ud/uq
        SVPWM 应用输出
    COGGING_CALIB / REINIT 特殊流程：
      跳过标准 Source Manager 发布
      跳过普通电流环输出
      消费对应特殊状态机的输出状态
      应用标定/重初始化专用 PWM 输出

Control ISR 慢线：
  Source Layer
    运行慢速 sensor 同步：[SLOW Encoder]、VBUS、电流快照同步
  Manager / Selector Layer
    不运行数据选择器，不切换 source，不发布 active source
    只读取 PWM ISR 已发布的 active_source_state / control_region / encoder_services
  Control Layer
    根据 active_source_state、control region、用户目标运行对应 Control Policy
    输出 iq_target
    NORMAL 下生成标准 iq_target
    COGGING_CALIB / REINIT 下推进特殊状态机
```

该模型不是“Source 链”和“Control 链”各自独立跑完，而是 PWM ISR 与 Control ISR 两条执行线在三层上交叉协作：

- PWM ISR 负责按 `control_phase` 选择输出流程；NORMAL 下负责高频 source 更新、source 切换决策、active source 发布、电流环和 SVPWM；特殊模式下负责执行对应特殊输出流程。
- Control ISR 负责低频 sensor 同步；NORMAL 下读取已发布 source view、control policy 选择与 `iq_target` 生成；特殊模式下推进标定/重初始化状态机并更新特殊输出状态。
- `active_source_state` 是两条执行线之间的标准 source 数据视图。
- `electrical_phase_angle` 是从 `active_source_state` 派生的 PWM 快线字段。
- `iq_target` 是 Control ISR 慢线输出给 PWM ISR 快线消费的控制量字段。
- OpenLoop 的角度源状态机和低速控制策略状态机逻辑上配套，但实现上必须分离，分别归属 Source Layer 和 Control Layer。

### 3.2 配置场景

| 场景 | 低速 Source | 高速 Source | 低速 Control Policy | 切换策略 |
|------|-------------|-------------|---------------------|----------|
| 强拖到 SMO | OpenLoop | SMO | OpenLoop low-speed policy | 速度跨阈值 + SMO 收敛 |
| HFI 到 SMO | HFI | SMO | 普通速度外环或 HFI 配套 policy | 收敛后滞回 |
| Encoder 到 SMO | Encoder | SMO | 普通速度/位置外环 | 收敛后滞回 |
| Encoder 全速域 | Encoder | None | 普通速度/位置外环 | 不切换 |
| HFI 全速域 | HFI | None | 普通速度外环或 HFI 配套 policy | 不切换 |

强拖到 SMO 场景下，低速阶段由 OpenLoop low-speed policy 产生 `iq_target`，OpenLoop Angle Source 产生旋转坐标系角度；PWM ISR 中的 Source Manager 将 OpenLoop source view 发布为 active source view。SMO 在后台预收敛。切换条件满足后，PWM ISR Source Manager 发布 SMO source view，Control ISR 中的 Control Policy Selector 读取新 view 后转入普通速度外环。

---

## 四、双 ISR 控制流程

### 4.1 PWM ISR 快线（数据选择器 + 电流环）

PWM ISR 是唯一的输出流程仲裁点。它首先根据 `control_phase` 选择输出流程：NORMAL 进入标准 FOC 快线流程；COGGING_CALIB / REINIT 进入对应特殊输出流程。数据选择器只在 PWM ISR 的 NORMAL 标准流程中运行，确保 source 切换、active view、`electrical_phase_angle` 与电流环消费顺序一致。NORMAL 标准流程中的 PWM ISR 不生成 `iq_target`。

```
公共阶段：输出服务
  SVPWM_InterpolationISR

按 control_phase 选择 PWM 输出流程：

NORMAL 标准流程：
阶段1：采样
  Sensor_ReadCurrent → sensor_fast.current_*
  [FAST encoder] Sensor_ReadEncoder → sensor_fast.mech_angle
  [FAST encoder] sensor_fast → sensor 角度同步

阶段2：Source Layer：运行高频 source
  OpenLoop Angle Source：更新虚拟角度/速度 source snapshot
  SMO/HFI：根据当前电流采样和上一周期实际施加电压更新 source snapshot
  Encoder：根据 sensor 角度更新 source snapshot
  后台 source 同步运行，用于预收敛

阶段3：Manager / Selector Layer：数据选择与发布
  读取 active/standby source snapshot
  根据速度阈值、收敛状态、滞回、消抖决定是否切 source
  同步 active source id、control_region 与 encoder_services
  根据当前 active source 选择对应 source snapshot
  写 active_source_state
  派生写 motor->electrical_phase_angle
  写 source valid/state/confidence/source id

阶段4：Control Layer：电流环
  读取 motor->electrical_phase_angle
  读取 motor->iq_target
  读取 sensor_fast.current_*
  输出 motor->ud / motor->uq / motor->iq_measured

阶段5：SVPWM
  使用 motor->ud / motor->uq / motor->electrical_phase_angle 更新 PWM
  记录本周期实际施加的 applied_ud / applied_uq / applied_electrical_angle

COGGING_CALIB 特殊流程：
  跳过标准 Source Manager 发布
  跳过普通电流环
  读取齿槽标定状态机输出状态
  应用齿槽标定开环电角度和电压/电流等效输出

REINIT 特殊流程：
  跳过标准 Source Manager 发布
  跳过普通电流环
  读取重初始化状态机输出状态
  应用零输出、D 轴对齐、方向扫描等专用输出
```

NORMAL 标准流程中，`electrical_phase_angle` 的写入属于 PWM ISR 中 Source Manager 的“发布唯一 active source view”职责。它不是外环或强拖控制策略的附带输出。切源判断、active view 发布、电角度镜像写入和电流环消费在同一高优先级 ISR 内顺序完成，避免快慢线交叉导致的数据混叠或重复作用。

COGGING_CALIB / REINIT 特殊流程中，`electrical_phase_angle` 可以由对应特殊输出流程写入，但该写入不代表 active source 发布，也不更新 `active_source_state`。特殊流程的角度和输出只服务于标定/重初始化算法本身。

SMO/HFI 类观测 source 采用一周期延迟的电压输入：本周期 source update 使用当前电流采样 `i[k]` 和上一周期实际施加的 `applied_udq/angle[k-1]`，电流环在本周期后半段生成 `ud/uq[k]`，SVPWM 应用后记录为下一周期观测输入。这样避免 observer 与 current loop 形成同周期代数环，也更符合 PWM 采样看到上一输出作用结果的离散时序。

### 4.2 Control ISR 慢线（已发布数据消费 + 控制量生成）

Control ISR 是慢速传感器同步、已发布 source view 消费、control policy 选择、控制量生成和特殊状态机推进路径。它不运行数据选择器，不切 source，不写 active source，不发布 NORMAL active-source `electrical_phase_angle`，也不直接运行 PWM 快线普通电流环。

```
阶段1：Source Layer：慢速传感器同步与有效性检查
  [SLOW encoder] Sensor_ReadEncoder → sensor.mech_angle
  Sensor_ReadVBUS
  Sensor_SyncCurrentSnapshot → sensor.current_*
  检查 adc_valid 和必要的 encoder_valid

阶段2：Manager / Selector Layer：读取已发布状态
  读取 PWM ISR 已发布的 active_source_state
  读取 PWM ISR 已更新的 control_region 与 encoder_services
  不执行 source swap，不写 active source，不发布 electrical_phase_angle

阶段3：Control Layer：按 control_phase 路由
  NORMAL：
    Control Policy Selector 根据 active source + control region 选择策略
    OpenLoop active → OpenLoop low-speed policy 生成 iq_target
    Encoder/HFI/SMO active → 普通速度/位置外环生成 iq_target
    齿槽补偿在允许时叠加 iq offset

  COGGING_CALIB：
    入口检查 encoder_services.calib_available
    直接读 sensor.mech_angle_rad
    不读 active_source_state
    不运行普通外环或强拖 policy
    推进齿槽标定状态机
    更新齿槽标定特殊输出状态

  REINIT：
    入口检查 encoder_services.reinit_available
    直接读 sensor.mech_angle_rad
    不读 active_source_state
    不运行普通外环或强拖 policy
    推进重初始化状态机
    更新重初始化特殊输出状态
```

### 4.3 共享字段归属

| 字段 | 写入方 | 读取方 | 说明 |
|------|--------|--------|------|
| source snapshots | 各 Source | Source Manager | 每个 source 只写自己的状态 |
| active_source_state / `est_state` 等价工作区 | PWM ISR Source Manager 发布阶段 | Control Policy、遥测、服务状态 | 名称可沿用 `est_state`，语义应改为结构体级 active source view |
| `electrical_phase_angle` | NORMAL：PWM ISR Source Manager 发布阶段；特殊流程：PWM ISR 特殊输出流程 | 电流环、SVPWM、特殊输出流程 | NORMAL 下是 active source view 的电角度快路径镜像；特殊模式下不代表 active source |
| `iq_target` | Control Policy | PWM ISR 电流环 | Source 不写 |
| `ud/uq` | NORMAL：PWM ISR 电流环；特殊流程：PWM ISR 特殊输出流程 | SMO、SVPWM | SMO 只读电压状态；特殊流程写入不属于普通电流环 |
| `sensor` / `sensor_fast` | L3 Sensor 路径 | Source、Control Policy、标定/重初始化 | 标定/重初始化始终读物理 `sensor` |
| source 切换状态 / control region | PWM ISR Source Manager | Source Manager、Control Policy Selector、遥测 | L1 / Control ISR 不直接写 |
| `encoder_services` | PWM ISR Source Manager | 协议、标定/重初始化入口、齿槽补偿 | 绑定 active source |
| applied output snapshot | PWM ISR SVPWM 应用后 | SMO/HFI source | 上一周期实际施加的 `ud/uq/electrical_angle`，用于下一周期观测器输入 |
| 特殊输出状态 | COGGING_CALIB / REINIT 状态机 | PWM ISR 特殊输出流程 | 只在对应 `control_phase` 有效，不参与 active source 发布；不是协议 command |

---

## 五、模块职责

### 5.1 Source 相关模块

| 模块 | 当前位置建议 | 修正后职责 |
|------|--------------|------------|
| Source Manager | 由 `foc_ctrl_transition.c` 演进或新建 `foc_ctrl_source_mgr.c` | 在 PWM ISR 的 NORMAL 标准流程中运行 source 决策、切换 active source、发布 active source view / `electrical_phase_angle`、维护 `encoder_services` |
| Encoder Source / Estimator | `foc_ctrl_estim_encoder.c` | 从 `sensor` 读取物理编码器角度，输出 source snapshot |
| SMO Source / Estimator | `foc_ctrl_estim_smo.c` | 从电流/电压模型估计角度速度，输出 source snapshot 和收敛状态 |
| HFI Source / Estimator | `foc_ctrl_estim_hfi.c` | 预留 HFI source 行为 |
| OpenLoop Angle Source | 从 `foc_ctrl_startup_openloop.c` 拆出 | 只维护虚拟角度/速度 ramp，输出 source snapshot，不写 `iq_target` |

### 5.2 Control Policy 相关模块

| 模块 | 修正后职责 |
|------|------------|
| OpenLoop low-speed policy | 低速强拖控制策略，依据 openloop 控制状态生成 `iq_target`，不生成 source |
| OuterLoop | Encoder/HFI/SMO 等闭环 source 下的速度/位置外环，生成 `iq_target` |
| Cogging Compensation | 仅在 Encoder source 活跃且服务允许时叠加 `iq_target` offset |
| Current Loop | NORMAL 标准流程中消费 `iq_target` 和已发布 `electrical_phase_angle`，生成 `ud/uq` |
| Actuation / SVPWM | 把 NORMAL 的 `ud/uq/electrical_phase_angle` 或特殊输出流程的专用输出状态转为三相 PWM |
| Control Policy Selector | 根据 active source、control region、control mode 选择本周期 NORMAL 下的控制策略 |
| Special Output Flow | 在 COGGING_CALIB / REINIT 下消费特殊状态机输出状态，应用专用 PWM 输出，不运行标准 Source Manager 和普通电流环 |

### 5.3 旧模块处理

- Bridge / `ctrl_input`：目标架构中移除。若为了渐进迁移临时保留，只能作为兼容视图，不能成为新的数据真相。
- Startup 模块：不再作为 L1 phase。其可复用逻辑拆到 OpenLoop Angle Source 和 OpenLoop Low-Speed Policy。
- Estimator Selector：职责收窄为 source 实现注册/初始化辅助，不负责运行时 active source 决策。

---

## 六、Source Manager 行为

### 6.1 初始化

```
FOC_SourceMgr_Init(motor, LOW_SOURCE, HIGH_SOURCE):
  初始化所有编译启用 source 的私有状态
  active_source = LOW_SOURCE
  standby_source = HIGH_SOURCE 或 NONE
  source_state[LOW_SOURCE].state = INIT
  source_state[HIGH_SOURCE].state = INIT
  control_region = LOW
  encoder_services 根据 active_source 初始化
```

初始化函数只设置初始静态状态，不执行周期 source 选择。启动后的 source 切换、`control_region` 更新和 `encoder_services` 同步均由 PWM ISR 中的数据选择器完成。

### 6.2 PWM ISR 周期运行、切换与发布

数据选择器的周期逻辑只在 `control_phase == NORMAL` 的 PWM ISR 标准流程中运行。原因是 PWM ISR 是电流环和 SVPWM 的直接上游，在这里完成 source 判断、切换和发布，可以保证同一周期内后级控制只看到一个 active source view。

```
FOC_SourceMgr_RunPwmStep(motor):
  运行/读取各 source snapshot
  active = 当前 active source snapshot
  standby = 当前 standby source snapshot

  low → high:
    abs(active.mech_speed_rad_s) > threshold_high 持续 N 周期
    且 standby.state >= CONVERGING
    → 切换 active/standby
    → control_region = HIGH
    → 同步 encoder_services

  high → low:
    abs(active.mech_speed_rad_s) < threshold_low 持续 N 周期
    或 active.state == DIVERGED
    → 切换 active/standby
    → control_region = LOW
    → 同步 encoder_services

  发布：
    active snapshot → active_source_state
    active.elec_angle_rad → motor->electrical_phase_angle
```

Source 切换是瞬时切换，不做角度加权混合。Encoder、SMO、HFI、OpenLoop 的角度模型不同，加权平均没有统一物理意义。

`control_region` 是 PWM ISR Source Manager 给 Control ISR Control Policy Selector 的运行区间提示，不是 L1 `control_phase`。它表达当前应使用低速策略还是高速策略，但不直接生成控制量。

### 6.3 Control ISR 只读规则

NORMAL 下，Control ISR 不调用 source manager 的切换/发布函数。它只读取上一个 PWM ISR 已经发布完成的：

- `active_source_state`
- `control_region`
- `encoder_services`
- source 状态遥测字段

这意味着 Control ISR 生成的 `iq_target` 天然作用于后续 PWM 周期，而不是反向改变当前 PWM 周期的数据来源。

COGGING_CALIB / REINIT 下，Control ISR 可以推进对应特殊状态机，但仍不调用 Source Manager，不发布 active source，不运行普通 Control Policy。特殊状态机产生的输出状态由 PWM ISR 特殊输出流程消费。

---

## 七、LS 配置约束与初始化流程

### 7.1 LS 层编译期约束

编译期配置校验不属于运行时初始化流程。它应保留在 LS 配置层，由 `foc_config.h` 的派生宏和 `foc_compile_limits.h` 的 `#error` 约束完成。

LS 层负责：

- 根据 `FOC_CONTROL_LOW_SOURCE` / `FOC_CONTROL_HIGH_SOURCE` 派生各 source、policy、transition 的启用宏。
- 校验 source pair 与硬件能力是否匹配。
- 校验无感 source 对电流采样、电机参数、启动 source 的依赖。
- 校验 Encoder 专属服务对编码器硬件的依赖。
- 用 `#error` 阻断非法组合，不把非法配置留到运行时初始化处理。

典型约束：

- `LOW_SOURCE/HIGH_SOURCE == ENCODER` 时，必须启用编码器硬件。
- `LOW_SOURCE/HIGH_SOURCE == SMO` 时，必须启用 SMO source，并满足电流采样和电机参数约束。
- `LOW_SOURCE/HIGH_SOURCE == HFI` 时，必须启用 HFI source，并满足注入和采样依赖。
- `LOW_SOURCE/HIGH_SOURCE == OPENLOOP` 时，必须启用 OpenLoop Angle Source 与 OpenLoop Low-Speed Policy。
- 无 Encoder 的 SMO 高速 source 必须配置合法低速启动 source，例如 OpenLoop 或 HFI。
- COGGING_CALIB / COGGING_COMP / REINIT 依赖物理 Encoder 服务时，必须有编码器硬件支持。

### 7.2 运行时初始化总流程

运行时初始化只处理已经通过 LS 编译期约束的合法组合，不重复做完整配置合法性判断。

```
FOC_App_Init:
  保持 control tick / PWM update / runtime interrupt 全部关闭
  平台 runtime、通信、调度器、队列、debug stream 初始化
  Sensor / SVPWM / PWM callback 初始化
  执行初始化传感器采样准备：
    电流零偏采样 Sensor_SetZeroOffset
    初始 VBUS 采样
    若编码器硬件启用，读取初始机械角
  motor 基础字段、PID、目标值、fault/state 清零
  载入运行默认配置与补偿默认状态
  按配置和硬件能力执行 init-time calibration
  初始化所有编译启用的 source 私有状态
  初始化所有编译启用的 Control Policy 私有状态
  初始化 COGGING_CALIB / REINIT 特殊状态机为 idle
  Source Manager 根据 LOW_SOURCE/HIGH_SOURCE 建立 active/standby/control_region
  control_phase = NORMAL
  执行初始化校验并生成 init_check/init_fail
  current_loop_ready 根据初始化校验、初始 source 和采样状态置位

FOC_App_Start:
  仅在 FOC_App_Init 完成后调用
  启动 control tick source
  开启 control runtime interrupt / PWM update ISR
```

初始化期间所有控制 ISR、PWM update ISR 和调度 ISR 必须保持关闭。初始化只允许配置硬件、采样零偏、做阻塞式 init-time calibration、初始化状态结构和建立初始 source/control 状态。PWM 硬件可以初始化到安全零输出状态，但不得在初始化完成前触发控制 ISR 流程。

目标架构中，`FOC_CONTROL_LOW_SOURCE == OPENLOOP` 不再把初始 `control_phase` 设为 STARTUP。强拖低速启动应表现为：

```text
control_phase = NORMAL
active_source = OPENLOOP
control_region = LOW
PWM ISR NORMAL 标准流程运行 OpenLoop Angle Source
Control ISR NORMAL 流程选择 OpenLoop Low-Speed Policy 生成 iq_target
```

### 7.3 source 组合初始化矩阵

| 配置场景 | 初始 control_phase | 初始 active source | 初始 standby source | 初始 control_region | 初始 Control Policy | init-time calibration |
|----------|--------------------|--------------------|---------------------|----------------------|---------------------|-----------------------|
| Encoder 全速域 | NORMAL | Encoder | None | LOW 或 FULL | 普通速度/位置外环 | 可执行 encoder 电零点/方向/极对数标定 |
| OpenLoop → SMO，无 Encoder | NORMAL | OpenLoop | SMO | LOW | OpenLoop low-speed policy | 不执行 encoder 标定；极对数和方向必须来自 LS 默认参数 |
| OpenLoop → SMO，有 Encoder | NORMAL | OpenLoop | SMO | LOW | OpenLoop low-speed policy | 可执行 encoder 标定，但 active source 仍从 OpenLoop 开始 |
| HFI → SMO，无 Encoder | NORMAL | HFI | SMO | LOW | HFI 配套 policy 或普通速度外环 | 不执行 encoder 标定；极对数和方向必须来自 LS 默认参数 |
| HFI → SMO，有 Encoder | NORMAL | HFI | SMO | LOW | HFI 配套 policy 或普通速度外环 | 可执行 encoder 标定，但 active source 仍从 HFI 开始 |
| Encoder → SMO | NORMAL | Encoder | SMO | LOW | 普通速度/位置外环 | 可执行 encoder 电零点/方向/极对数标定 |
| HFI 全速域 | NORMAL | HFI | None | LOW 或 FULL | HFI 配套 policy 或普通速度外环 | 按是否有 Encoder 决定是否可执行 encoder 标定 |

`FULL` 只是“不需要低高切换”的实现语义，可用 `LOW` 加 `standby=None` 表达，具体枚举在实现时确定。

### 7.4 初始化采样与标定行为

初始化阶段分成三类动作，不能混成同一条控制流程：

1. 基础传感器准备
   - `Sensor_SetZeroOffset` 用于电流 ADC 零偏采样，属于电流传感器路径。
   - 初始 VBUS 采样用于供电有效性和电压限幅。
   - 初始 Encoder 采样只在编码器硬件启用时执行，用于 sensor 初始状态和可选标定。

2. init-time calibration
   - 用于上电阶段补齐 `mech_angle_at_elec_zero_rad`、`direction`、`pole_pairs`。
   - 该流程依赖物理 Encoder 机械角读取。
   - 该流程可以使用 direct-output 锁定电角度采样机械角，属于初始化专用阻塞式特殊过程。
   - 它不属于 NORMAL Source Manager 发布链，也不属于 REINIT 非阻塞特殊流程。

3. 初始化校验
   - 校验基础通信、PWM、传感器、VBUS、电机参数和 source 启动条件。
   - 校验结果写入 `init_check_mask/init_fail_mask/last_fault_code/system_running`。
   - 只有校验通过且初始 source 可用时，才允许 `current_loop_ready` 进入可运行状态。

init-time calibration 的行为门控：

- 若 `FOC_INIT_CALIBRATION_ENABLE == DISABLE`，初始化不执行电零点/方向/极对数标定，直接使用 LS 默认参数。
- 若编码器硬件不可用，初始化不执行依赖物理 Encoder 的标定；需要 LS 默认参数满足对应 source 的启动要求。
- 若 `mech_angle_at_elec_zero_rad` 已定义，则不重新估计电零点。
- 若 `direction` 已定义，则不重新估计方向。
- 若 `pole_pairs` 已定义，则不重新估计极对数。
- 若任何需要标定的字段无法成功标定，应保持 undefined，并由初始化校验决定是否阻断启动。

### 7.5 初始化校验门控

初始化校验应按控制组合决定必需项，而不是固定要求所有能力都存在：

| 检查项 | 适用范围 | 失败影响 |
|--------|----------|----------|
| PWM / runtime / protocol / scheduler | 所有组合 | 阻断启动 |
| VBUS 有效 | 所有需要输出的组合 | 阻断启动或进入 fault |
| 电流 ADC 有效 | SMO/HFI/闭环电流环需要电流采样时 | 阻断 `current_loop_ready` |
| Encoder 有效 | active/standby source 包含 Encoder，或启用 Encoder 专属服务时 | 阻断对应 source/service；若初始 active 为 Encoder 则阻断启动 |
| `pole_pairs` 已定义 | OpenLoop、SMO、HFI、Encoder 电角度转换均需要 | 阻断需要电角度转换的 source |
| `direction` 已定义 | OpenLoop、Encoder 电角度转换、齿槽标定等需要 | 阻断依赖方向的 source/policy/service |
| `mech_angle_at_elec_zero_rad` 已定义 | Encoder source 和 Encoder 服务需要机械角到电角度转换时 | 阻断 Encoder source/service |

无 Encoder 的无感组合不能因为 `sensor.encoder_valid == 0` 失败；但必须保证无感启动所需的 `pole_pairs`、`direction` 和电流采样参数可用。

有 Encoder 硬件但 active source 不是 Encoder 时，初始化可以读取 Encoder 并执行可选标定；标定结果只更新 motor 参数和 Encoder 服务可用性，不改变初始 active source。

### 7.6 初始化后的职责边界

- L1 初始化负责调用各模块 Init，并设置初始 `control_phase = NORMAL`。
- L1 初始化与启动必须分离：`FOC_App_Init` 不启动控制定时器和运行时中断，`FOC_App_Start` 才启动。
- Source Manager 初始化负责建立 active/standby source 和初始 `control_region`。
- Source Manager 周期发布仍只发生在 PWM ISR NORMAL 标准流程。
- Control Policy 初始化只清状态，不选择 source。
- COGGING_CALIB / REINIT 初始化只把特殊状态机置为 idle，不自动进入特殊流程。
- 初始化阶段的阻塞式电角度/方向标定属于现有兼容路径；目标架构中若保留，应标注为 init-time special procedure，不属于 NORMAL source 发布链，也不属于非阻塞 REINIT。

---

## 八、OpenLoop 强拖设计

### 8.1 当前问题

当前 `foc_ctrl_startup_openloop.c` 同时做三件互相独立的事：

1. 生成虚拟角度。
2. 直接写 `electrical_phase_angle`。
3. 直接写 `iq_target`。

这把“角度 source”和“控制量生成 policy”混在一起，导致旧 STARTUP 路径绕过统一 source/control 架构。

### 8.2 修正方案

OpenLoop 拆成两个状态机模块。它们在“强拖低速运行”这个场景中协作，但实现上不互相包含，不共享职责。

**OpenLoop Angle Source**：

- 持有 `virtual_angle_rad`、`virtual_speed_rad_s`、ramp rate、方向等角度发生器状态。
- 根据目标速度和 ramp 限制更新虚拟速度。
- 积分得到虚拟机械角/电角度。
- 输出 source snapshot：`source=OPENLOOP`、角度、速度、`valid=1`、`state=LOCKED`、`confidence=1.0`。
- 不写 `iq_target`。
- 不直接写 `electrical_phase_angle`，该字段只由 Source Manager 发布 active source 时写入。

**OpenLoop Low-Speed Control Policy**：

- 持有强拖电流参考、限幅、必要的启动/运行状态。
- 由 Control Policy Selector 在 active source 为 OpenLoop 且 control_region 为 LOW 时选择运行。
- 生成 `iq_target`。
- 不生成角度，不切 source，不写 source 状态。

### 8.3 强拖到 SMO 的运行顺序

```
PWM ISR:
  OpenLoop Angle Source 更新虚拟角度/速度
  SMO Source 后台运行并更新收敛状态
  Source Manager 判断速度阈值与 SMO 收敛
  如条件满足，active source 从 OpenLoop 切到 SMO，control_region 从 LOW 切到 HIGH
  Source Manager 发布当前 active source view 到 active_source_state
  Source Manager 派生写 electrical_phase_angle
  电流环使用 electrical_phase_angle + iq_target 输出 ud/uq

Control ISR:
  读取 PWM ISR 已发布的 active_source_state / control_region
  Control Policy Selector 选择 OpenLoop Low-Speed Policy
  OpenLoop Low-Speed Policy 生成 iq_target

切换条件满足:
  后续 Control Policy Selector 选择普通速度外环生成 iq_target
```

强拖不是独立 `control_phase`，也不是 estimator。它在逻辑场景上需要 OpenLoop Angle Source 与 OpenLoop Low-Speed Control Policy 协作；实现上两者应作为分离状态机，由 Source Manager 和 Control Policy Selector 分别调度。

---

## 九、control_phase、特殊输出流程与有感服务

### 9.1 control_phase 定义

```c
typedef enum {
    FOC_CONTROL_PHASE_NORMAL        = 0U,
    FOC_CONTROL_PHASE_COGGING_CALIB = 1U,
    FOC_CONTROL_PHASE_REINIT        = 2U
} foc_control_phase_t;
```

`control_phase` 表示当前顶层控制模式，并决定 Control ISR 的状态机推进入口以及 PWM ISR 的输出流程。低速/高速、OpenLoop/SMO/Encoder/HFI 切换不通过 `control_phase` 表示，只属于 NORMAL 标准流程内部的 source/control state。

### 9.2 特殊输出流程

COGGING_CALIB 和 REINIT 是同一顶层架构内的特殊 PWM 输出流程，不是外部补丁，也不是普通 Source / Control Policy：

- 特殊模式由 `control_phase` 进入，L1 只负责设置和路由 phase。
- PWM ISR 根据 `control_phase` 选择特殊输出流程，并跳过 NORMAL 标准 Source Manager、普通外环和普通电流环。
- Control ISR 可以推进特殊模式的低频状态机，并更新特殊输出状态。
- 特殊流程可以写 `electrical_phase_angle`、`ud/uq` 或 direct duty，但这些写入只属于当前特殊算法，不代表 active source view 发布。
- 特殊流程结束后切回 NORMAL，下一次 NORMAL PWM ISR 再恢复 Source Manager 发布和普通电流环输出。

特殊输出状态不是协议 command。协议 command 只表示外部请求，例如请求进入 COGGING_CALIB 或 REINIT；特殊状态机接受请求后，在自身私有状态中推进阶段，并把当前应由 PWM ISR 应用的输出投影为 `foc_phase_output_state_t` 一类状态结构。

建议的特殊输出状态语义：

```c
typedef enum {
    FOC_PHASE_OUTPUT_IDLE = 0U,
    FOC_PHASE_OUTPUT_ZERO,
    FOC_PHASE_OUTPUT_DQ_VOLTAGE_ANGLE,
    FOC_PHASE_OUTPUT_DIRECT_DUTY
} foc_phase_output_type_t;

typedef struct {
    uint8_t phase;          /* foc_control_phase_t */
    uint8_t type;           /* foc_phase_output_type_t */
    uint8_t valid;
    uint8_t state_id;       /* 所属特殊状态机内部阶段，用于遥测/一致性检查 */

    float electrical_angle_rad;
    float ud;
    float uq;

    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} foc_phase_output_state_t;
```

具体字段可随状态机实现收敛，但命名和语义必须保持为“状态”，不得命名为 command，也不得与协议层请求结构复用。

齿槽标定特殊流程：

- 必须依赖物理 Encoder 服务可用。
- 直接读取 `sensor.mech_angle_rad` 作为标定采样数据。
- 使用自身开环角度/速度推进标定轨迹。
- 生成齿槽标定专用输出状态，由 PWM ISR 特殊流程应用。
- 不读取 `active_source_state`，不切 source，不生成 NORMAL `iq_target`。

重初始化特殊流程：

- 必须依赖物理 Encoder 服务可用。
- 直接读取 `sensor.mech_angle_rad` 和必要的电流/电压采样。
- 执行零输出、D 轴对齐、方向扫描、参数应用等专用步骤。
- 生成重初始化专用输出状态，由 PWM ISR 特殊流程应用。
- 不读取 `active_source_state`，不切 source，不生成 NORMAL `iq_target`。

### 9.3 有感编码器服务状态

标定、重初始化、齿槽补偿是物理 Encoder source 的专属服务。

```c
typedef struct {
    uint8_t comp_available;
    uint8_t comp_active;
    uint8_t calib_available;
    uint8_t reinit_available;
} foc_encoder_services_state_t;
```

Source Manager 是 `encoder_services` 的唯一写入入口：

- active source 为 Encoder：`calib_available=1`，`reinit_available=1`，`comp_available` 由 LUT 可用性决定，`comp_active` 不自动恢复。
- active source 非 Encoder：四个标志全部清零。

标定/重初始化始终直接读 `sensor.mech_angle_rad`，不读 active source。即使 active source 曾由 Encoder 写入，也不把 source snapshot 作为标定数据源。

最终边界：

- runtime `encoder_services` 绑定 active physical Encoder source。
- active source 不是 Encoder 时，runtime 齿槽补偿、齿槽标定入口、重初始化入口均不可用。
- init-time calibration 是初始化专用例外，只看物理 Encoder 硬件和配置，不看 active source。
- COGGING_CALIB / REINIT 一旦进入，运行中仍直接读取物理 `sensor.mech_angle_rad`，但入口可用性由进入前的 `encoder_services` 门控决定。

---

## 十、L1 编排边界

L1 负责：

- 初始化系统、motor、队列、调度器和平台 API。
- 调度 Service / Monitor / Control。
- 在 `control_phase` 上路由 NORMAL、COGGING_CALIB、REINIT。
- 让 PWM ISR 和 Control ISR 都能根据 `control_phase` 进入对应标准或特殊流程。
- 在协议请求后设置 phase 或 dirty 标志。

L1 不负责：

- 不直接切换 active source。
- 不直接操作 source 函数指针或 source 状态。
- 不知道 “OpenLoop 到 SMO” 的特殊路径。
- 不生成 `iq_target`。
- 不发布 `electrical_phase_angle`。
- 不直接仲裁 PWM 输出；最终输出流程由 PWM ISR 根据 `control_phase` 执行。

---

## 十一、协议接入、遥测与可见性

### 11.1 协议接入边界

原有协议体系可以继续接入目标架构，但协议层只写请求、配置、目标和使能状态，不直接参与 Source Manager、Control Policy 或 PWM 输出流程。

- `P` 组运行目标继续写目标角度、目标速度、控制模式等 runtime target，由 NORMAL Control Policy 消费。
- `C` 组控制参数继续写 PID、补偿、电流软切换等配置，由对应 policy 或控制模块消费。
- `S` 组状态开关继续写 motor enable、telemetry enable、cogging comp enable 等运行开关。
- `Y` 组系统动作继续写请求或 `control_phase`，例如 COGGING_CALIB / REINIT / fault recovery。
- 协议不得直接写 `active_source_state`、不得直接选择 active source、不得直接写 PWM 输出。
- source pair 仍按编译期配置确定；运行时通过协议动态修改 `LOW_SOURCE/HIGH_SOURCE` 不纳入本轮目标架构。

### 11.2 遥测可见性

- Source 状态是只读系统状态，可通过现有批量读取和遥测通道暴露。
- 建议暴露 active source id、active source state、confidence、control_region、source switch counter。
- `iq_target` 继续作为控制输出状态暴露，用于观察 Control Policy 结果。
- NORMAL 下，`electrical_phase_angle` 可作为 source 发布结果暴露，用于确认电流环消费的唯一角度；特殊模式下应标注为特殊输出角度。
- 不新增协议写入口修改 source 内部状态；source 切换只由 Source Manager 根据配置和运行状态决定。

---

## 十二、验证要求

### 12.1 文档审查

1. 全文不得再把 OpenLoop 描述为 estimator，也不得把全部 source 统称为 estimator。
2. 每处 `iq_target` 的写入职责必须归属 Control Policy。
3. NORMAL 下每处 `electrical_phase_angle` 的写入职责必须归属 PWM ISR Source Manager 发布阶段；COGGING_CALIB / REINIT 下写入必须归属 PWM ISR 特殊输出流程，且不得更新 active source view。
4. 每个场景都必须能明确回答：角度从哪里来、谁发布电角度、谁生成 iq、何时切 source。
5. 特殊模式必须能明确回答：谁推进状态机、谁生成特殊输出状态、PWM ISR 中哪个特殊流程应用输出、哪些标准流程被跳过。
6. 初始化流程必须能明确回答：编译期约束在哪里完成、初始 active source 是谁、初始 standby source 是谁、初始 `control_phase/control_region` 是什么。
7. 初始化标定必须能明确回答：是否需要 Encoder、是否执行 direct-output 阻塞标定、失败后哪些字段保持 undefined、由哪个校验项阻断启动。
8. 无 Encoder 无感组合不得因为 `encoder_valid == 0` 阻断初始化，但必须校验 `pole_pairs/direction` 和电流采样依赖。

### 12.2 后续代码验收

1. 编译通过，0 error，不引入新增 warning。
2. Encoder 全速域路径不回归。
3. OpenLoop 低速控制不再混入 estimator/source 模块。
4. Source 选择和 Control Policy 输出可分别遥测观察。
5. `STARTUP phase` 从目标架构中移除后，L1 仍只负责 phase 路由，不承担 source 切换逻辑。
6. COGGING_CALIB / REINIT 不进入 NORMAL Source Manager 发布链，不运行普通电流环覆盖特殊输出。
7. OpenLoop 低速启动时初始 `control_phase` 为 NORMAL，不再依赖 STARTUP phase。
8. init-time calibration 只在 Encoder 硬件可用且配置允许时执行；无 Encoder 组合依赖 LS 默认 motor 参数启动。
9. `current_loop_ready` 只能在初始化校验和初始 source 可用性满足后置位。

---

## 十三、实施约束

1. 宏裁剪链路声明、定义、调用三者必须一致。
2. 已有 Encoder 全速域路径不得因无感重构退化。
3. 禁止 L2 模块包含 `L1_Orchestration/` 头文件。
4. 禁止 L2 任何模块持有全局实例；实例仍由 L1 分配，L2 通过指针操作。
5. 配置常量收敛在 `foc_cfg_*.h`，禁止在业务 `.c` 散落默认值。
6. 协议裁剪宏不得影响控制算法行为；`FOC_PROTOCOL_ENABLE_*` 只控制协议命令可见性。
7. 每次代码修改后必须编译验证，并同步构建入口与源文件列表。
8. `encoder_services` 的唯一写入入口是 Source Manager。
9. 标定/重初始化始终从 `sensor` 直读物理编码器角度。
10. 编译期约束必须用 `#error` 阻断非法配置组合。
11. 特殊输出流程必须并入 PWM ISR 的 `control_phase` 分支，不能作为绕过顶层架构的隐式 direct-output 补丁。
12. 编译期约束只放在 LS 配置层，不在运行时初始化中散落重复判断。
13. 初始化校验必须按 source 组合门控传感器与电机参数，不能固定要求所有硬件传感器都有效。
