# Changelog

All notable changes to the HYWfoc (何易位FOC) project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.1.0] - 2026-08-07

### Changed
- **L2 传参收窄重构（耦合域显式化，延续 `结构优化重构.md` G1/G3）**：
  - **L3 SVPWM 收窄**：`SVPWM_Init/Update/ApplyDirectDuty/SetRuntimeDutyTarget/InterpolationISR/GetOutput` 首参 `foc_motor_t *` → `svpwm_interp_state_t *`，消除 L3 对 motor 聚合的反向依赖。
  - **Estimator 域收窄**：`FOC_EstimSMO_Step(state, params, sensor, applied, ctrl, active_source, control_region, dt)`、`FOC_EstimSMO_Init(state, params)`；`FOC_EstimHFI_Step/Init(state, ...)`；`foc_ctrl_estim.h` 收敛为纯状态宏层（API 声明转移至各模块 `.h`），删除无实现的 Flux 空声明。
  - **执行输出域收窄**：`FOC_ControlApplyElectricalAngleRuntime/Direct`、`FOC_ControlRecordPhaseOutputDqAngle/Zero`、`FOC_ControlApplyPhaseOutputRuntime`、`FOC_SampleLockedMechanicalAngle` 全部改为 `(ctrl, params, svpwm, applied, alpha_beta, phase_output, ...)` 子结构传参。
  - **SourceMgr 上下文视图**：新增 `foc_source_mgr_ctx_t`（输入 sensor/params/cfg/control_mode + 输入输出 switch_cfg/state/active/ctrl/outer_loop/pids/encoder_services + 条件编译 source 状态），`FOC_SourceMgr_Init/Select/Publish(ctx)`；新增 `foc_source_read_ctx_t` 只读窗，`FOC_SourceMgr_ReadSourceAngle/Speed(ctx, ...)`；删除无调用点的 `FOC_SourceMgr_GetActive`。
  - **配置/补偿域收窄**：`FOC_ControlConfigResetDefault(cfg, soft_switch, comp_status, table, count)`、`FOC_Control_ApplyConfig(params, ctrl, pids, cfg)`、`FOC_ControlApplyCoggingCompensation(status, table, ctrl, angle, speed)`、`FOC_ControlLoadCoggingCompTableQ15(status, table, src, ...)`；删除无调用点的 `FOC_ControlGetCurrentSoftSwitchStatus`。
  - **参数学习收窄**：`FOC_EstimateDirectionAndPolePairs(ctrl, params, svpwm, applied, alpha_beta, ...)`。
  - **include 清理**：openloop/outer_loop/current_loop/compensation/estim_smo/estim_hfi/actuation/cfg/source_mgr/sensor/svpwm 移除 `foc_motor_aggregate.h` 依赖；L2/L3 各模块 `.h` 移除 `foc_motor_t` 前置声明。
- **函数传参排序规范（可读性）**：统一按"本域私有状态 → 跨域可写总线 → 跨域只读输入(const) → 显式 out → 标量(dt 最后)"分组排序。执行输出域（`FOC_ControlApplyElectricalAngleRuntime/Direct`、`FOC_ControlApplyPhaseOutputRuntime`、`FOC_SampleLockedMechanicalAngle`）`params` 移入 const 只读组；`FOC_ControlApplyCoggingCompensation` 改为 `(status, ctrl, table, ...)`；`FOC_Control_ApplyConfig` 改为 `(ctrl, pids, cfg, const params)` 且 `params` 加 const。规范写入 `.clinerules/hywfoc-project-rules.md`。
- **聚合访问权唯一化**：`foc_motor_t` 完整类型仅保留给 L1 编排、L2 Executor（facade）、协议/调试只读链、冷路径状态机（init/标定/重初始化）。`foc_motor_aggregate.h` 物理路径保留在 `L2_Core/`（阶段 1b 上移 L1 需 Executor 上下文视图化后实施，见 `结构优化重构.md`）。
- **平台 API 契约重构（foc_platform_api.h / empty 模板 / GD32 实例同步）**：
  - **契约模型确立**：接口面稳定（全部无条件声明，不随配置宏裁剪）+ 行为自适应（条件功能实现内部按宏适配，关闭时退化为 no-op / 返回 0）+ 三档契约标注（【必须】/【按需】/【可选】），宏组合行为矩阵写入头文件。
  - **参数传递统一**：编译期固定配置由平台实现内部读 LS 宏，不进入接口签名——`FOC_Platform_PWMInit(void)`、`FOC_Platform_SensorInputInit(void)`、`FOC_Platform_ControlTickSourceInit(void)` 统一无参；仅运行时参数（AuxTimer 频率、采样偏移）显式传参。L3 同步简化：`Sensor_Init(void)`、`SVPWM_Init(svpwm)`（插值步数改读 `FOC_PWM_FREQ_KHZ`）。
  - **回调类型统一**：删除 `FOC_Platform_TickCallback_t` / `FOC_Platform_PwmIsrCallback_t` 冗余双类型，统一为单一 `FOC_Platform_IsrCallback_t`（消除 AuxTimer 借用 PWM 回调类型的错位；`FOC_Init_Runtime` 签名同步）。
  - **通信源收敛**：8 个 `CommSource{1..4}_IsFrameReady/ReadFrame` → 1 个 `FOC_Platform_CommSource_ReadFrame(FOC_Platform_CommSourceId_t id, ...)`；删除 4 个无调用者的孤儿接口 `IsFrameReady`（ReadFrame 返回 0 已含"无帧"语义）。
  - **区块重组与描述修正**：`WaitMs`/`MemoryBarrier` 归入 Runtime & Clock（原 PWM/Sensor 区块错位）；`ReadVbusVoltage` 归入 Sensor 区块并移除头文件注释中绑定的 ADC2 平台细节；守卫注释统一 `FOC_PLATFORM_API_H`（原 `FOC_PAL_H` 与结尾注释矛盾）；头文件注释全面中文化。
  - **孤儿 API 删除**：`FOC_Platform_UndervoltageProtect`（L1 欠压状态机从不调用，且其注释声称的调用上下文与 L1 ISR 检测矛盾）、`CommSource*_IsFrameReady`×4。
  - `FOC_Platform_SetControlRuntimeInterrupts` 更名为 `FOC_Platform_SetControlInterruptsEnabled`（消除"Runtime"误导）。
  - 命名修正：`FOC_Platform_AuxTimerId_t`、`FOC_Platform_CommSourceId_t` 枚举化；实例 `CommSource_ReadFrame` switch(id) 映射 USART1/2。
  - ROM：54.90kB → 54.82kB（删 5 个孤儿函数 + 去参）。
- **契约文档化**：`architecture.md` 新增"平台 API 契约"小节（三档 + 宏组合矩阵 + 参数约定）；`foc_platform_api.h` 文件头契约说明。

### Documentation
- `docs/architecture.md` L2 约束区第 5 条更新为"L2 传参规范 + 聚合访问权唯一化"。

### Note
- 平台 API 为稳定契约面，移植新平台时无需关心配置宏组合——接口恒存在，条件功能退化语义已文档化。
- 通信源 0/1 为必须实现（GD32 实例映射 USART1/USART2）；源 2/3 可选（恒返回 0 即可）。

## [2.0.7] - 2026-08-07

### Changed
- **重复算法函数去重**：
  - `FOC_NormalizeDt`（current_loop / outer_loop 双份）→ L3 `Math_NormalizeDt(dt_sec, fallback_dt_sec)`，所有调用点收敛。
  - `wrap_2pi`（SMO）/ `OpenLoop_Wrap2Pi`（OpenLoop）→ 统一使用 L3 既有 `Math_WrapRad`。
  - `ResetPID`（executor）/ `FOC_ResetPIDState`（outer_loop）→ 统一为 `FOC_PIDReset`（归入 `foc_ctrl_cfg` 配置域）。
  - 删除无调用点的向后兼容死代码 `DebugStream_GenerateLine`（含 static 不可重入缓冲），消除不可重入隐患。
- **无逻辑影响的冗余赋值清理**：`FOC_OutputMgr_PollOneSource` 恒返回 0 且调用方忽略，改 `void` 返回。
- **计数/累加溢出防护（不新增配置宏）**：
  - SMO `converge_counter` 达到 `FOC_ESTIM_SMO_DIVERGE_CONSECUTIVE` 后停止递增，消除 uint16 回绕导致收敛状态倒退。
  - SourceMgr `LOW_RECOVERY` 分支无消费的 `switch_counter++` 移除。
  - `control_skip_count` / `protocol_error_count` 饱和到 `UINT32_MAX`；`overflow_count` 饱和到 `UINT8_MAX`。
- **64 位运算清除（32 位平台双字撕裂防护 + ROM 缩减）**：
  - 移除全部 `(double)` 强制转换与 `%f/%.Nf` 格式符，新增 L3 定点格式化工具 `Math_FloatToFixed(value, decimals, ipart, fpart)`，调用方以 `%d.%0Nd` 输出。
  - 涉及：`foc_output_mgr.c`、`foc_ctrl_sens_reinit.c`、`foc_protocol_handler.c`、`foc_protocol_parser.c`、`foc_debug_stream.c`。
  - 全链路不再链接 double 打印库；ROM：57.02kB → 54.96kB（-2.06kB，-3.6%）。
- **参数调整行为收敛（写入契约简化）**：
  - 删除运行时 `cfg_dirty → FOC_Control_ApplyConfig` 链路：`ApplyConfig` 收敛为冷路径专用（仅初始化/重初始化时基于 `max_phase_voltage`/`phase_resistance` 重算 PID 输出限幅）；运行时协议写参数**直写即生效**，无派生重算。
  - 删除死代码/死字段：`foc_motor_state_t.cfg_dirty`、`foc_protocol_frame_result_t.param_changed`（L1 从不消费）、`FOC_Protocol_Commit`（无调用者）。
  - PID 增益写入统一"**不清积分**"：移除协议对 `integral/prev_error` 的写入（协议只写增益字段，不插手控制动态状态），消除"电流环全清 vs 速度/角度环仅 kp 清"的不一致。
  - `P:W`（sensor_sample_offset_percent）开放运行时可写：写入即生效（立即调 `Sensor_ADCSampleTimeOffset`），读回恢复；文档标注为调试用途。
  - ROM：54.96kB → 54.90kB。

### Note
- 电流环 / 速度环 / 角度环三份 PID 核心按设计保持独立（电流环条件积分、速度环 back-calculation、角度环死区+限幅），本轮不做行为合并。
- PID 参数配置撕裂结论：参数为"主循环单写者 + ISR 多读者"，单字段对齐原子访问，1~2 控制周期读取延迟可接受；不引入临界区/发布点等原子化机制（过度设计）。

## [2.0.6] - 2026-08-03

### Fixed
- **SMO 测速窗口消除 ISR 频率硬编码（结构性传参修复）**：
  - `EstimSMO_StepTail` 原以 2ISR 推导宏 `FOC_CURRENT_LOOP_ISR_FREQ`（8kHz）重建 16 拍测速窗口时长，三 ISR 模式（实际电流环 4kHz）下窗口被算成 2ms（实际 4ms），导致 SMO 测速恒定放大 2 倍。现改为累计调用方每拍传入的 `dt_sec`（新增 `speed_dt_accum` 字段），彻底消除对 ISR 频率宏的依赖。
  - `FOC_CURRENT_LOOP_DT_SEC` 收敛为按 `FOC_CURRENT_LOOP_ISR_MODE` 条件编译的唯一真值（双 ISR 取分频周期，三 ISR 取 `1/FOC_CURRENT_LOOP_ISR_FREQ_HZ`）；删除语义孪生的 `FOC_CURRENT_LOOP_ISR3_DT_SEC`，Executor 两个电流环入口统一使用 `FOC_CURRENT_LOOP_DT_SEC`。
  - **编译期约束收口**：`foc_compile_limits.h` 校验 `FOC_CURRENT_LOOP_ISR_MODE` 合法值；三 ISR 模式强制 `FOC_CURRENT_LOOP_ISR_FREQ_HZ` 非零，避免辅助定时器不初始化导致电流环静默停摆。
- **速域切换判据收口（修复电机反向接线时低速域被速度尖峰误切高速域的问题）**：
  - **目标速度域门槛**：升域（`LOW_ACTIVE→HIGH_ACQUIRE`）、等待（`HIGH_ACQUIRE` 保持）、降级（`HIGH_ACTIVE→HIGH_SUSPECT`）、降级恢复（`HIGH_SUSPECT→HIGH_ACTIVE`）全部要求 `SPEED_ONLY` 且 `|speed_only| > high_th`。目标在低速域时，即使实测瞬时速度 > 门限、SMO 已收敛，也禁止切入/驻留高速域，杜绝单拍速度尖峰盖过其他判据。
  - **降级去抖独立宏**：新增 `FOC_SOURCE_SWITCH_DEGRADE_CONFIRM_CYCLES`（默认 20）。`HIGH_SUSPECT` 恢复 `HIGH_ACTIVE` 需目标在高速域 且 实测速度 ≥ high 门限连续该宏拍数；移除 `speed_valid==0` 无条件放行，速度读取失败或单拍尖峰不再打断敏捷降级。
  - **非速度控制模式锁定**：`FOC_SourceMgr_Select` 在非 `SPEED_ONLY` 时锁定 LOW 源、不推进速域状态机（角度模式依赖编码器可靠源）。
  - **运行时角度模式编译期收口**：`FULL` 构建下（可在运行时切到 `SPEED_ANGLE`）要求 `LOW` source 为 ENCODER/HFI；`HIGH` 可为 SMO 仅服务速度模式高速无感段。
  - **枚举收口**：删除未使用的 `FOC_REGION_STATE_HIGH_READY`；`foc_source_mgr_state_t` 新增 `degrade_hold_counter`（追加在非条件编译段，保持字段顺序兼容）。

### Documentation
- `docs/architecture.md` Source Manager 状态机判据、`Select` 伪码、编译期约束条目同步更新；版本基线统一更新至 v2.0.6。

## [2.0.5] - 2026-08-01

### Added
- **ISR 架构双模式**：新增 `FOC_CURRENT_LOOP_ISR_MODE` 宏（`FOC_ISR_MODE_2ISR` 默认 / `FOC_ISR_MODE_3ISR`），三 ISR 模式将电流环拆分到独立定时器 ISR（默认 8kHz），与 PWM 频率解耦。
- **PWM 插值正交裁剪**：新增 `FOC_SVPWM_INTERP_ENABLE` 宏，两种 ISR 模式均可独立启用/禁用 PWM 插值，禁用后直接写占空比。
- **三 ISR 模式独立电流环频率**：`FOC_CURRENT_LOOP_ISR_FREQ_HZ` 可任意配置电流环 ISR 频率。
- **通用辅助定时器 API**：`FOC_Platform_AuxTimerInit/Start/Stop/SetCallback`，ID 枚举化（`FOC_Platform_AuxTimerId_t`），平台内映射空闲硬件定时器（GD32 实例映射 TIMER4）。
- **平台内存屏障 API**：`FOC_Platform_MemoryBarrier()`，用于多 ISR 共享数据的写序保证。
- **实例层 AUXTIMER 驱动**：`Utilities/AUXTIMER/auxtimer.h/.c`，自由运行模式，8kHz 独立节拍。

### Changed
- **Executor 三入口拆分**：`FOC_ControlExecutor_RunISR`（双 ISR）、`FOC_ControlExecutor_RunISR_PwmOnly`（三 ISR PWM 仅插值+守卫）、`FOC_ControlExecutor_RunISR_CurrentLoop`（三 ISR 独立电流环）。
- **SVPWM 并发保护**：三 ISR 模式采用 pending + commit 标志，PWM ISR 入口原子取走目标，零锁零阻塞。
- **`FOC_Init_Runtime` 签名扩展**：新增 `current_loop_cb` 参数，三 ISR 模式注册独立电流环回调。
- **L1 编排适配**：新增 `FOC_App_OnCurrentLoopISR`；OSC 快照在三 ISR 模式移入电流环 ISR（反映控制环实际数据）。
- **空平台 API 同步**：`foc_platform_api_empty.c` 补齐辅助定时器与内存屏障接口。
- **`foc_platform_api.c` 分块整理**：按功能区块（Runtime/Indicator/Comm/Sensor/PWM/AuxTimer/Protection/Diagnostics）重排函数顺序。

### Documentation
- 版本基线统一更新至 v2.0.5。

## [2.0.4] - 2026-07-29

### Added
- **特殊控制状态退出机制**：新增 `Y:A` 系统命令（`FOC_SPECIAL_PHASE_ABORT_ENABLE`），可中断齿槽标定/重初始化等长流程非阻塞状态机，自动回退至 NORMAL 控制阶段。支持自动退出：禁能（`S:M=0`）或控制模式切换（`P:D`）时自动退出特殊状态。退出时统一走 `FOC_ControlExecutor_FullStop` 归零 PWM 和控制状态。
- **齿槽标定/重初始化 Abort 函数**：`FOC_CoggingCalib_Abort` / `FOC_ReInit_Abort` 清理对应模块内部状态机。
- **Abort 诊断输出**：退出时通过 `FOC_Protocol_OutputDiag("INFO", "abort", ...)` 输出被中断的 phase 名称。
- **滤波器架构完善**：补齐 `FOC_FILTER_ENCODER_SPEED` Kalman 四参数宏；编码器速度后级滤波通路接通（滑动平均后追加 filter gate）。
- **`foc_filter_none_t` 占位类型**：替代 `uint8_t` 作为 `FOC_FILTER_TYPE_NONE` 的统一中间状态类型，消除 NONE 配置下 `.output_value` 访问的编译失败。
- **角度模式编译期约束**：新增 `FOC_CONTROL_SRC_IS_ANGLE_CAPABLE` 宏，`SPEED_ANGLE_ONLY` 构建或 `FULL` 构建下默认角度模式时，若 source 非 ENCODER/HFI 则 `#error`。

### Changed
- **安全输出统一化**：引入 `FOC_ControlExecutor_FullStop` 作为唯一停止入口，收敛 fault/disable/phase-switch 三条归零路径。清零 ud/uq/iq_target、全部 PID、外环累积状态、mode_transition 标记、current_loop_ready、软切换状态，最后归零 PWM。
- **L1 守卫简化**：`FOC_App_OnPwmUpdateISR` 中删除 `motor_enabled` 拦截，该决策下沉到 L2 `RunISR`。
- **`RunISR` 门控重排**：`motor_enabled==0` → FullStop；特殊 phase（COGGING_CALIB/REINIT）→ 仅 ApplyPhaseOutputRuntime（ControlTrigger 负责设置输出）；其余非 NORMAL → 跳过。
- **SVPWM 滤波器删除**：`FOC_FILTER_SVPWM` 开关和 `FOC_FilterGate_Svpwm` 函数完整删除。
- **`FOC_FILTER_TYPEDEF_0` 语义修正**：从 `uint8_t` 改为 `foc_filter_none_t`。

### Documentation
- 版本基线统一更新至 v2.0.4。

## [2.0.3] - 2026-07-29

### Changed
- **PWM ISR 编排统一**：将 `system_fault`/`system_running`/`motor_enabled` 系统级守卫从 L2 `RunISR` 提升到 L1 `FOC_App_OnPwmUpdateISR`，与 Control ISR 编排风格对齐。L2 RunISR 仅保留管线内部守卫（`control_phase`/`current_loop_ready`）。
- **模块精简**：删除 `foc_ctrl_transition.c/.h`（仅含 39 行的单函数模块），`OnModeSwitch` 合并到 `foc_ctrl_executor.c` 作为 static 函数，executor 职责扩展为"PWM ISR 与 Control ISR 路由，外环调度、控制模式切换"。
- **Source Manager 状态机优化**：合并 `HIGH_READY` 到 `HIGH_ACQUIRE`（消抖完成后直接检查切入条件，消除中间状态），补齐 `HIGH_SUSPECT` 入边（HIGH_ACTIVE 降级嫌疑转为 HIGH_SUSPECT 而非原地消抖）。`RebaseSource` 切换回 OpenLoop 且旧源无物理速度时，使用 `ramped_speed_rad_s` 作为速度保底值。

### Documentation
- 版本基线统一更新至 v2.0.3。
- 架构文档同步 Control ISR 欠压保护描述、删除 `Sensor_SyncCurrentSnapshot`、PWM ISR L1 系统守卫段。

## [2.0.3-prev] - 2026-07-28

### Added
- **速域状态机显式化**：Source Manager 新增 `region_state` 与 `config_valid`，将低速、高速获取、高速就绪、高速运行和低速恢复拆成可观测状态。
- **切换提交同步**：source/region 切换时同步外环机械角历史、速度误差状态、ramped speed，并预置速度 PID/电流 PID 状态，降低 open-loop 到 closed-loop 接管时的控制量断层。
- **全局控制加速度机制**：将速度斜坡/速域上限作为外环通用策略处理，OpenLoop 及普通外环路径统一受速度限制与加速度约束管理。

### Changed
- **Source Manager 鲁棒性提升**：低速升高速改为低速侧运动授权 + 高速候选源有效性双条件；openloop 低速源不再仅因外部拖动 encoder 速度越过门限而触发升域。
- **高速降级消抖**：`HIGH_ACTIVE` 中的失锁/低速降级由单拍触发改为连续确认，门限附近不再直接跳入降级状态。
- **角度发布链路收口**：运行时电角度由 Source Manager Publish 统一发布，执行输出路径只消费已发布角度。
- **电流软切换默认参数调整**：AUTO 模式说明更新，默认闭环运行；降低 blend 时间常数，并将 AUTO 开环阈值调整为 0，避免长期混合降低高速带宽。
- **代码优化**：清理 source snapshot 和过期切换字段，简化 Source Manager 数据路径；移除旧的 source-switch transition 入口。

### Fixed
- 修复反向/测试配置下 source 状态乱跳、门限附近频繁切换的问题。
- 修复电角度环绕写入导致 ISR 时间随运行逐渐上升的问题。
- 修复 current soft-switch 内指针与整数比较 warning。

### Documentation
- 版本基线统一更新至 v2.0.3。
- 架构文档同步 Source Manager 状态机、切换条件、切换同步和全局加速度策略。

## [2.0.2] - 2026-07-27

### Changed
- **文档全面同步**：全量文档审计，版本基线统一更新至 v2.0.2。
- **架构文档同步**：`docs/architecture.md` 目录路径修正（`L2/` → `L2_Core/`）、L2 Control 模块列表去 CXX 编号改用实际文件名、数据流图同步（消除 `ctrl_input` 桥接残余，改用 `active_source_state` + `motor->ctrl.iq_target`）、ISR 阶段描述修正（PWM ISR 5 阶段实际执行顺序：采样→Estimator→Select/Publish→电流环→SVPWM；Control ISR 按 source 类型路由 OpenLoop/外环）。
- **架构文档扩展**：新增 `control_region`（LOW/HIGH/FULL）描述；新增独立 "Source Manager 体系" 章节，详述两步分离设计（Select/Publish）、预收敛机制（所有 Estimator 后台迭代）、切换策略三分支（阈值/收敛/消抖）和 encoder_services 绑定。
- **开发文档同步**：`docs/development.md` 模块命名列表去 CXX 编号，使用实际文件名。
- **规则文件同步**：`.clinerules/hywfoc-project-rules.md` L2 Control 模块命名段去 CXX 编号，版本基线同步。
- **README 基线**：`README.md`、`docs/README.md` 版本基线更新至 v2.0.2，协议文档链接从双语版修正为单语版。
- **协议文档修正**：`docs/protocol-parameters.md` 语义调试行标签 `encoder_angle` → `mech_angle`（v2.0.0 重构后数据源变更）。

### Documentation
- 版本基线统一更新至 v2.0.2
- 全量文档审计：8 个文件更新

## [2.0.1] - 2026-07-21

### Added
- **滤波器架构重构**：可替换滤波器系统，支持编译期切换 Kalman/LPF1/Biquad 滤波器类型。
- 新增 `foc_filter_type_map.h`（LS_Config）— 滤波器类型枚举 + 类型推导宏表，用户每个位置只需修改一个整数值宏。
- 新增 `foc_filter_types.h`（L3_Hal）— `foc_filter_kalman_t`、`foc_filter_lpf1_t`、`foc_filter_biquad_t` 统一滤波器类型。
- 新增 `foc_filter_math.h/.c`（L3_Hal）— 纯数学函数：`KalmanStep`、`KalmanAngleStep`、`Lpf1Step`、`BiquadStep`（骨架），零状态依赖，可任意复用。
- 新增 `foc_filter_gate.h`（L3_Hal）— 每个滤波位置一个 `static inline` 门控函数，编译期 `#if` 选择纯数学函数，调用方只需 `FOC_FilterGate_*()`。

### Changed
- **传感器滤波统一**：`sensor_data_t` 中 `kalman_filter_t` 替换为 `FOC_FILTER_TYPEDEF()` 推导宏，支持每个通道独立选择滤波器类型。
- **电流Iq滤波统一**：`motor->iq_lpf` 替换为 `motor->iq_lpf_filter`（`FOC_FILTER_TYPEDEF` 推导）。
- **角度数据流收口**：`mech_angle_rad` 滤波后统一通过 `output_value` 字段读取，消除 `raw_value`/`filtered_value` 旁路泄漏（`foc_ctrl_executor.c`、`foc_ctrl_estim_encoder.c`、`foc_debug_stream.c`）。
- **Kalman 函数提升**：`foc_sensor.c` 中 `static` 的 `Kalman_Init/Update/Update_Angle` 提升为 L3 公开 API（`FOC_FilterMath_Kalman*`）。
- **旧宏清理**：删除 `FOC_SENSOR_KALMAN_CURRENT_ENABLE`、`FOC_SENSOR_KALMAN_ANGLE_ENABLE`，统一为 `FOC_FILTER_*` 宏体系。
- **构建入口同步**：`builder.params` 新增 `foc_filter_math.c`。

### Fixed
- 修复 `foc_filter_lpf1_t` 缺少 `raw_value`/`output_value` 字段导致切换 LPF 编译失败。
- 修复 `Sensor_InitSnapshot` 初始化共用 `CURRENT_A` 条件导致 `CURRENT_B`/`CURRENT_C` 类型不匹配。
- 修复 `foc_cfg_filter.h` 缺少 `FOC_FILTER_SENSOR_CURRENT_C_KALMAN_*` 参数宏（当前 `FOC_CURRENT_SENSE_PHASES=2U` 下被屏蔽）。
- 修复 `Sensor_InitSnapshot` 中 C 相 Kalman Init 引用了 A 相参数宏（`FOC_FILTER_SENSOR_CURRENT_A_KALMAN_*`），应引用 C 相宏。
- 修复角度 LPF（`FOC_FilterMath_Lpf1Step`）在 0/2PI 切换点的环绕振荡：新增 `FOC_FilterMath_Lpf1AngleStep`，使用 `Math_WrapRadDelta` 计算最短路径角度增量后平滑，Angle 门 LPF1 分支改为此函数（`foc_filter_gate.h`、`foc_filter_math.h/.c`）。

## [2.0.0] - 2026-07-20

### Added
- **无感 FOC 架构扩展**：项目从仅支持编码器反馈扩展至多源估计器统一框架，覆盖编码器（有感）、SMO（中高速无感）、HFI（零速/低速无感）、FLUX（预留）。
- **估计器体系**：新增 `foc_ctrl_estim.h/c`（选择/注册中心）、`foc_ctrl_estim_encoder.c`（编码器估计器实现）、`foc_ctrl_estim_smo.c`、`foc_ctrl_estim_hfi.c`、`foc_ctrl_estim_flux.c`（空桩）。
- **桥接层**：新增 `foc_ctrl_bridge.h/c`，实现估计器输出到控制输入快照（`est_state` → `ctrl_input`）的单一拷贝路径。
- **启动策略模块**：新增 `foc_ctrl_startup.h/c`（管理状态机）+ `foc_ctrl_startup_openloop.c`（强拖空桩）。
- **过渡管理**：新增 `foc_ctrl_transition.h/c`，支持双估计器加权混合过渡。
- **控制输入快照**：新增 `foc_control_input_t`，控制算法统一从 `ctrl_input` 读取角度和电流，不直接接触估计器内部状态。
- **配置宏**：新增 `FOC_SENSOR_ENCODER_ENABLE`、`FOC_ESTIMATOR_*_ENABLE`、`FOC_STARTUP_OPENLOOP_ENABLE`、`FOC_TRANSITION_ENABLE`，传感器硬件和算法使用通过不同宏独立控制。

### Changed
- **数据结构扩展**：`foc_motor_t` 新增 `est_state`/`est_state_alt`、函数指针、5 个条件编译子结构体、`ctrl_input` 字段。
- **控制阶段枚举**：新增 `FOC_CONTROL_PHASE_STARTUP`。
- **故障码**：新增 `FOC_FAULT_ESTIMATOR_INVALID`。
- **Control ISR 4 阶段**：传感器读取 → 估计器更新 → 桥接拷贝 → 控制执行。
- **PWM ISR 4 阶段**：插值与采样 → 电流环 → 角度同步 + 估计器 → SVPWM。
- **外环数据源**：从 `sensor->mech_angle_rad.output_value` 改为 `motor->ctrl_input.mech_angle_rad`。
- **传感器有效性检查**：收口到 L1 单一检查点，由 `FOC_SENSOR_ENCODER_ENABLE` 宏条件化。
- **DebugStream 适配**：角度门控从 `encoder_valid` 改为 `ctrl_input.valid`，标签从 `encoder_angle` 改为 `mech_angle`。
- **有感模块重命名**：`foc_ctrl_cogging_calib` → `foc_ctrl_sens_cogging_calib`，`foc_ctrl_reinit` → `foc_ctrl_sens_reinit`。
- **编译期约束**：新增 13 条约束（估计器依赖、齿槽依赖、SMO 启动策略依赖等）。

### Documentation
- `docs/architecture.md`：完整更新为 20 模块架构，新增估计器数据流、控制阶段枚举、宏组合表。
- `.clinerules/hywfoc-project-rules.md`：新增 6 条通用规则（架构扩展零回归、构建入口同步、头文件一致性等）。
- 版本基线更新至 v2.0.0。

## [1.10.2] - 2026-07-13

### Changed
- **X 指令输出路径从快路径改为队列**：`P:X`/`C:X`/`S:X`/`Y:X` 四个批读命令的批量输出从快路径直写改为 TX FIFO 队列输出，消除 Service 段因批量 DMA 写入引起的长阻塞（最大约 50ms）。批量输出函数移至 L2 Protocol 层（`FOC_Protocol_QueueParams/Configs/States/SystemInfo`），复用已有的 `ReadParam/ReadConfigParam/ReadState` 静态函数，消除 L1 层的字段映射重复。
- **`foc_app.c` 精简重构**：初始化逻辑移至 `foc_init.h/.c`（取代 `foc_init_check.h/.c`），Monitor 段元素处理移至 `foc_output_mgr.c` 的 `FOC_OutputMgr_ProcessMonitorElements`，辅助函数（`IsCalibrating/ApplyCfgDirty/SampleSensors`）内联。`foc_app.c` 从 432 行精简至 272 行。

### Fixed
- **齿槽标定逆方向（direction=-1）不工作**：`foc_ctrl_cogging_calib.c` SCAN 阶段修复两个 bug——`pred_wrapped` 使用 `2π - pred_mech_angle` 而非 `Math_WrapRad(pred_mech_angle)` 导致 `dtheta` 计算错误；反向标定时 `iq_comp` 缺少符号反转。

### Removed
- `foc_init_check.h/.c` 删除，由 `foc_init.h/.c` 替代。

### Documentation
- 版本基线更新至 v1.10.2。

## [1.10.1] - 2026-07-13

### Added
- **电流环 ISR 执行时间监控**：新增语义调试行 8（`control.current_loop_execution_time_us`），在调试流中输出 PWM ISR 中电流环的执行时间（微秒）。
- **系统参数只读命令 `Y:X`**：新增系统子命令 `Y:X`，支持通过 `aaYXb` 读取电机系统参数：相电阻、极对数、零位、方向、总线电压、最大相电压、ADC 零偏等。
- **X指令裁剪宏 `FOC_PROTOCOL_ENABLE_BATCH_READ`**：新增 `foc_cfg_feature_switches.h` 总控宏，关闭后 `P:X`/`C:X`/`S:X`/`Y:X` 四个批读命令的字符串和函数体全部编译排除，节省 ROM。

### Changed
- **语义调试流行数扩展至 9 行**：`SEMANTIC_LINE_COUNT` 从 8 增加到 9，新增行 8 为电流环 ISR 执行时间，行 7（调度器执行时间）格式移除末尾多余空行。

### Documentation
- `docs/protocol-parameters.md`：全子命令索引表（3.1 节）、裁剪宏列合并到参数表、表格列结构统一为 9 列、示例精简为附录 A、新增 4.7 节语义调试行说明。Y 组新增 `X` 子命令。
- `docs/README.md`、`README.md`、`NEXT_MISSION.md`：版本基线更新至 v1.10.1。
- `CHANGELOG.md`：新增 v1.10.1 变更记录。

## [1.10.0] - 2026-07-10

### Changed
- **P 命令组拆分为 P 组 + C 组**：因原 P 组（`'P'`）26 个大写字母地址空间已耗尽且存在 3 组字符冲突（`'U'`: PID_SPEED_KI / COGGING_COMP_IQ_LIMIT；`'V'`: PID_SPEED_KD / COGGING_COMP_SPEED_GATE；`'N'`: PID_ANGLE_KD / CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ），将调优/配置参数（PID、control fine-tuning、齿槽补偿、电流软切换）移至新命令组 `'C'`（Configuration）。P 组仅保留运行时参数（目标角度/速度/控制模式/遥测配置），释放 17 个空闲 A-Z 槽位。
- **C 组新增 `X` 批量读取哨兵**：`C:X` 对应 `aaCXb`，一次性输出所有 C 组配置参数（PID + fine-tuning + 齿槽 + 软切换），格式 `config.<name>=<value>`。

### Added
- `COMMAND_MANAGER_CMD_CONFIG` 命令组（`'C'`）及 20 个 `COMMAND_MANAGER_CONFIG_SUBCMD_*` 子命令宏。
- `FOC_Protocol_OutputConfigParam()` / `ProtocolText_GetConfigName()` / `ProtocolText_IsIntegerConfigParam()` / `ProtocolText_FormatConfigLine()` — C 组参数格式化/输出函数链。

### Fixed
- 消除 P 组 `'U'`/`'V'`/`'N'` 子命令宏定义冲突（speed PID tuning / cogging compensation / current soft switch 互斥限制不再需要）。

### Documentation
- `docs/protocol-parameters.md`：完整更新 C 命令组表格、示例命令、裁剪映射表。
- `docs/README.md`、`README.md`、`NEXT_MISSION.md`：版本基线更新至 v1.10.0。

## [1.9.4] - 2026-07-09

### Changed
- **禁用 ADC 硬件过采样**：移除 `adc_oversample_mode_config/enable` 配置，所有计算使用 12-bit 原始值（0~4095）。解决过采样导致采样窗口过大、大电流下有效采样窗口过小的问题。
- **`monitor_elem_q` 队列竞态修复**：协议摘要（PROTOCOL_SUMMARY）不再经 `monitor_elem_q` 中转，改为 Service 段直接入 `tx_fifo`。消除 ISR（MonitorTrigger）与主循环（Service）的队列写冲突导致的帧丢失和串口卡死。
- **示波器输出 5 通道**：默认输出 `current_a.output_value`、`current_b.output_value`、`current_c.output_value`、`current_a_raw`（ADC 驱动输出值，方向系数后零偏前）、`current_b_raw`。精简默认掩码，关闭 angle/iq_target/iq_measured 等不必要通道。

### Added
- `sensor_data_t` 新增 `current_a_raw` / `current_b_raw` 字段（ADC 原始采样值，过采样后，未零偏/电周期补偿）。
- `DEBUG_STREAM_OSC_PARAM_CURRENT_A_RAW` / `CURRENT_B_RAW` 位掩码定义。

### Fixed
- **串口输出卡死问题**：Service 段入队 PROTOCOL_SUMMARY 到 `monitor_elem_q` 与 ISR 端写入冲突 → 队列状态损坏 → 出队读到不完整 tag → 帧丢弃 → 串口无输出。修复后 `monitor_elem_q` 为纯 ISR 写 → 主循环读，无竞态。

## [1.9.3] - 2026-07-06

### Changed
- **SETTLE 阶段重写**：将原来"旋转一圈"的积分行程判定改为"锁定当前角度 N 个控制周期"（`FOC_COGGING_CALIB_SETTLE_CYCLES`，默认 150 周期 ≈ 100ms）。消除了因微小角度差（~0.0004 rad/步）逐周期积分噪声导致的行程误判。
- **SCAN 阶段 bin 判定去抖**：改用方向锁定的前进判定替代单纯的 `!=` 比较——仅接受沿旋转方向前进的 bin 切换，拒绝传感器噪声导致的 bin 弹回和重复写入。
- **移除角度同步**：删除 SETTLE→SCAN 和 CHECK→SCAN 过渡处的 `pred_mech_angle = sensor->mech_angle_rad.output_value` 同步，保持驱动电角度连续无跳变。
- **USART TX 快慢路径分离**：将 USART1 输出重构为两条独立路径——快路径（`USART1_FastWriter_*`，ISR-safe，环形缓冲区+TXE中断驱动）和慢路径（`USART1_SlowWriter_SendData`，主循环阻塞DMA）。删除旧 `USART1_SendByte/SendString/SendData` API。
- **新增 `FOC_Platform_WriteDebugFast`**：ISR-safe 快路径文本输出 API，用于 ISR 中的短状态信息。
- **齿槽标定输出路径分离**：`CoggingCalib_Finish` 中所有短状态文本改为快路径输出，大表 dump 改为设 `request_dump` 标志由主循环 Service 段输出，消除 ISR 中阻塞 DMA 导致的死锁。

### Added
- `USART1_FastWriter_PutString` — 快路径非阻塞字符串输出。
- `FOC_Platform_WriteDebugFast` — L3 平台 API 快路径文本输出（`foc_platform_api.h`）。

### Removed
- `foc_cogging_calib_state_t` 中 `travel_accum_rad`、`angle_prev_rad`、`rev_count` 三个字段（不再使用）。
- `FOC_COGGING_CALIB_SETTLE_REV` 配置宏（替换为 `FOC_COGGING_CALIB_SETTLE_CYCLES`）。
- `CoggingCalib_AngleDelta` 辅助函数（不再使用）。
- `USART1_SendByte`、`USART1_SendString`、`USART1_SendData` — 已由 FastWriter/SlowWriter 替代。

### Fixed
- 齿槽标定过程中因 `bins_collected` 计数误触（噪声使 bin index 弹回导致同一 bin 被重复写入）导致的 LUT 连续异常值。
- 标定过渡阶段因 `pred_mech_angle` 同步导致的驱动电角度跳变。
- **ISR 中调用慢路径阻塞 DMA 导致系统卡死**：`CoggingCalib_Finish` 在控制 ISR 中通过 `FOC_Platform_WriteDebugText`（慢路径 DMA）输出状态信息和齿槽表，DMA 阻塞等待被 ISR 优先级阻塞导致永久死锁。修复方式：短文本改为 `FOC_Platform_WriteDebugFast`（非阻塞 TXE 中断），大表输出改为设标志位由主循环 Service 段处理。

## [1.9.2] - 2026-06-30

### Changed
- **L1 层职责重构**：`foc_service_handler.c/h` 删除，拆分为 `foc_indicator.c/h`（指示器）+ `foc_init_check.c/h`（初始化校验），其余函数内联到 `foc_app.c`。
- **`foc_app.c` 精简**：从 ~575 行精简至 ~260 行，通信轮询迁至 `foc_output_mgr.c`，无效/摘要行格式化迁至 L2 层。
- **`foc_runtime_ctx_t` 新增 `osc` 字段**：替代 `foc_app.c` 静态全局示波器累积缓冲区。

### Fixed
- **齿槽补偿标定后 DMA 死锁**（USART 不可重入函数被 ISR 抢占）：标定阶段主循环跳过 Monitor/Service 任务、dump 输出移至 ISR 内 `CoggingCalib_Finish`。
- **示波器 `iq_measured` 始终为 0**：`DebugStream_PollNextValue` 中 `case 8` 不匹配掩码位 `0x0200`（bit 9），switch 缺少 `case 9U`。

### Added
- `DebugStream_FormatInvalidLine()` — L2 无效语义行格式化。
- `FOC_Protocol_FormatSummaryLine()` — L2 协议摘要行字符串转换（纯方法，无全局变量）。
- `FOC_OutputMgr_PollSources()` / `FOC_OutputMgr_WriteStartupInfo()` — L1 输出管理器扩展。

## [1.9.1] - 2026-06-26

### Changed
- **Monitor 输出路径修复**：`FOC_App_MonitorTrigger` 由仅设标志位改为 ISR 中快照 sensor 关键字段 + `DebugStream_PollNextValue` 逐元素入 `monitor_elem_q`。主循环 Monitor 段改为出队 + tag switch 格式化 + 入 TX FIFO。协议摘要通过 `MONITOR_ELEM_PROTOCOL_SUMMARY` 经 monitor_elem_q 统一输出。
- **慢路径统一为标记元素队列**：新增 `monitor_element_t {tag, aux, value}` 标记元素类型，`FRAME_START` 帧隔离标记防止主循环阻塞后帧交错。语义行（SEMANTIC_0~7）、示波器值（OSC_VALUE/OSC_END）、协议摘要（PROTOCOL_SUMMARY）全部走同一队列。
- **`foc_debug_stream` 拆分为双接口**：`DebugStream_PollNextValue`（ISR 安全，跑 state machine 输出元素）和 `DebugStream_FormatSemanticLine`/`AppendOscValue`/`FormatOscLine`（主循环格式化）。原 `GenerateLine` 保留为向后兼容封装。

### Added
- `foc_monitor_queue_types.h` — `monitor_elem_tag_t` 枚举与 `monitor_element_t` 结构体（L1 新增）。
- `foc_runtime_ctx_t` 中新增 `monitor_elem_q` 队列字段（FIFO，32 元素深度）。
- `FOC_MONITOR_ELEM_QUEUE_DEPTH` / `FOC_MONITOR_MAX_DEQUEUE_PER_CYCLE` 配置宏（`foc_cfg_init_values.h`）。

### Documentation
- `docs/architecture.md`：新增 Monitor 元素队列机制描述，更新数据流图和输出路径表。
- `docs/README.md`、`copilot-instructions.md`：版本基线 v1.9.0→v1.9.1。

## [1.9.0] - 2026-06-25

### Changed
- **控制任务阶段路由重构**：`FOC_App_ControlTrigger` 由直接调用 `RunCycle` 改为 `control_phase` 状态机路由。新增 NORMAL/COGGING_CALIB/REINIT 三个阶段，公共安全检查（传感器、欠压）上提到 L1。
- **齿槽标定提为独立控制阶段**：从 `foc_ctrl_executor.c` 的 `RunCycle` 中迁出，独立为 `foc_ctrl_cogging_calib.c/.h`（L2/Control）。不再嵌入 `RunCycle`，由 L1 通过 `control_phase` 路由调用。
- **重初始化非阻塞化**：将阻塞式 `FOC_Service_ReInitMotor` 改为非阻塞状态机 `foc_ctrl_reinit.c/.h`（L2/Control）。D 轴对齐流程与上电阻塞标定一致（`ud=calib_uq, uq=0`），通过控制周期计数替代 `FOC_Platform_WaitMs`。
- **协议 Y 通道精简**：`Y:G`/`Y:W`/`Y:E`/`Y:R` 命令改为直接调用模块 API，不再经过 `pending_system_action` 中转。
- **PWM ISR 中 SVPWM 插值独立于控制阶段**：`SVPWM_InterpolationISR` 移至 `RunISR` 最前，任何阶段（含标定/重初始化）均执行，确保开环驱动目标值生效。

### Added
- `foc_ctrl_cogging_calib.c/.h` — 齿槽标定独立模块，由 `FOC_CoggingCalib_RunStep()` 状态机驱动。
- `foc_ctrl_reinit.c/.h` — 非阻塞重初始化状态机，由 `FOC_ReInit_RunStep()` 驱动。
- `foc_control_phase_t` 枚举（`FOC_CONTROL_PHASE_NORMAL/COGGING_CALIB/REINIT`）— 控制阶段路由选择。
- `foc_reinit_state_t` 结构体 — 非阻塞重初始化状态（每电机实例）。

### Removed
- `pending_system_action` 字段（`foc_motor_state_t`）— 被 `control_phase` 替代。
- `reinit_pending` 字段（`foc_motor_state_t`）— 被 `control_phase == REINIT` 替代。
- `FOC_SYSACTION_*` 宏 — 不再使用。
- `FOC_Service_ReInitMotor()` — 阻塞式重初始化已替换为非阻塞状态机。
- cogging calibration 代码从 `foc_ctrl_compensation.c` 移出（约 700 行）。
- `RunCycle` 中的 cogging 分支和重复安全检查。

### Documentation
- `docs/architecture.md`：L2/Control 模块数 8→10，新增 cogging_calib/reinit 描述。
- `docs/development.md`：模块命名列表更新。
- `docs/README.md`、`copilot-instructions.md`：版本基线 v1.8.2→v1.9.0。

## [1.8.2] - 2026-06-25

### Changed
- **L1 函数封装统一**：`foc_app.c` 中提取 4 个静态函数（`FOC_App_OutputStartupInfo`、`FOC_App_PollCommSources`、`FOC_App_ApplyCfgDirty`、`FOC_App_HandleControlResult`），消除内联松散逻辑，使顶层函数保持在单一抽象层。
- **L1→L3 硬件初始化调用收口**：L1 不再直接调用 `Sensor_*`/`SVPWM_*` 等 L3 硬件初始化方法，改为调用 L2/C12 新增的 `FOC_ControlPlatform_InitHardware()`。同时消除 `FOC_ControlExecutor_Init()` 在 L1 和 L2 的重复调用。
- **4 源接收公平轮询**：`FOC_App_ServiceTrigger` 中的 4 源轮询改为 round-robin 起始偏移策略，避免固定优先级导致低优先级源被饿死。新增 `FOC_COMM_MAX_FRAMES_PER_SERVICE` 宏（默认 0=全部）可配置每 ServiceTrigger 处理的帧数上限。
- **`comm_source_rr` 并入运行时结构体**：round-robin 起始偏移状态保存在 `foc_runtime_ctx_t` 中，不引入独立全局变量。跨 reinit 保持状态。

### Added
- `FOC_ControlPlatform_InitHardware(motor)` — L2/C12 新增硬件初始化收口函数，封装传感器、SVPWM、快速电流环初始化序列。
- `FOC_COMM_MAX_FRAMES_PER_SERVICE` — 可配置宏（`foc_cfg_init_values.h`），控制每 ServiceTrigger 轮询入队的最大帧数（0=自动，1~4=固定上限）。

### Cleanup
- `foc_app.c` 中移除 `#include "L3/foc_sensor.h"` 和 `#include "L3/foc_svpwm.h"`（L1 依赖树宽度缩减）。
- `FOC_App_ApplyCfgDirty` 移除冗余的 `Sensor_ADCSampleTimeOffset` 调用（已在 `FOC_Control_ApplyConfig` 中自动执行）。

### Documentation
- `docs/architecture.md`：分层约束新增第9条——L1 不直接调 L3 硬件初始化方法，硬件初始化通过 L2 收口。

## [1.8.1] - 2026-06-24

### Changed
- **队列配置 LS 化**：将 `FOC_OUTPUT_QUEUE_DEPTH`、`FOC_OUTPUT_FRAME_MAX_LEN`、`FOC_OUTPUT_MAX_PER_CYCLE` 从 `foc_system_types.h` 和 `foc_output_mgr.h` 移入 `foc_cfg_init_values.h`，实现队列配置默认值收敛到 LS 层。用户可在构建前通过 `#define` 覆盖。

### Removed
- `foc_system_types.h` 中的 `FOC_OUTPUT_QUEUE_DEPTH` 和 `FOC_OUTPUT_FRAME_MAX_LEN` 本地默认值（已移到 LS_Config）
- `foc_output_mgr.h` 中的 `FOC_OUTPUT_MAX_PER_CYCLE` 本地默认值（已移到 LS_Config）

### Documentation
- `PLAN.md`：更新为评估收敛版本，记录所有评估结论和完成项

## [1.8.0] - 2026-06-22

### Changed
- **运行时管线彻底删除**：消除冗余的 C1→C2→C3→C4→C5 运行时管线，删除 `foc_runtime_entry.c/.h`、`foc_runtime_protocol.c/.h`、`foc_runtime_fsm.c/.h`、`foc_runtime_store.c/.h`、`foc_runtime_output.c/.h`。原功能分散到新的 Protocol 块和 L1 编排。
- **运行时状态并入 motor 结构体**：`foc_motor_t` 大幅扩展，新增嵌套结构体 `.state`（运行时状态，含故障/init_check/cfg_dirty 等）、`.cfg`（控制配置参数），所有运行时数据变为 per-motor。
- **PID 对象移入 motor**：`g_torque_current_pid`、`g_speed_pid`、`g_angle_pid` 从独立全局变量变为 `motor.torque_current_pid` / `motor.speed_pid` / `motor.angle_pid`。
- **快照机制取消**：删除 `runtime_snapshot_t`、`runtime_state_snapshot_t`、`control_config_snapshot_t` 等类型，L1 直接读取 `motor.state` / `motor.cfg` 字段。
- **Protocol 块扩展**：新建 `foc_protocol_handler.c/.h`，统一帧读取（多源轮询）+ 协议解析 + 命令执行 + 参数存储（直接写 motor 字段）。`foc_runtime_output.c/.h` 改名移入为 `foc_protocol_output.c/.h`。
- **Control 块接口简化**：`FOC_ControlOuterLoopStep(motor, pid_cur, pid_spd, pid_angle, sensor, mode, ...)` → `FOC_Control_Run(motor, sensor, dt)`。新增 `FOC_Control_ApplyConfig(motor)` 统一配置应用。
- **L1 编排重写**：`foc_app.c` 删除全部 snapshot/Runtime 引用，直接操作 `g_motor.state/cfg`。协议调用改为 `FOC_Protocol_Process(&g_motor, budget)`。
- **类型系统精简**：`foc_snapshot_types.h` 只保留 `telemetry_policy_snapshot_t`。`foc_runtime_types.h` 删除废弃的 `runtime_fault_code_t`（改用 `foc_fault_code_t`）。删除 `foc_runtime_snapshot.h` 间接头文件。新增 `foc_motor_state_t`、`foc_motor_cfg_t`、`foc_fault_code_t`。

### Added
- `foc_motor_types.h`：新增 `foc_motor_state_t`（运行时状态）、`foc_motor_cfg_t`（控制配置）、`foc_fault_code_t`（故障码枚举）、`FOC_SYSACTION_*`（系统动作枚举）。
- `foc_ctrl_entry.h`：新增 `FOC_Control_Run`（统一外环入口）、`FOC_Control_CurrentLoop`（电流环）、`FOC_Control_ApplyConfig`（配置应用）、`FOC_Control_Init`（控制初始化）。
- `foc_protocol_handler.h`：新增 `FOC_Protocol_Process(motor, budget)`、`FOC_Protocol_Init()`、`FOC_Protocol_Commit(motor)`、`FOC_Protocol_GetTelemetry()`。

### Removed
- `foc_runtime_entry.c/.h`
- `foc_runtime_protocol.c/.h`
- `foc_runtime_fsm.c/.h`
- `foc_runtime_store.c/.h`
- `foc_runtime_output.c/.h`（改名移入 Protocol）
- `foc_runtime_snapshot.h`（间接头文件）
- `runtime_state_snapshot_t`、`control_config_snapshot_t`、`runtime_snapshot_t`
- `runtime_c4_runtime_view_t`、`runtime_c4_params_view_t`、`runtime_c4_states_view_t`
- `RUNTIME_FAULT_*`（改用 `FOC_FAULT_*`）
- `RUNTIME_SYSACTION_*`（改用 `FOC_SYSACTION_*`）

### Fixed
- 修复上电进入故障模式 bug：`foc_app.c` 中 init_check_mask 在 `FOC_MotorInit()` 清零 state 后设置，确保 init 检查通过。

## [1.7.6] - 2026-06-15

### Changed
- **全库注释中文化**：将所有代码逻辑注释（除API文件、配置头以外）翻译为中文，便于国内开发团队（?）理解和维护。
  - L1/L2/L3各层级源文件的英文注释全部替换为中文。
  - API空实现模板(`foc_platform_api_empty.c`)的注释升级为中英双语，细化每个函数参数说明、值范围、实现指南。
  - 涉及文件：`foc_app.c`、`runtime_c1_entry.c`、`runtime_c2_frame_source.c`、`runtime_c3_runtime_fsm.c`、`runtime_c5_output_adapter.c`、`debug_stream.c`、`foc_control_c11_entry.c`、`sensor.c`、`foc_control_c31_actuation.c`、`foc_platform_api_empty.c` 等。

### Added
- **API移植文档完善**：`foc_platform_api_empty.c` 中添加了详细的参数范围、实现注意事项和中断优先级建议。

## [1.7.5] - 2026-06-10

### Added
- **电周期平均动态偏置补偿**：新增 `FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE` 特性，在电流环中通过累计一个完整电周期的相电流平均值，提取并补偿低测采样引入的负载相关零点漂移。
  - 状态全在 `foc_motor_t` 结构体中，无凌散全局变量。
  - 堵转安全：电角度不前进则窗口不闭合，偏置不更新。
  - 电周期检测器仅在 `sensor->encoder_valid` 为真时运行，快速电流环 ISR 路径不触发。
  - 新增宏：`FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE`、`FOC_ELEC_CYCLE_OFFSET_LPF_ALPHA`（默认 0.10f）、`FOC_ELEC_CYCLE_OFFSET_MIN_VALID_CYCLES`（默认 1）。
  - 受影响文件：`foc_cfg_feature_switches.h`、`foc_cfg_init_values.h`、`foc_motor_types.h`、`foc_control_c12_init.c`、`foc_control_c22_current_loop.c`、`sensor.c`（仅 1 行 sanify `encoder_valid = 0` 在 `Sensor_ReadCurrentOnly` 中）。

## [1.7.2-1.7.4] - 2026-05-22

### Fixed
- **电流环问题修复**：修复电流环积分需错误限幅导致的振荡问题，提高了纯电流闭环下的控制效果
- **删除采样漂移抑制**：采样漂移抑制反而会引入误差，直接移除该功能，实测无影响

## [1.7.1] - 2026-05-21

### Fixed
- **电流环 PID 死区+漏积分器组合导致积分振荡**：`FOC_CurrentLoopPIDRun()` 中原有死区抑制（`FOC_CURRENT_LOOP_ERROR_DEADBAND_A = 0.05A`）与漏积分器（`FOC_CURRENT_LOOP_INTEGRAL_SUPPRESS_LEAK = 0.99`）组合形成正反馈循环——误差进入 deadband 后积分指数衰减导致输出下降、误差重新增大、积分再次累加，如此反复振荡且电流无法稳定跟踪目标值。修复方案：删除死区抑制逻辑和漏积分器，改为标准积分累加 + back-calculation 抗饱和（与外环 PID 策略一致）。
- **移除废弃宏**：`FOC_CURRENT_LOOP_ERROR_DEADBAND_A`、`FOC_CURRENT_LOOP_INTEGRAL_SUPPRESS_LEAK` 已无代码引用，从 `foc_cfg_init_values.h` 中删除。

## [1.7.0] - 2026-05-09

### Added
- **参数标定重初始化接入协议**：新增系统子命令 `Y:I`，通过协议触发运行时电机参数重初始化，包括完整的重新标定（方向/极对数/零位）。
  - 新增 `COMMAND_MANAGER_SYSTEM_SUBCMD_REINIT 'I'` 符号定义。
  - L2 运行时链新增 `reinit_pending` 状态位及其传递链：`RuntimeC4_RequestReinit()` → `RuntimeC4_ClearReinit()` → `RuntimeC3_ClearReinit()` → `RuntimeC2_ClearReinit()` → `Runtime_ClearReinit()`。
  - `FOC_App_ReInitMotor()`：停止快速电流环、重置软切换状态，执行初始化标定，完成后清除重初始化标志。
  - `g_reinit_in_progress` 栅保护：在重初始化过程中屏蔽 PWM ISR 和控制循环访问 `g_motor`。
  - 新增 `FOC_App_InitMotorHardware()` 提取公共初始化逻辑。
- **齿槽补偿边界不连续修复增强**：
  - `COGGING_BOUNDARY_Q15_THRESHOLD` 从局部 `#define` 提升为配置宏 `FOC_COGGING_BOUNDARY_Q15_THRESHOLD`（默认值 100，原 1200）。
  - 新增 `FOC_COGGING_BOUNDARY_BLEND_WIN`（默认 3）窗口宽度宏。
  - 新增 Stage 2 基于窗口的边界漂移渐进校正算法，在首尾多点窗口间线性混合消除累积偏差。
- **新增快照结构体字段**：`runtime_state_snapshot_t` 和 `runtime_c4_runtime_view_t` 增加 `reinit_pending` 字段。

### Changed
- **初始化标定功能恢复**：`FOC_INIT_CALIBRATION_ENABLE` 从 `FOC_CFG_DISABLE` 改为 `FOC_CFG_ENABLE`。
- **齿槽标定默认值优化**：
  - `FOC_COGGING_CALIB_GAIN_K`：0.05 → 0.03
  - `FOC_COGGING_CALIB_SPEED_RAD_S`：0.8 → 0.6
  - `FOC_COGGING_CALIB_NUM_PASSES`：2 → 1
  - `FOC_COGGING_CALIB_IQ_A`：0.50 → 0.30
- **电机初始化默认值调整**（配合重新启用标定）：
  - `FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD`：3.1606f → `FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED`
  - `FOC_MOTOR_INIT_DIRECTION_DEFAULT`：`FOC_DIR_REVERSED` → `FOC_DIR_UNDEFINED`
- **齿槽查询优化**：移除冗余的 `mech_angle_rad` 角度归一化循环（调用者保证有效输入）；移除冗余的 `iq_comp` 输出钳位（已由调用者或表边界处理）。
- **移除未使用变量**：`FOC_ControlCoggingLookupIq` 中移除 `float interp`。

### Fixed
- **齿槽补偿边界检测阈值过大**：原局部宏 `COGGING_BOUNDARY_Q15_THRESHOLD = 1200` 过大导致边界不连续检测几乎永远不触发，现降低为 100 并提升为配置宏。
- **协议帧入口时序修复**：`FOC_App_OnPwmUpdateISR` 中 `MotorControlService_RunPwmInterpolationIsr()` 的调用顺序被移动到 `g_fast_current_loop_enabled` 检查之后，避免重初始化期间无关中断调用插值。
- **齿槽查找表角度归一化双重执行**：移除齿槽查询函数中多余的 `mech_angle_rad` wrap 处理，消除与调用者的重复归一化。

### Cleanup
- 注释掉的 `CoggingCalib_SecondOrderDiff` 二阶差分变换代码（规划阶段实验预留）。

## [1.6.0] - 2026-05-07

### Changed
- **文档全面同步与版本提升**：以实际代码为准更新全部文档，开发版本提升至 v1.6.0。
- **修复文档-代码不一致**：
  - `docs/protocol-parameters-bilingual.md`：完整重写，修复软切换 Y/Z 子命令映射反转（代码：Y=闭环阈值, Z=开环阈值）、默认值全面对齐代码、增补缺失的 `P:k` 齿槽标定增益行、补全示波掩码位表、统一实例命令示例。
  - `docs/architecture.md`：移除已废弃的 `FOC_COGGING_INIT_LEARN_ENABLE` 和 `FOC_COGGING_DEBUG_DUMP_ENABLE` 引用。
  - `foc_core/include/LS_Config/foc_compile_limits.h`：修正碰撞警告中的子命令引用（`SOFT_SWITCH_AUTO_OPEN_IQ` → `SOFT_SWITCH_AUTO_CLOSED_IQ`），匹配实际符号 `'Y'` 的归属。
- **版本基线统一**：`CHANGELOG.md`、`NEXT_MISSION.md`、`copilot-instructions.md`、`docs/README.md`、`README.md`、`.clinerules/hywfoc-project-rules.md` 全部对齐至 v1.6.0。

## [1.4.7] - 2026-05-06

### Changed
- **LS_Config 宏头文件重命名与去 `cfg` 语义**：
  - `foc_cfg_symbol_defs.h` → `foc_symbol_defs.h`（非设置项名称去 `cfg` 前缀）
  - `foc_cfg_compile_limits.h` → `foc_compile_limits.h`（同上）
  - `foc_cfg_cogging_table.h` → `foc_cogging_table.h`（数据表文件去 `cfg` 前缀）
  - 以上名称变更同步更新了所有 `#include` 引用点和文档
- **移除死转发头文件**：
  - `L3_Algorithm/protocol_core_types.h`（仅转发 LS_Config/foc_protocol_types.h，已无人引用）
  - `L2_Service/runtime_internal_types.h`（仅转发 LS_Config/foc_runtime_types.h，已无人引用）
- **宏头文件之间零交叉依赖**：
  - `foc_config.h` 保持统一入口角色，内部按序 `#include`，各子文件不互相包含
- **类型定义全部分类收敛至 LS_Config**：
  - 所有枚举/结构体定义已分散至 `foc_math_types.h`、`foc_motor_types.h`、`foc_scheduler_types.h`、`foc_protocol_types.h`、`foc_runtime_types.h`、`foc_snapshot_types.h`

## [1.4.6] - 2026-04-29

### Changed
- **齿槽标定算法完全重写为纯积分预测器法**：替代原有的"开环积分 + 循环差分"方案。
  - 旧算法系统性错误已修正：
    1. **calib_gain_k 被应用两次**：CoggingCalib_Finish() 中 `iq_comp = diff * calib_gain_k` 写入表，
       运行时 `FOC_ControlApplyCoggingCompensation()` 又执行 `motor->iq_target += iq_comp * calib_gain_k`，
       净增益平方衰减（0.08 × 0.08 = 0.0064），导致补偿电流几乎为零。
    2. **旧算法中 per-bin-once 逻辑使用 θ_pred 而非 θ_actual 定 bin**：
       用 θ_pred 定 bin → bin 边界稳定、但 Δθ 测量点与 bin 绑定的"时机"在预测空间而非实际空间，
       导致 Δθ 和 bin 不对齐。现在更正为用 θ_actual 定 bin。
  - **新算法核心原理**：
    - `θ_pred`（预测值）是**纯积分器**：`θ_pred(t) = θ₀ + ∫speed×dt`
    - `θ_pred` 单调递增，永不从传感器重置，驱动开环 SVPWM
    - `θ_actual`（实际值）是传感器读数，两条线**完全独立**
    - `Δθ = wrap_delta(θ_actual - wrap(θ_pred))` 是齿槽导致的真实位置偏差
    - Δθ 按 θ_actual 所属 LUT bin 累积（bin 由实际位置决定）
  - **Finish 阶段改为直接映射**：`iq_comp[i] = -dtheta_avg[i] × DTHETA_SCALE`，
    Δθ → iq 直接转换，不做循环差分。存储格式为 Q15(0.005A LSB)，运行时 `gain_k`
    提供在线强度调节（gain_k=0 关闭，gain_k=1 按表值原样输出）。
  - 新增 `FOC_COGGING_CALIB_DTHETA_SCALE` 宏（默认 50.0），Δθ(rad)→iq(A) 映射系数。
  - 新增边界不连续修复 `CoggingCalib_FixBoundaryDiscontinuity()`：
    检测并平滑 bin[0] 与 bin[N-1] 的 0/2π 接缝突变。
  - 新增 IQR 离群值剔除（对 dtheta_avg 在 Finish 中执行），防止单次毛刺污染补偿表。
- `FOC_ControlApplyCoggingCompensation()` 保留 `gain_k` 作为运行时强度系数（一次应用）。
- `FOC_COGGING_CALIB_GAIN_K` 保持可调（协议命令 P:k），默认值 1.0。

## [1.4.5] - 2026-04-29

### Changed
- 齿槽标定算法根本性修复：电角度/机械角度混用导致标定表完全错误。
  - **根因**：标定算法自维护了一个电角度预期值 `g_calib_expected_elec_angle`，
    但在 SCAN 阶段计算位置误差 Δθ 时，用 `θ_elec / pole_pairs` 反推机械角度预期值。
    由于开环驱动中的 `Math_WrapRad()` 在 `pole_pairs=7` 时会将完整机械角度旋转
    错误地截断到 `[0, 2π/7)` 范围内，导致 Δθ 记录的是电角度 wrap 的假偏差而非真实齿槽扰动。
  - **修复方案**：标定算法全线改用机械角度作为主参考量：
    - `OpenLoopDriveStep` 中自维护机械角度（单调累积，不 wrap），再从机械角度派生出电角度用于 SVPWM 驱动
    - `θ_mech_drive = θ_mech_expected`（直接），`θ_elec_drive = θ_mech_wrapped * pole_pairs`
    - SCAN 阶段的 Δθ 计算直接使用 `mech_angle_sensor - mech_angle_expected_wrapped`
    - 消除了旧算法中 `Math_WrapRad(g_calib_expected_elec_angle)` 在除以极对数时产生的截断误差
  - **影响**：标定表从此反映真实的机械位置齿槽偏差，补偿前馈量级和相位均正确。
- 新增离群值剔除逻辑（Step 2b）：在计算差分表之前，对 dθ 的循环差分
  `diff[i] = dtheta_avg[i+1] - dtheta_avg[i]` 执行 1.5×IQR 准则检测，
  异常值用相邻差分的均值替代。避免单个传感器的毛刺或摩擦瞬态污染整张补偿表。
- 调整默认标定参数：
  - `FOC_COGGING_CALIB_SPEED_RAD_S`：0.5 → 1.5（更快扫过齿槽，减少摩擦非线性影响）
  - `FOC_COGGING_CALIB_NUM_PASSES`：2 → 6（更多圈平均，提高信噪比）
  - `FOC_COGGING_CALIB_IQ_A`：0.25 → 0.50（更大的标定电流使齿槽效应更显著）
  - `FOC_COGGING_CALIB_SETTLE_REV`：1 → 2（更长的平衡建立时间）
- 新增 `FOC_COGGING_LUT_RAD_LSB_A` 宏定义（0.0002），为未来以弧度为单位存储补偿表预留。

## [1.4.4] - 2026-04-28

### Changed
- 齿槽标定算法完全重写为纯开环位置偏差法：替代旧的 iq 测量值累积方法。
  - 标定期间电机纯开环驱动（自维护电角度 + Uq = iq * R），绕过外环 PID 和电流环 PID。
  - SETTLE 阶段：电机先旋转 N 圈建立动平衡。
  - SCAN 阶段：每步读取传感器角度，计算与预期开环位置的偏差 Δθ，按实际机械角度映射到 LUT bin 累加。
  - FINISH 阶段：多圈平均 → 去DC → 循环差分 → iq 补偿表 → 加载到电机。
  - 新增运行时增益系数 `FOC_COGGING_CALIB_GAIN_K`，可通过协议命令 `P:k` 在线修改。
  - 新增软开关状态保存/恢复机制：标定开始保存、结束时恢复。
  - 标定期间显式调用 `FOC_ControlSetCoggingCompUnavailable()` 禁用齿槽补偿。
- 清理废弃宏：移除 `FOC_COGGING_CALIB_SETTLE_CYCLES`（替换为 `FOC_COGGING_CALIB_SETTLE_REV`）；移除 `FOC_COGGING_LEARN_SAMPLE_COUNT`、`FOC_COGGING_LEARN_MIN_VALID_PERCENT`、`FOC_COGGING_LEARN_ERR_TO_IQ_GAIN_A_PER_RAD`、`FOC_COGGING_LEARN_MAX_RETRIES`、`FOC_COGGING_LEARN_STEP_SETTLE_MS` 等旧学习算法宏。
- 新增 `FOC_COGGING_CALIB_ENABLE` 编译期约束检查。
- 协议文档更新：`P:k` 的参数说明和裁剪宏归属。

## [1.4.3] - 2026-04-27

### Changed
- 齿槽补偿重构（v1.4.2 中初步提交，v1.4.3 完成清理）：将旧"初始化阶段自动标定"策略（`FOC_COGGING_INIT_LEARN_ENABLE`）替换为"运行时用户手动触发标定 + 补偿"新机制（`FOC_COGGING_CALIB_ENABLE`）。
  - 新增运行时标定状态机，在补偿执行步骤（`FOC_ControlCompensationStep`）中通过 `FOC_CoggingCalibProcess()` 驱动，以 iq 计算值波动为输入。
  - 初始化不再执行齿槽标定，仅输出齿槽表状态（已定义/空）。
  - 新增协议命令：`Y:G`（触发标定）、`Y:D`（串口输出补偿表），通过系统命令通道接入。
  - 新宏 `FOC_COGGING_CALIB_ENABLE` 控制标定功能，旧宏 `FOC_COGGING_INIT_LEARN_ENABLE` 设为默认禁用以保持兼容（默认关闭）。
  - 补偿仅在 `FOC_COGGING_COMP_ENABLE` 开启且齿槽表非空时生效。
- 移除已废弃的初始化标定相关函数和符号：`FOC_CoggingInitLearn`、`FOC_CoggingInitStart`、`FOC_CoggingInitProcess`、`FOC_CoggingInitIsBusy`、`FOC_COGGING_INIT_LEARN_ENABLE`、`foc_cogging_init_learn_t`。
- 清理齿槽补偿模块中残留的 `FOC_COGGING_INIT_LEARN_ENABLE` 条件编译分支和对应的初始化调用路径。
- 清理 `foc_cfg_init_values.h` 中不再使用的初始化标定相关默认值。
- 编译零 warning 通过（1 条 linker warning 为 scatter 文件固有，不计）。

## [1.4.2] - 2026-04-27

### Changed
- 齿槽补偿重构（v1.4.2 初步提交包含标定状态机 + 协议命令框架，后续在 v1.4.3 完成废弃代码清理）。
- 电流环 PID 抗饱和策略从 back-calculation（反算积分限幅）替换为 conditional integration（条件积分）：饱和冻结 + 积分钳位安全网。新策略在输出饱和且误差同向加深时回滚积分增量，并附加 `|integral| ≤ |out_max/ki|` 硬钳位。相比原 back-calculation 方法释放了积分器在瞬态响应中的建立能力，更适合电流环小 KP + 大 KI 的参数风格，阶跃响应更快且无过冲。
- 删除 Ki 低电流缩放死代码：`FOC_CURRENT_LOOP_KI_LOW_CURRENT_START_A`、`END_A`、`SCALE` 三项宏及其消费函数 `FOC_CurrentLoopComputeKiScale`，默认值组合 (start=0.00, end=0.0, scale=1.0) 导致函数恒返回 1.0f，无实际效果。同时将 `FOC_CurrentLoopPIDRun` 中 `ki_effective` 局部变量精简为直接使用 `pid->ki`。
- 修复 Clarke 变换 β 系数：`Math_ClarkeTransform` 中 β = (b-c) * √3/2 改为 β = (b-c) / √3，消除 Park 变换后 Iq 的 2 倍频电角度正弦波动和 25% 直流偏置，同时修复 Id 的串扰问题。
- 精简电流环冗余电压钳位：`FOC_CurrentControlClosedLoopStep` 中移除 `Math_ClampFloat(uq_cmd, -voltage_limit, voltage_limit)`，因 PID 输出限幅和执行层缩比已提供双重保护。

## [1.4.1] - 2026-04-25

### Fixed
- 修复电流环 iq 与电角度周期性同步波动问题：根因为 `Sensor_ReadADC()` 中共模抑制（CMR）因 C 相软件重构（C = -(A+B)）而完全失效，三相加和恒为零，无法消除 ADC 直流偏置。移除无效的共模抑制逻辑，添加在线 DC 偏置漂移跟踪器（一阶 LPF，α=5e-5, fc≈0.1Hz），持续跟踪 A/B 相的残差直流，在保留基频 AC 电流的前提下消除与电角度同频的 iq 波动。

### Documentation
- 更新版本基线至 v1.4.1，同步 `docs/README.md`、`copilot-instructions.md`、`NEXT_MISSION.md`、`README.md`。

### Changed (Previous)
- 将 `foc_control_c25_cfg_state` 提升至 C13 层（`foc_control_c13_cfg_state`），消除跨层依赖：C25 无任何 L3 内部依赖，仅操作结构体字段和宏配置，提升为 C13（配置状态管理），紧接 C12 构成「入口 → 初始化 → 配置」语义链。
- 清理 `c11_entry.h` 接口污染：删除 11 个属于 c13_cfg_state 的函数声明（`FOC_ControlConfigResetDefault`、5个 fine-tuning setter、4个 soft-switch setter、`FOC_ControlSetCoggingCompEnable`、`FOC_PIDInit`）。c11_entry.h 仅保留 c11_entry.c 实现的 7 个入口函数，名实相符。
- 更新 `motor_control_service.c`：直接 `#include "foc_control_c13_cfg_state.h"`，不再依赖 c11_entry.h 透传。
- 删除旧文件 `foc_control_c25_cfg_state.c/.h`。

## [1.4.0] - 2026-04-23
### Changed
- 修复部分状态机BUG，恢复电压限幅逻辑
- 调整状态机组织结构，修复LED闪烁不正常以及错误恢复不正常的BUG
- 完善欠压保护功能，监测VBUS电压并在过低时进入FAULT状态
- 大量文档未更新

## [1.3.6] - 2026-04-21

### Changed
- Completed structure-convergence closure as the current baseline: L2 production runtime pipeline is fixed at `runtime_c1_entry -> runtime_c2_frame_source -> runtime_c3_runtime_fsm -> runtime_c4_runtime_core -> runtime_c5_output_adapter`.
- Aligned cogging compile guards in `foc_control_c24_compensation`: when cogging feature is trimmed off, runtime call path now exits before referencing feature-scoped helper functions.
- Extended protocol state text mapping with cogging state entry (`S:G`) under `FOC_PROTOCOL_ENABLE_COGGING_COMP` guard.

### Documentation
- Rewrote `docs/architecture.md` as current-code SSOT using actual `L1/L2/L3/L41/L42/LS` paths and current control/runtime chain naming.
- Rewrote `docs/development.md` to remove outdated temporary-plan and legacy numbering references, and synced it with current compile/macro-governance rules.
- Updated protocol bilingual guide defaults/ranges/path references to current `foc_cfg_*` values and documented current cogging protocol-state scope.
- Synced mission/workflow docs (`NEXT_MISSION.md`, `.github/AGENTS.md`, agent docs, instance software README) to current directory structure and version baseline.

## [1.3.4] - 2026-04-17

### Changed
- Completed P1 architecture-prep inventory and consolidated file-level boundary facts (control/protocol split, config ownership, platform API minimal set, timing impact) into the SSOT architecture document.
- Completed P2 structure-clarification pass for runtime interrupt lifecycle: application init now explicitly keeps control runtime IRQs disabled until runtime start, preserving init/runtime execution-path separation.
- Added compile-time timing constraints for PWM/current-loop divider legality to prevent invalid non-integer or zero-divider configurations.
- Completed physical directory convergence for L1/L2/L4-1 source layout: runtime orchestration/service modules now build from `core`/`service`, and math/LUT modules build from dedicated `math` directories.
- Split `command_manager` into focused submodules (`command_manager_dispatch`, `command_manager_diag`) and split `foc_control` auxiliary strategies into dedicated modules (`foc_control_softswitch`, `foc_control_compensation`).
- Synced EIDE project sources and builder source list with the refactored module paths and newly introduced source files.
- Completed trig LUT unification for init calibration path in `foc_control_init`: replaced direct trig-library usage with `FOC_MathLut_Sin` / `FOC_MathLut_Atan2` to align init/runtime math behavior.
- Fixed state-to-control propagation after L2/L3 decoupling: writing current soft-switch enable state now marks runtime params dirty so L1 applies it in unified runtime config refresh.

### Documentation
- Added P3 regression-prep records in `docs/development.md`, including build result, resource usage summary, startup calibration path check, and runtime SVPWM interpolation path check.
- Updated `NEXT_MISSION.md` to mark P1/P2/P3 as completed and recorded this cycle's closure summary.
- Updated `docs/architecture.md` structure tree and layer mapping to reflect `math` directory extraction and service/algorithm internal module splits.

## [1.3.2] - 2026-04-15

### Changed
- 将结构/依赖文档收敛到单一事实源 `docs/architecture.md`，并删除重复的结构文档入口，避免兼容式跳转维护。
- 重写并中文化仓库流程治理文档，统一边界口径：`AI_INITIALIZATION.md`、`copilot-instructions.md`、`.github/AGENTS.md`、`.github/WORKFLOW_CHECKLIST.md`、`.github/DOCUMENTATION_STRUCTURE.md`。
- 重构 `NEXT_MISSION.md` 为面向结构重排的分阶段任务单，并将"待确认问题"固化为"已确认决议"。
- 清理失效文档入口与过期工作流引用，修复 `docs/README.md`、`.github/*`、`README.md` 的断链与版本口径。

### Documentation
- 在 `docs/development.md` 增补"编译/调试经验沉淀"条目，并与仓库长期记忆保持同措辞同步。

## [1.3.1] - 2026-04-14

### Added
- Added optional cogging compensation framework with feature switches, runtime status tracking, LUT load API, and low-speed lookup injection in speed/speed-angle outer-loop paths.
- Added startup cogging source selection logic: static profile first, init-time learning fallback, and no-source graceful disable path with diagnostics.
- Added init-time cogging learning routine with finer lock sampling, table quantization, and structured debug dump output.

### Changed
- Refined motor init calibration stepping strategy to use finer electrical-angle subdivision for direction/pole-pair estimation.
- Added new compile-limit checks for cogging-related feature switches and init-calibration trimming constraints.

## [1.3.0] - 2026-04-13

### Changed
- Reformatted current-loop anti-noise and soft-switch config macros in `foc_cfg_init_values.h` with aligned value columns for easier scanning.
- Added concise inline `//` comments for soft-switch defaults, anti-noise deadband/integral/Ki-scaling macros, and blend time constants.
- Added explicit `FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_MIN_SEC` default definition for soft-switch blend lower-bound configuration.

### Documentation
- Updated repository baseline/version references to `v1.3.0` and moved active mission target to `v1.3.1`.
- Updated protocol documentation to describe both soft-switch blend time-constant macros (`TAU_MIN` and `TAU_DEFAULT`).

## [1.2.0] - 2026-04-11

### Changed
- Completed high-rate execution-path refactor: PWM update ISR callback is now the canonical interpolation/current-loop path and protocol parsing is fully polling-based (`IsFrameReady + ReadFrame`).
- Completed outer/inner loop decoupling baseline: outer loop stays on scheduler task path, while current-loop execution is driven by PWM ISR cadence.
- Updated ADC sampling pipeline to support distinct slow/fast averaging windows and moved average-count boundary normalization into ADC driver layer.
- Added 16-bit trig lookup module (`0.001 rad` step, symmetry reconstruction) under special dependency layer and switched SVPWM trigonometric hotspots to LUT path.
- Improved macro clipping semantics for current-loop disable mode: ISR keeps interpolation-only behavior and skips fast-loop compute chain.
- Added optional first-order `iq` LPF switch/alpha configuration in current-loop path for noise suppression experiments.
- Cleaned instance platform API adapter responsibility by removing non-adapter helper logic from `foc_platform_api`.

### Documentation
- Updated release baseline references and mission document targets to align with `v1.2.0` snapshot delivery.

## [1.1.0] - 2026-04-10

### Changed
- Added independent speed reference parameters for speed-only and speed-angle control paths, including dedicated subcommand mapping and runtime getters.
- Updated speed-only runtime path to consume the dedicated speed-only reference value instead of reusing the speed-angle parameter.
- Added circular-domain angle LPF processing path in sensor readout and kept it behind compile-time feature switches.
- Unified sensor filter compile-time trimming structure so declaration/call/definition guards are aligned for Kalman and angle LPF paths.
- Updated sensing defaults to use angle LPF while keeping current and angle Kalman paths disabled by default for the current hardware baseline.
- Expanded command parameter compile limits and defaults to include signed speed-only range and related initialization values.

### Documentation
- Updated protocol bilingual parameter documentation for speed-only and speed-angle subcommand mapping and examples.
- Updated mission planning document to set v1.1.0 as baseline and define next-stage algorithm priorities (cogging feedforward and current-loop tuning).
- Refreshed project/hardware readme descriptions for current repository and instance status.

## [1.0.0] - 2026-04-09

### Changed
- Extended command frame format to `a<driver_id><cmd><subcmd><param>b` with protocol-level address filtering.
- Added driver-id validation range (`0x32-0x7E`) and broadcast support (`0xFF`); non-targeted valid frames are silently dropped.
- Kept instance platform timing and indicator constants as local macros in platform implementation to avoid direct dependency on library config headers.
- Decoupled instance I2C driver from `foc_config.h` by using local timeout and unlock loop macros in `i2c0`.

### Documentation
- Updated protocol bilingual guide and GD32F303 instance adaptation examples to the new driver-id command format.
- Reworked root `README.md` as external-facing Chinese project introduction with unified naming baseline (HYWfoc / 何易位FOC).
- Reworked instance `examples/GD32F303_FOCExplore/README.md` into platform usage guide and aligned links to detailed instance docs.
- Updated `docs/README.md` and `NEXT_MISSION.md` (P3.5) for document-role boundary and naming/task consistency.
- Updated repository workflow and initialization docs to repository-level governance (`AI_INITIALIZATION.md`, `copilot-instructions.md`, `docs/development.md`, and workflow rules in `docs/engineering/dev-guidelines/rules/`).
- Added third-party dependency and license notice inventory in `THIRD_PARTY_NOTICES.md`.

## [0.4.0] - 2026-04-03

### Changed
- Reworked I2C wait-timeout strategy to loop-budget polling in `i2c0` driver path, removing runtime dependency on millisecond tick waits for flag/STOP handling.
- Reworked I2C unlock timing to bounded busy-loop delays (non-DWT) for SCL pulse and STOP generation path.
- Added control-loop FAULT-state early gate in app runtime path to skip sensor refresh and avoid repeated encoder I2C access in invalid hardware states.
- Added debug-stream FAULT-state gate to suppress semantic/osc periodic output while fault remains active.
- Kept fault recovery path on command channel (`F+C`) for runtime reinit without power cycle.

### Documentation
- Updated root/docs version baseline to `v0.4.0`.
- Aligned scheduler-rate descriptions to `1kHz/100Hz/200Hz/1Hz`.
- Corrected protocol document channel mapping to current implementation (`USART1` unified TX feedback + debug text).
- Synced protocol parameter defaults with code (`angle_speed_rad_s`, semantic enable default, speed PID defaults).
- Rewrote mission plan for next iteration as architecture/organization/documentation-only scope (no business-logic changes).

### Added
- Added trim-able undervoltage protection feature switch (`FOC_FEATURE_UNDERVOLTAGE_PROTECTION`) in startup config.
- Added platform placeholder API `FOC_Platform_UndervoltageProtect(vbus_voltage)` with no-op implementation for current hardware.
- Added FOC-side undervoltage fault logic path in control loop with trip/recover thresholds and fault-state propagation via command manager.

### Documentation
- Updated project README to describe the undervoltage protection placeholder path and current hardware limitation.

## [0.3.8] - 2026-04-02

### Changed
- Completed communication path refactor: multi-source frame aggregation is now implemented in L3 `protocol_parser` with trigger-source-priority behavior.
- Reworked platform communication APIs in `foc_platform_api` to one global init plus four explicit source read/trigger interfaces.
- Reorganized configuration architecture into the `foc_cfg_*` split (`symbol_defs`, `feature_switches`, `init_values`, `compile_limits`) and converged includes through `foc_config.h`.
- Unified scheduler/control timing semantics using dedicated control-rate macros and divider-based callback gating.
- Unified project-wide mathematical constants into `foc_shared_types.h` and aligned control/SVPWM/sensor math usage.

### Removed
- Removed legacy L4 communication multiplexer module (`Utilities/USART/comm_frame_mux.[ch]`).
- Removed deprecated legacy `foc_config_*` header set in favor of the `foc_cfg_*` split.

### Documentation
- Updated architecture and dependency-tree documents to reflect L3 communication aggregation and current L4 module set.
- Refreshed mission baseline to v0.3.8 and rewrote next-iteration objectives.

## [0.3.6] - 2026-03-28

### Changed
- Completed P1.2 scheduler standardization: task-rate naming is unified to `FOC_TaskRate_t`, scheduler dividers are configured through centralized config macros, and old compatibility aliases were removed.
- Continued P1.3 macro/config convergence: domain configuration headers are connected through `foc_config.h`, and command/protocol/debug/runtime defaults were further migrated from implementation files into config headers.
- Advanced P1.4 header-boundary convergence: public headers were slimmed to reduce cross-layer exposure, and includes were moved to source files where implementation ownership belongs.
- Completed feature-cut macro introduction for diagnostics and control algorithms: diagnostics output/statistics can be trimmed independently, and control algorithm build set supports `FULL`, `SPEED_ONLY`, and `SPEED_ANGLE_ONLY` with explicit compile-time guards.

### Documentation
- Refreshed mission planning document for `v0.3.6`: removed completed tasks and reordered unfinished tasks by priority and execution dependency.
- Updated project README version section to reflect current delivery baseline and focus.

### Changed
- Moved scalar math helpers and constants into `math_transforms` (`Math_WrapRad`, `Math_WrapRadDelta`, `Math_ClampFloat`, `MATH_PI`, `MATH_TWO_PI`).
- Split control-layer responsibilities by introducing `foc_control_init` for motor initialization/calibration while keeping `foc_control` focused on runtime control algorithms.
- Added L2 internal bridge header `foc_control_internal.h` to share control-only helpers with init/calibration module.

## [0.3.5] - 2026-03-27

### Added
- Added command and runtime-state management module (`command_manager`) with unified command dispatch, parameter read/write, and init/runtime diagnostics.
- Added protocol parser module (`protocol_parser`) with frame extraction, structured command cache, parse-pending trigger model, and single-char feedback path.
- Added dual debug stream module (`debug_stream`) supporting semantic low-rate output and high-rate osc payload output.
- Added USART frame multiplexer (`comm_frame_mux`) with source mask and round-robin arbitration.
- Added protocol and parameter bilingual reference documentation (`docs/protocol-parameters-bilingual.md`).

### Changed
- Completed P0 communication base: USART1/USART2 moved to DMA RX + IDLE event frame capture with double-buffer strategy and DMA TX output path.
- Reworked platform communication API to transport-agnostic frame interfaces (`FOC_Platform_CommInit`, `FOC_Platform_ReceiveFrame`, `FOC_Platform_SetCommRxTriggerCallback`).
- Updated app loop communication path to non-blocking pending-poll processing with one-frame-per-step budget.
- Updated architecture and structure docs to reflect new module boundaries and communication flow.

### Fixed
- Unified invalid-command and invalid-parameter error accounting/reporting path.
- Fixed frame parsing robustness on mixed DMA chunks by extracting valid head-tail frame window before parse.

### Notes
- Build and flash validation passed on GD32F30X_CL target in this release cycle.

## [0.3.3] - 2026-03-25

### Changed
- Completed speed/speed-angle decoupling updates and stabilized transition behavior in the outer-loop control path.
- Removed USART protocol/loopback coupling and migrated USART1/USART2 TX paths from TBE interrupt sending to DMA-based sending.
- Simplified USART interrupt paths to focus on RX callback and buffer handling.

### Fixed
- Removed ADC debug pin usage on PB11 to eliminate conflict with USART2 RX and avoid pseudo-UART interrupt noise.
- Restored ADC init/start in platform sensor-input initialization after resolving the PB11 conflict.

## [0.3.2] - 2026-03-24

### Changed
- Finalized layered refactor for scheduler ownership: `ControlScheduler_Init` and task callback registration are now owned by application layer, while platform layer only encapsulates control-tick source init/bind/start.
- Completed sensor-path decoupling: sensor acquisition now reads raw device values via level-3 platform API wrappers, and level-2 `sensor` keeps processing/filter/structuring responsibilities.
- Introduced shared type hub `foc_shared_types.h` and centralized cross-module public types (`sensor_data_t`, `kalman_filter_t`, `foc_motor_t`, `foc_pid_t`, `foc_torque_mode_t`, scheduler callback/rate types).
- Renamed IRQ forwarding implementation source from `foc_irq_api_gd32.c` to `foc_irq_api.c` and synchronized project/docs references.
- Updated API and IRQ naming toward functional semantics and reduced peripheral-oriented names at upper layers.

### Fixed
- Resolved scheduler/platform coupling compile errors caused by header-only include removal by assigning lifecycle boundaries instead of adding superficial include fixes.
- Resolved scheduler/platform coupling compile errors caused by header-only include removal by assigning lifecycle boundaries instead of adding superficial include fixes.
- Restored build consistency after refactor by aligning callback binding path with the new control-tick source API.

### Notes
- Hardware build and flash verification passed for this release.

## [0.3.0] - 2026-03-23

### Changed
- Completed low-speed sensored FOC functional path for this stage: startup calibration, torque/current control entry, position/speed loop framework, and TIMER2-driven SVPWM interpolation.
- Refactored speed-loop implementation to follow position-loop principle by integrating speed reference (rad/s) into accumulated angle reference per 1kHz cycle.
- Unified direction semantics to signed `1/0/-1` across control APIs and internal calculations.
- Consolidated control dataflow around torque/current loop reuse to reduce duplicated logic between outer loops.

### Fixed
- Corrected direction mapping inconsistencies that could cause effective reverse behavior in some paths.
- Removed unstable direct speed-PID path and cleaned obsolete speed-state fields/code.

### Notes
- Current precise current control performance is still the primary remaining gap for the next mini-version.
- Build and flash verification passed in hardware validation for this release.

## [0.2.7] - 2026-03-23

### Changed
- Added SVPWM linear interpolation path driven by TIMER2 update interrupt callback to improve duty-cycle continuity.
- Added current-loop feedforward + PID implementation on `iq` channel with low-current PID bypass (`|iq_ref| < 0.1A`).
- Added speed-loop API (`FOC_SpeedControlStep`) as cascade outer loop over torque/current loop.
- Updated main control integration with speed-loop PID initialization and runtime entry.

### Fixed
- Unified motor direction semantics to signed `1/0/-1` across definitions, APIs, and direction-related calculations.
- Corrected direction-mapping inconsistency that caused effective sign inversion in some control paths.

### Notes
- Current hardware validation shows improved behavior with TIMER2-driven interpolation compared with prior interrupt path.
- Speed-loop gains are initialized with conservative defaults and may require hardware tuning.

## [0.2.6] - 2026-03-23

### Changed
- Added position-loop framework on top of torque control: introduced `foc_angle_loop_t` and `FOC_AngleControlStep()` to generate torque-current reference from angle error.
- Added accumulated mechanical position state in motor model (`mech_angle_accum_rad`) with wrap-aware incremental update for multi-turn position control.
- Updated main control integration with angle-loop PID initialization and retained torque-loop runtime as default path for staged validation.
- Updated next-mission direction to focus on control-effect optimization (current-loop feedforward PID and low-current open-loop fallback).

### Fixed
- Improved startup calibration flow ordering to avoid angle-state discontinuity by finalizing zero-angle lock after direction/pole-pair estimation.

### Notes
- Basic FOC functional verification is completed in current hardware tests.
- Next mini-version will focus on control performance optimization and parameter robustness.

## [0.2.5] - 2026-03-21

### Changed
- Added torque-control APIs in `foc_control` with mode switch support (open-loop and current-loop mode).
- Refactored control path to absolute-voltage semantics: `vbus_voltage` as global limit, `set_voltage` as user clamp, `ud/uq` as absolute voltage commands.
- Updated SVPWM update interface to use explicit `voltage_command` amplitude input.
- Added mechanical-angle to electrical-angle mapping in FOC based on direction, pole-pairs, and calibrated zero electrical reference.
- Decoupled sensor acquisition from control algorithm by passing measured current and mechanical angle into FOC control APIs.

### Fixed
- Corrected torque-axis command routing so torque command is applied on q-axis in control path.
- Fixed open-loop torque branch polarity/axis assignment issue in control-step implementation.
- Reduced electrical-angle wrap ambiguity by applying modulo-based single-electrical-cycle mapping in mechanical-to-electrical conversion.

### Notes
- Open-loop torque control is validated in current hardware tests.
- Closed-loop behavior is logically correct but still requires parameter tuning for final dynamic performance.

## [0.2.4] - 2026-03-21

### Changed
- Refactored FOC motor model naming to explicit rad-based semantics (`electrical_phase_angle`, `mech_angle_at_elec_zero_rad`).
- Unified encoder angle processing and UART debug output to radians across Sensor/AS5600/FOC paths.
- Added startup calibration entry `FOC_CalibrateElectricalAngleAndDirection()` and invoked it at the end of `FOC_MotorInit()`.
- Reworked zero electrical angle measurement to locked static sampling at electrical angle 0, with settle and multi-sample circular averaging.
- Reworked direction and pole-pair estimation to one-way stepped electrical-angle sampling with larger total probe span.

### Fixed
- Removed mechanical/electrical angle mixed-domain operations from calibration flow.
- Corrected open-loop output to use tracked electrical phase command instead of fixed zero-angle forcing.
- Calibration now updates zero angle, direction, and pole pairs only when each field is undefined.

## [0.2.3] - 2026-03-20

### Changed
- Removed redundant 1kHz modulo check in timer callback dispatch path.
- Updated development guidance for where null checks are required versus redundant.
- Refactored open-loop FOC motor model around simplified `foc_motor_t` parameter set.
- Updated SVPWM init API to `SVPWM_Init(freq_kHz, deadtime_percent)` and passed bus voltage explicitly in `SVPWM_Update`.
- Updated `main.c` control path to use current open-loop call pattern (`FOC_OpenLoopStep(&g_motor, turn_speed_hz)`).
- Adjusted ADC sample trigger offset from 94% to 96%.

### Fixed
- Reworked I2C0 timeout handling so flag wait timeout now triggers internal bus recovery.
- Replaced unbounded STOP wait loops with timeout-protected wait logic.
- Corrected I2C unlock sequence to operate on both SCL/SDA in open-drain mode and generate a valid STOP.
- Removed misplaced application-level `I2C0_Unlock()` call from UART debug path; recovery is now centralized in I2C driver.
- Added neutral-current compensation step in sensor current processing path.

## [0.2.1] - 2026-03-18

### Added
- New SVPWM module under Application layer with six-sector normalized duty output API
- USART2 protocol-oriented send/read interfaces with frame assembly placeholders
- Temporary issue tracking document for planning the next mini-version

### Changed
- Main control loop now runs a 1kHz SVPWM simulation update path for waveform observation
- UART oscilloscope debug output switched to direct SVPWM three-phase duty telemetry
- ADC startup behavior improved by pre-filling initial sample buffer
- Development workflow rules updated to default delayed commits and main-branch development

### Fixed
- USART1 API type safety and const-correctness for byte/string send functions
- Initial SVPWM sector timing and duty normalization consistency

## [0.3.0-legacy] - 2026-03-13

### Added
- ADC current sampling implementation (PA6/PA7 synchronous sampling)
- DMA-based ADC data transfer
- Current calculation with zero offset calibration
- Multi-language development guidelines (English/Chinese)
- Comprehensive documentation reorganization
- Semantic versioning and branching strategy

### Changed
- Renamed `.cursor/` to `dev-guidelines/` for generalization
- Reorganized documentation structure under `docs/`
- Simplified main.c by removing test loops for production readiness
- Optimized ADC module error checking for embedded efficiency
- Updated development workflow to support AI-assisted development

### Fixed
- Compiler warnings (newline in header, implicit function declarations)
- Code style consistency across modules
- Resource usage optimization for embedded constraints

### Removed
- Test functions from main loop (moved to conditional compilation)
- Redundant error checking in time-critical paths

## [0.2.0] - 2026-03-11

### Added
- PWM dead time implementation
- Timer1 algorithm callback system
- Hardware I2C driver for GD32F30x
- AS5600 magnetic encoder driver
- LED blink callback encapsulation

### Changed
- Timer modules now use parameterized initialization
- Improved module boundaries and API consistency

## [0.1.0] - 2026-03-10

### Added
- Basic project structure with GD32F303CC
- Timer-based multi-rate scheduling framework
- USART1 interrupt-driven communication
- PWM output for 3-phase motor control
- LED status indication
- Initial development guidelines and rules

