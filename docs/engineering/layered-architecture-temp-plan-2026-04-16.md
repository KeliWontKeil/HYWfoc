# 分层相邻单向依赖临时方案（评估版）

日期：2026-04-16  
状态：临时方案（执行期参考文档；保留至 M11 完成并验收后再归档）

2026-04-17 修订说明：新增“重开收口约束（R 版）”，本次修订覆盖此前“兼容壳可短期保留”的执行口径。

## 1. 目标与约束

- 目标依赖规则：仅允许相邻层单向依赖。
- 允许方向：L1 -> L2 -> L3 -> L4。
- 禁止项：跨层调用（例如 L1 -> L3、L2 -> L4）和任意逆向调用。
- 层内依赖策略：优先采用 `.c/.h` 文件分层形成单向调用链；L2 允许少量受控 inline（模块内私有且可检索），L3 除 `L41_Math` 外完全去业务 inline。
- 裁剪职责：L3 负责算法裁剪（滤波、补偿、控制子算法），L2 负责功能裁剪（启停某功能链、协议能力开关装配）。

## 2. 现状主链路梳理（基于源码）（已过时，请勿当作现状）

### 链路 A：慢速控制任务链（当前）

- 入口：L1 调度触发 Motor_Control_Loop。
- L1 内动作：读传感器、故障判断、协议参数应用、直接调用外环算法。
- 关键调用路径：
  - L1 Motor_Control_Loop -> L3 FOC_ControlOuterLoopStep
  - L1 FOC_App_RunControlAlgorithm -> L3 FOC_ControlOuterLoopStep
  - L1 FOC_App_ApplyPidRuntimeParams -> L3 FOC_ControlSet* 系列
- 评估：L1 直接触达 L3，违反相邻层边界（应改为经 L2 编排函数）。

### 链路 B：PWM ISR 快速电流环链（当前）

- 入口：L1 PWM 中断回调 FOC_App_OnPwmUpdateISR。
- L1 内动作：SVPWM 插值、可选采样、直接调用 L3 FOC_CurrentControlStep。
- L3 动作：计算电流环并调用 SVPWM Runtime/Direct 更新。
- 评估：L1 直接调用 L3 电流环核心，边界不符合目标模型。

### 链路 C：协议执行链（当前）

- 入口：L1 FOC_App_Loop 中直接调用 ProtocolParser_Process + CommandManager_Process。
- L2 动作：解析并写入参数池 g_params/g_states，置 dirty 标志。
- 回写动作：L1 在控制循环中读取大量 CommandManager_Get*，再直接写入 L3。
- 评估：协议参数应用链跨过 L2 组织边界，形成 L1 <-> L2 <-> L3 的交织调用。

### 链路 D：监控上报链（当前）

- 入口：L1 monitor task 触发 DebugStream_Process。
- L2 动作：读取 CommandManager 状态并经 PAL 输出调试流。
- 评估：L2 内部仍直接依赖 L42 PAL，后续若严格执行相邻规则会冲突。

## 3. 现状核心问题总结

- L1 职责过重：调度、协议驱动、参数下发、算法调用、平台操作都在 L1。
- L2 缺少“算法组织门面”：没有专门的 L2 控制编排函数承接 L1 请求。
- L2 与 L3 都存在层内耦合与直接硬件依赖，边界不清晰。
- 裁剪粒度错位：协议宏多在命令读写分支，参数读取 API 与上游调用链仍常驻。

## 4. 目标结构方案（程序/函数块组织）

## 4.1 目标主链路（建议）

- 控制链：
  - L1 TaskTick_Control()
  - -> L2 MotorControlService_RunControlTask()
  - -> L3 SensorAlgo_ReadSnapshot()
  - -> L3 MotionAlgo_OuterLoopStep()
  - -> L3 CurrentAlgo_PublishTarget()
  - -> 返回 L2 汇总结果
  - -> L1 仅做状态流转/调度决策
- 快速链：
  - L1 ISR_PwmUpdate()
  - -> L2 MotorControlService_RunFastCurrentIsr()
  - -> L3 CurrentAlgo_FastLoopStep()
  - -> L3 PwmAlgo_UpdateRuntimeDuty()
- 协议链：
  - L1 TaskTick_Service()
  - -> L2 ProtocolService_ProcessFrame()
  - -> L2 ParamService_CommitAndStage()
  - -> L2 MotorControlService_ApplyPendingConfig()
  - -> L3 AlgoConfig_Apply()
- 监控链：
  - L1 TaskTick_Monitor()
  - -> L2 TelemetryService_Run()
  - -> L3 TelemetryPack_BuildFrame()
  - -> L4 输出

## 4.2 每层建议保留职责

- L1（仅编排）：
  - 任务调度触发。
  - 中断入口转发。
  - 系统状态机高层切换。
  - 禁止直接 include L3 与 L4。
- L2（功能组织与策略装配）：
  - 电机控制流程组织（何时跑外环、何时跑快环、何时停机）。
  - 协议处理和参数事务。
  - 运行时功能开关裁剪装配（例如启用/禁用电流环链）。
  - 对 L1 暴露少量门面 API。
- L3（可裁剪算法集合）：
  - 速度环、角度环、电流环、采样算法、SVPWM 计算、滤波与补偿。
  - 算法开关由 compile-time 宏集中管理。
  - 对 L2 暴露细粒度算法接口。
- L4（基础能力）：
  - L41：纯数学能力。
  - L42：平台 API（驱动/外设输出输入）。

## 4.3 文件组织建议（含 v1.3.5 编号命名建议）

- L1_Orchestration
  - app_tasks.c：只保留 Task/ISR 入口和调度。
  - app_state_machine.c：系统态转换。
- L2_Service
  - motor_control_service.c：控制链组织门面。
  - protocol_service.c：解析与执行流程门面。
  - param_service.c：参数池与提交策略。
  - telemetry_service.c：监控上报编排。
  - fault_service.c：故障判定与恢复策略。
- L3_Algorithm
  - sensor_algo.c：采样/滤波。
  - motion_outer_algo.c：速度环/角度环。
  - current_inner_algo.c：电流环与软切换。
  - pwm_algo.c：SVPWM 更新。
  - algo_config_apply.c：配置参数应用入口。
  - foc_control_c11_entry.c：控制入口编排与统一对外门面。
  - foc_control_c12_init.c：初始化与标定链。
  - foc_control_c21_cfg_state.c：运行时配置与状态维护。
  - foc_control_c22_runtime_dispatch.c：运行态分发桥接（层级收口，承接 C11/C12 到主算法链）。
  - foc_control_c31_outer_loop.c：外环与慢速链核心。
  - foc_control_c32_current_loop.c：快环与电流环核心。
  - foc_control_c33_softswitch.c：电流软切换并行分支。
  - foc_control_c34_compensation.c：补偿并行分支。
  - foc_control_c41_actuation.c：执行后处理（电角应用与占空比下发）。
- L4
  - 保持 L41_Math、L42_PAL，不承载业务编排逻辑。

## 4.4 接口收敛建议

- L1 仅调用以下 L2 门面（示例）：
  - MotorControlService_RunControlTask()
  - MotorControlService_RunFastCurrentIsr()
  - ProtocolService_PollAndExecute(max_frames)
  - TelemetryService_RunMonitorTick(exec_cycles)
- L2 仅调用以下 L3 细粒度能力（示例）：
  - SensorAlgo_ReadAll()
  - SensorAlgo_ReadCurrentOnly()
  - MotionAlgo_OuterLoopStep(...)
  - CurrentAlgo_Step(...)
  - PwmAlgo_InterpolationIsr()
  - AlgoConfig_Apply(...)

## 5. 裁剪方案与层级对应

- L3 算法裁剪：
  - 例：FOC_SENSOR_ANGLE_LPF_ENABLE、FOC_CURRENT_LOOP_PID_ENABLE、补偿与软切换相关宏。
  - 目标：宏关闭时，算法实现、状态变量、导出接口同步裁剪。
- L2 功能裁剪：
  - 例：是否启用电流环快链、是否启用示波上报、是否接入某协议功能组。
  - 目标：关闭后不再触发对应 L3 链路入口。
- 协议裁剪：
  - 必须覆盖完整链：命令子项 -> 参数字段 -> getter/setter -> 配置应用调用点。

## 6. 最小风险重构切分（先后顺序）

- 第 1 步：先新增 L2 门面空壳与调用转发（不改变算法行为）。
- 第 2 步：L1 改为只调 L2 门面，移除 L1 对 L3/L4 直接 include。
- 第 3 步：把参数应用逻辑从 L1 下沉到 L2 的 ApplyPendingConfig。
- 第 4 步：在 L3 内部做算法函数拆分与裁剪点对齐。
- 第 5 步：最后清理 L2/L3 层内强耦合点，固化编号单向依赖链并按分层约束执行去内联策略。

## 7. M5 细化评估结论（执行前基线）

### 7.1 当前问题（面向 M5）

- 现有 `motor_control_service.c` 直接 include 4 个 L3 头：`foc_control.h`、`foc_control_init.h`、`sensor.h`、`svpwm.h`，接口面仍偏大。
- L2 当前是“薄转发 + 部分编排”混合形态：既有纯转调接口，也有带参数装配逻辑的接口（例如 `ApplyPendingConfig`）。
- 控制链存在“入口粒度不统一”风险：
  - 外环走 `FOC_ControlOuterLoopStep`（总入口）。
  - 快环走 `FOC_CurrentControlStep`（局部入口）。
  - 安全停机走 `FOC_OpenLoopStep`（旁路入口）。

### 7.2 M5 目标解释（“稳定细粒度算法接口”）

- 这里的“细粒度”不是要求 L2 直接调用大量底层数学函数，而是要求按职责分组，形成稳定且可约束的最小算法接口集合。
- 采用“分组稳定接口 + 单职责入口”策略：
  - 运动控制组：外环与快环执行。
  - 传感采样组：全量采样、快采样、采样偏移设置。
  - PWM 执行组：PWM 初始化与插值 ISR。
  - 配置应用组：PID 与控制运行时参数提交。
  - 电机初始化组：电机初始化与校准入口。

### 7.3 需要补齐的 L3 头文件接口（建议新增）

为避免 L2 继续直接 include 过宽的 `foc_control.h`，建议在 L3 新增 3 个聚合头（仅声明，不迁移实现）：

1. `foc/include/L3_Algorithm/motion_control_iface.h`
  - 对外导出：
    - `FOC_ControlOuterLoopStep`
    - `FOC_CurrentControlStep`
    - `FOC_ControlRequiresCurrentSample`
    - `FOC_OpenLoopStep`
2. `foc/include/L3_Algorithm/control_config_iface.h`
  - 对外导出：
    - `FOC_ControlConfigResetDefault`
    - `FOC_PIDInit`
    - `FOC_ControlSet*` 运行时配置接口
    - `FOC_ControlResetCurrentSoftSwitchState`
3. `foc/include/L3_Algorithm/motor_init_iface.h`
  - 对外导出：
    - `FOC_MotorInit`

说明：`sensor.h` 与 `svpwm.h` 已具备较清晰职责分界，可继续沿用，不强制拆分。

### 7.4 重复/旁路调用清单与改法

M5 需要显式治理以下“重复或旁路”风险点：

1. 风险：同一控制周期同时存在多个控制入口（总入口 + 旁路入口）。
  - 现状：`RunOuterLoop`、`RunCurrentLoop`、`RunOpenLoop` 并存。
  - 改法：在 L2 增加统一编排函数（建议：`MotorControlService_RunControlTask`），把模式分发集中在一个函数里，确保一次周期只走一条入口链。

2. 风险：L1 触发多种门面组合，可能绕开 L2 既定顺序。
  - 现状：L1 仍分别调用多个 `MotorControlService_*`。
  - 改法：M5 只在 L2 内组织，不再新增 L1 调用分支；对外保留少量稳定 API，逐步将 L1 收敛到单入口调用。

3. 风险：配置应用与控制执行边界不稳。
  - 现状：`ApplyPendingConfig` 已下沉，但其接口边界未文档化。
  - 改法：固定约束：仅 `ApplyPendingConfig` 允许调用 `FOC_ControlSet*`；其余执行路径禁止直接改运行时配置。

### 7.5 文件移动/更名结论

- M5 阶段不要求移动 `.c` 文件，不建议做目录迁移。
- M5 阶段不要求重命名已有实现文件（`foc_control.c`、`sensor.c`、`svpwm.c` 保持不变）。
- 仅建议新增 L3 聚合头文件（见 7.3），并将 L2 include 从“大而全头文件”替换为“职责聚合头文件”。
- 若后续进入更深层重构（例如 M6 之后），再评估文件更名/拆分，不在 M5 执行窗口内处理。

### 7.6 M5 可执行改造清单（落地顺序）

1. 新增 3 个 L3 聚合头（仅声明）。
2. 调整 `motor_control_service.c` include，替换为聚合头。
3. 在 `motor_control_service.c` 内新增统一编排入口（`RunControlTask`），把外环/快环/安全停机的入口选择集中。
4. 保留现有接口做兼容壳（短期），但将调用统一转入编排入口，避免逻辑分叉。
5. 文本检索确认 L2 对 L3 的调用面仅来自约定接口组。

### 7.7 M5 验收口径（细化）

- 调用面验收：`motor_control_service.c` 中 L3 调用仅来自约定接口组；无未登记旁路调用。
- 行为验收：控制主链一次周期只走一条算法入口链，不出现“双入口并行”。
- 构建验收：`phase-d-verify-build` 或等价 `phase-d-rebuild` 通过，0 error，不新增 warning。

## 8. M6 前新增执行步骤（协议链重组准备）

### 8.1 重组目标冻结

- 协议链采用“L2 运行态适配 + L3 纯处理内核”两段式组织。
- L3 仅承载纯处理：帧提取、语法解析、命令归一化、文本格式构建。
- L2 保留运行态实现：多源收发、状态回执、参数池、故障统计、系统态流转。

### 8.2 可下沉与保留清单（执行基线）

- 可下沉到 L3（纯处理）：
  - `ProtocolParser_ExtractFrame` -> `ProtocolCore_ExtractFrame`（帧边界提取）。
  - `ProtocolParser_ParseFrame` -> `ProtocolCore_ParseFrame`（帧语法解析）。
  - `ProtocolParser_ParseSignedFloat` -> `ProtocolCore_ParseSignedFloat`（参数数字解析）。
  - `ProtocolParser_IsDriverIdFormatValid` -> `ProtocolCore_IsDriverIdFormatValid`（地址格式校验）。
  - `ProtocolParser_IsDriverAddressedToLocal` -> `ProtocolCore_IsDriverAddressedToLocal`（地址归属校验）。
  - `CommandManager_ParseStateValue` -> `ProtocolCore_ParseStateValue`（状态值归一化）。
  - `CommandManager_GetParamName/GetStateName/IsIntegerParam` -> `ProtocolText_*`（文本映射与格式判定）。
- 必须保留在 L2（运行态）：
  - `FOC_Platform_CommSource*_IsFrameReady/ReadFrame` 等平台收发。
  - `FOC_Platform_WriteStatusByte/WriteDebugText` 等回执输出。
  - 参数池 `g_params`、状态池 `g_states`、运行态 `g_runtime_state`。

### 8.3 M6 前闭环任务序列（与 NEXT_MISSION 对齐）

1. M5.1：文档与接口契约冻结闭环。
2. M5.2：协议纯处理内核下沉到 L3（第一阶段，行为不变）。
  - 新增 `protocol_core_types.h`、`protocol_core_parser.h/.c`、`protocol_core_normalize.h/.c`、`protocol_text_codec.h/.c`。
  - L2 对应文件改为调用 `ProtocolCore_*` / `ProtocolText_*`。
3. M5.3：协议链依赖收口与调用面定型。
  - `protocol_parser.h` 与 `command_manager_dispatch.h` 统一依赖 `protocol_core_types.h`。
  - `protocol_service.c` 仅保留步进编排；`command_manager_diag.c` 仅保留平台输出桥接。

### 8.4 执行约束

- 每一步都必须“改动 + 构建 + 文档回写”同次闭环。
- 严禁把运行态平台读写逻辑整体搬入 L3。
- L3 控制链重构必须满足“编号命名可读 + 单向依赖可检索”，禁止先拆分后补依赖方向。
- 去内联约束分层处理：L2 允许少量受控 inline（模块内私有且可检索）；L3 除 `L41_Math` 外不新增业务 inline 共享头。
- 本文档在 M11 完成前为有效约束源，不得删除或替换为口头约定。

### 8.5 M7 前新增步骤（L3 控制算法编号单向链准备）

1. 目标：把 `foc_control.c` 从“单文件大实现”重构为“同一算法整体下的分层单向链实现”。
2. 命名规则：按功能链编号命名 `foc_control_cxy_*`，其中 `x`=调用链层级、`y`=并列层级编号（与 `L` 层编号语义一致）；文件名直接反映依赖层与职责（不用 `L` 前缀避免与分层编号混淆）。
3. 依赖规则：仅允许 `C11/C12 -> C21/C22 -> C31/C32 -> C41` 主方向；`C33/C34` 作为并行分支仅供 `C32/C31` 调用；禁止跨级 include（例如 `C11/C12 -> C31/C41`）。
4. 对外接口：`motion_control_iface.h`、`control_config_iface.h` 保持稳定，不把分层细节泄漏给 L2。
5. 边界约束：`sensor.c`、`svpwm.c` 本轮不并发拆分；仅在控制链文件内完成重构。
6. 验收检查：
  - include 检索：不存在 `c32/c41` 反向 include `c11/c21`。
  - 宏闭环：`FOC_CURRENT_LOOP_PID_ENABLE` 在新链路中满足“宏开关 -> 状态变量 -> 接口 -> 调用点”闭环。
  - 构建验证：`phase-d-verify-build`（缺 `eide` 时用 `phase-d-rebuild`）通过且无新增 warning。

## 9. 2026-04-17 重开收口约束与任务目标（R 版）

### 9.1 基线评估（重开时源码快照，历史基线）

1. L3 存在“新链 + 旧壳”并存：控制链文件已拆分，但 `foc_control.c` 仍在工程清单中。
2. L3 链内仍通过公共总头/公共 internal 头耦合：`foc_control.h`、`foc_control_internal.h` 被多级链路复用，违背“层级最小依赖”。
3. L3 非控制链算法仍分散：协议纯处理位于 `protocol_core_parser.c`、`protocol_core_normalize.c`、`protocol_text_codec.c` 三文件并列，未做单文件聚合。
4. L2 存在兼容壳外露与并列散点：`RunOpenLoop/RunOuterLoop/RunCurrentLoop` 与统一入口并存；同域读写与诊断/输出职责仍有并列文件。

### 9.2 强化约束（覆盖旧口径）

1. 禁止兼容共存：调用链落地后，旧壳文件必须同次删除，不允许“先保留、后清理”。
2. 禁止公共 internal 聚合头穿透：L3 链内只能依赖“本级头 + 下一级头 + 必要共享类型头”，不得通过 `foc_control_internal.h` 作为跨级入口。
3. 严格层层单向，允许同层并行：同层可有多个算法分支（如外环/电流环并行），但分支必须汇入下一层统一执行出口。
4. 非链同域算法必须合并：L3 协议纯处理、L2 同域并列处理模块不允许长期分散。
5. 删除动作必须联动工程清单：删源文件时必须同步更新 `builder.params`、`.eide/eide.yml`，否则不通过验收。

### 9.3 重开任务目标（按执行顺序）

1. R1-L3-Control（全链 Cxy 化，不限于 `foc_control.c`）：
  - 文件范围：`foc_control_c11_*`、`foc_control_c12_*`、`foc_control_c21_*`、`foc_control_c22_*`、`foc_control_c31_*`、`foc_control_c32_*`、`foc_control_c33_*`、`foc_control_c34_*`、`foc_control_c41_*`。
  - 目标：所有 `foc_control` 相关控制算法函数都归入 `Cxx` 调用链；旧文件 `foc_control.c`、`foc_control.h` 必删。
  - 上行约束：对 L2 只保留一个上行实现文件（`C11`）。
2. R2-L3-Compaction+Protocol（合并任务）：
  - C1 收口：`C22` 职责已并入 `C11`，独立分发壳已删除。
  - 初始化拆分：已落地为“算法组织级（L3-C1，与 C11 同层）”和“控制算法级（L3 初始化原语层）”。
  - R2 范围限定：仅完成 L3 内部层级重排与收口，不进行 L1/L2 大层级提升。
  - C21/C12 评估结论：不做整文件合并；`C21` 保持运行配置/状态库，`C12` 保持初始化算法原语，仅保留最小接口连接。
  - 协议纯处理合并（并入 R2 同次执行）：
    - 合并来源：`protocol_core_parser.c`、`protocol_core_normalize.c`、`protocol_text_codec.c`（已删除）。
    - 合并结果：`protocol_core.c` 单实现入口。
  - R2 优先执行锁（本轮）：执行与验收只以本节 R2 条目为准；若与其他文档冲突，本节口径优先，R3 及后续任务仅作规划不进入实施面。
  - R2 执行结果（2026-04-17）：`phase-d-rebuild` 全量重建通过（0 error），未发现 R2 删除项在工程清单残留，warning 无新增。
3. R3-L3-Restructure（新增，L3 深化重排与层级合并/整体命名提升）：
  - 当前 `C33`（软切换）并入电流环主链，避免并行分支过细化。
  - 当前 `C34` 收口为齿槽补偿算法，并承接初始化侧补偿准备逻辑。
  - 从 `C12` 拆分“电机参数学习算法”为新 `C33`，`C12` 保留初始化组织与标定原语。
  - `C11` 去除对当前 `C41` 的直接依赖，执行输出链改为经电流环组织路径收口。
  - 第一步：旧 `C2` 并入旧 `C3`，形成 `C1/C2/C3/C4 -> C1/C3/C4` 的职责收口。
  - 第二步：对旧 `C3/C4` 做整体命名提升，落为新 `C2/C3`（即最终层级为 `C1/C2/C3`）。
4. R3.5-L2-Chain-Style（新增，宏裁剪前前置）：
  - 在现有文件名基础上引入“类 Cxy”的链式口径：`entry -> dispatch -> store/query -> diag`，不强制同轮改文件名。
  - `motor_control_service` 仅保留统一任务入口，兼容壳 `RunOpenLoop/RunOuterLoop/RunCurrentLoop` 退场。
  - `command_manager_store.c` 与 `command_manager_query.c` 合并为同域单文件，避免并列碎片化。
5. R3.6-L3-Residual-Cleanup（新增，宏裁剪前前置）：
  - 清理 L3 结构化残留：冗余 wrapper/helper、过细头文件边界、孤儿 `.c/.h`。
  - 保持 `C11 -> C22` 桥接边界，不回退为 `C11` 直连执行层。
  - 该步只做结构清理，不引入算法参数语义变化。
6. R4-L2-Chain（原 R3，链式与并列拆分治理）：
  - 链式固化：`protocol_service.c -> protocol_parser.c -> command_manager.c`。
  - 命令链固化：`command_manager.c -> command_manager_dispatch.c -> store/query -> command_manager_diag.c`。
  - 非链同域聚合：`command_manager_store.c` 与 `command_manager_query.c` 合并为单文件。
  - 控制门面收口：`motor_control_service.c` 仅保留统一调度入口，删除兼容壳导出。
7. R5-List+Build（原 R4，清单与构建闭环）：
  - 清单同步：`builder.params`、`.eide/eide.yml` 与源文件删除/合并动作同次更新。
  - 验证：检索 + `phase-d-rebuild` + 文档回写同次完成。

### 9.3.1 L3 控制链函数映射（强制）

1. C11（R2 后上行唯一入口文件）：`FOC_ControlOuterLoopStep`、`FOC_OpenLoopStep`，并吸收原 `C22` 的模式分发、入口复位与执行桥接职责。
2. C11（R3 目标）：移除对当前 `C41` 的直接 include/直接调用，执行输出经电流环组织路径收口。
3. C12（初始化算法原语层）：保留 `FOC_MotorInit`、`FOC_CalibrateElectricalAngleAndDirection` 与初始化组织逻辑；剥离“电机参数学习算法”。
4. 新 C33（R3 目标）：承接从 `C12` 拆分的电机参数学习算法。
5. 当前 C21（运行配置/状态层）：`FOC_ControlConfigResetDefault`、全部 `FOC_ControlSet*`、`FOC_PIDInit`、状态访问函数；R3 中并入旧 C3 族，不再保留独立 C2 层。
6. 当前 C31（外环）：`FOC_SpeedOuterLoopStep`、`FOC_SpeedAngleOuterLoopStep`、角度/速度累积 helper；R3 后提升为新 `C21`。
7. 当前 C32（电流环）：`FOC_CurrentControlStep`、`FOC_ControlRequiresCurrentSample`、闭环融合逻辑；R3 吸收当前 `C33` 与执行输出组织后提升为新 `C22`。
8. 当前 C34（补偿分支）：`FOC_ControlCoggingLookupIq`；R3 收口为齿槽补偿算法并提升为新 `C24`。
9. 当前 C41（执行层）在 R3 后提升为新 `C31`，且不再作为 C11 直连依赖点。
10. C21/C12 评估结论：保持分离，不做整文件合并。
11. R3 层级合并与提升映射冻结：旧 `C2` 并入旧 `C3`；旧 `C3->新 C2`、旧 `C4->新 C3`；示例映射 `C31->C21`、`C32->C22`、`C33->C23`、`C34->C24`、`C41->C31`。

### 9.3.2 L2 函数级细化（强制）

1. `motor_control_service.c`：
  - 保留：`MotorControlService_RunControlTask`、`MotorControlService_ApplyPendingConfig`。
  - 删除对外兼容壳：`RunOpenLoop`、`RunOuterLoop`、`RunCurrentLoop`（仅转发时）。
2. `command_manager_dispatch.c`：`CommandManager_DispatchExecute` 只做分发，不承载参数存储读写细节。
3. `command_manager_store/query` 合并文件：保持 `WriteParam/ReadParam/WriteState/ReadState/ReportAll*` 函数边界，不改变协议行为。
4. `command_manager_diag.c`：仅保留输出桥接与文本封装，不承载状态机与参数逻辑。

### 9.4 验收口径（R 版）

1. 检索无 `foc_control.c`、`foc_control.h` 在控制链中的残留依赖与工程清单条目。
2. `Cxx` 源文件检索无 `foc_control_internal.h` 作为跨级公共入口的用法。
3. L3 协议纯处理仅保留单文件实现入口。
4. L2 `store/query` 完成同域合并，且 `DispatchExecute` 不回流写入细节。
5. L3 对 L2 的运行上行入口仅保留单文件（C11）。
6. `foc_control_c22_runtime_dispatch.*` 在 R2 完成后不再保留独立构建条目。
7. 初始化职责满足“组织级与算法级均在 L3 完成重排”约束：组织级与 `C11` 同层，算法级保持初始化原语层，且无跨层回流。
8. 构建通过且无新增 warning。

