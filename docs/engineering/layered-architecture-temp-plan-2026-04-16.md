# 分层相邻单向依赖临时方案（评估版）

日期：2026-04-16  
状态：临时方案（执行期参考文档；保留至 M11 完成并验收后再归档）

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
  - foc_control_c01_entry.c：控制入口编排与统一对外门面。
  - foc_control_c02_config_state.c：运行时配置与状态维护。
  - foc_control_c03_outer_loop.c：外环与慢速链核心。
  - foc_control_c04_current_loop.c：快环与电流环核心。
  - foc_control_c05_actuation.c：执行后处理（软切换/补偿/电角应用）。
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
2. 命名规则：按功能链编号命名 `foc_control_c01~c05_*`，文件名直接反映依赖层与职责（不用 `L` 前缀避免与分层编号混淆）。
3. 依赖规则：仅允许 `C01 -> C02 -> C03 -> C04 -> C05` 方向，禁止低层 include 高层。
4. 对外接口：`motion_control_iface.h`、`control_config_iface.h` 保持稳定，不把分层细节泄漏给 L2。
5. 边界约束：`sensor.c`、`svpwm.c` 本轮不并发拆分；仅在控制链文件内完成重构。
6. 验收检查：
  - include 检索：不存在 `c04/c05` 反向 include `c01/c02`。
  - 宏闭环：`FOC_CURRENT_LOOP_PID_ENABLE` 在新链路中满足“宏开关 -> 状态变量 -> 接口 -> 调用点”闭环。
  - 构建验证：`phase-d-verify-build`（缺 `eide` 时用 `phase-d-rebuild`）通过且无新增 warning。

