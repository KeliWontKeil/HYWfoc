# Source/Control 架构重构执行计划

> 状态：执行准备
> 架构事实源：`docs/TEMP_IMPLEMENTATION_PLAN.md`
> 约束：在重构完成前不修改架构事实源。本文只记录执行顺序、检查点和代码落地策略。

## 一、执行原则

1. 严格按 Source + Control Policy + PWM ISR 输出流程架构重构，不做旧路径补丁式兼容。
2. 每个阶段结束必须保持可编译，除非该阶段在开始前明确标注为不可编译迁移窗口。
3. 先建立类型和边界，再迁移行为；避免在旧 `STARTUP` / estimator selector / transition 语义上继续叠补丁。
4. 不回退工作区中已有的非本轮改动。若同一文件已有改动，先读懂再在其上继续。
5. 新增 `.c` 文件必须同步 EIDE `builder.params` / 工程源列表。
6. `TEMP_IMPLEMENTATION_PLAN.md` 在实现完成前保持不变，作为架构事实源。

## 二、阶段 0：基线确认

目标：确认当前工程状态、构建入口和已有脏改动，建立实现前快照。

动作：

- 记录 `git status --short`，区分本轮计划文件与既有改动。
- 确认构建命令：优先 `tools\build_gd32f303.ps1`。
- 确认工程源列表位置：`examples/GD32F303_FOCExplore/software/build/GD32F30X_CL/builder.params` 和 Keil/EIDE 工程文件。
- 读取当前 `foc_ctrl_types.h`、`foc_ctrl_executor.c`、`foc_app.c`、`foc_init.c`、`foc_config.h`、`foc_compile_limits.h`。

验收：

- 清楚当前 dirty 文件范围。
- 能说明新增模块需要加入哪些构建入口。

## 三、阶段 1：类型与状态结构落地

目标：先建立目标架构所需的数据结构，不迁移行为。

主要改动：

- 在 `foc_ctrl_types.h` 中新增或重命名：
  - `foc_source_type_t`
  - `foc_source_state_t`
  - `foc_active_source_state_t`
  - `foc_control_region_t`
  - `foc_encoder_services_state_t`
  - `foc_phase_output_type_t`
  - `foc_phase_output_state_t`
  - applied output snapshot 字段
- 保留必要的 `estim_*` 私有状态命名，但把对外语义转成 source snapshot。
- 删除目标架构中不再需要的 `FOC_CONTROL_PHASE_STARTUP` 枚举入口，或先用编译错误驱动后续迁移。

边界：

- 不在本阶段改 Control ISR/PWM ISR 流程。
- 不让协议层直接访问或写 source manager 私有状态。

验收：

- 类型定义自洽。
- 旧代码若因 `STARTUP` 删除产生编译错误，错误应集中指向需要迁移的旧路径。

## 四、阶段 2：Source 模块与 Source Manager

目标：建立 NORMAL 下唯一 source 发布链。

主要改动：

- 新增 `foc_ctrl_source_mgr.h/.c`，替代 `foc_ctrl_transition.c` 的运行时选源职责。
- Source Manager 职责：
  - 初始化 active/standby source 和 `control_region`
  - 在 PWM ISR NORMAL 流程运行各 source step
  - 根据阈值、收敛状态和滞回切换 active source
  - 发布 `active_source_state`
  - 写 `electrical_phase_angle`
  - 维护 `encoder_services`
- Encoder/SMO/HFI source 输出统一 source snapshot。
- SMO/HFI 使用上一周期 applied output snapshot 作为电压输入。

边界：

- Source 不写 `iq_target`。
- Control ISR 不调用 Source Manager 切换/发布函数。
- `encoder_services` 只由 Source Manager 写。

验收：

- Encoder 全速域可由 Source Manager 发布 active source。
- OpenLoop/SMO/HFI 编译裁剪组合不出现声明/定义/调用不一致。

## 五、阶段 3：OpenLoop 拆分

目标：移除旧 STARTUP phase，把强拖拆为 OpenLoop Angle Source + OpenLoop Low-Speed Policy。

主要改动：

- 从 `foc_ctrl_startup_openloop.c` 拆出或重命名：
  - OpenLoop Angle Source：只更新虚拟角度/速度 snapshot。
  - OpenLoop Low-Speed Policy：只生成 `iq_target`。
- 移除 `FOC_Startup_*` 在 L1 `control_phase` 中的路由。
- 初始化时：
  - `control_phase = NORMAL`
  - `active_source = OPENLOOP`
  - `control_region = LOW`

边界：

- OpenLoop Angle Source 不写 `electrical_phase_angle`。
- OpenLoop Low-Speed Policy 不写 source snapshot。

验收：

- `FOC_CONTROL_PHASE_STARTUP` 不再存在。
- OpenLoop → SMO 场景可以明确：角度由 OpenLoop source 发布，`iq_target` 由 OpenLoop policy 生成，切源由 Source Manager 完成。

## 六、阶段 4：PWM ISR 标准流程重构

目标：让 PWM ISR 成为 NORMAL source 发布和输出流程仲裁点。

主要改动：

- 重构 `FOC_ControlExecutor_RunISR()`：
  - 公共安全输出服务
  - 按 `control_phase` 路由 PWM 输出流程
  - NORMAL：采样 → source update → Source Manager publish → current loop → SVPWM → record applied output
  - COGGING_CALIB / REINIT：调用特殊输出流程，跳过标准 Source Manager 和普通电流环
- 电流环只消费已发布 `electrical_phase_angle`。
- SVPWM 应用后记录 applied output snapshot。

边界：

- PWM ISR 不生成 `iq_target`。
- 特殊模式不进入 NORMAL Source Manager 发布链。

验收：

- NORMAL 下同一 PWM 周期只有一个 active source view。
- SMO/HFI observer 不使用同周期刚生成的 `ud/uq`。

## 七、阶段 5：Control ISR 与 Control Policy Selector

目标：Control ISR 只消费已发布 source view，生成 NORMAL 控制量或推进特殊状态机。

主要改动：

- 重构 `FOC_App_ControlTrigger()` NORMAL 路径：
  - 慢速 sensor/VBUS/current snapshot 同步
  - 读取 `active_source_state/control_region/encoder_services`
  - 调用 Control Policy Selector
- Control Policy Selector：
  - OpenLoop active + LOW → OpenLoop Low-Speed Policy
  - Encoder/HFI/SMO active → 普通速度/位置外环
  - Encoder active 且服务允许 → 齿槽补偿叠加
- 移除 `FOC_Bridge_CopyInput` 对新架构的真实数据职责。

边界：

- Control ISR 不发布 `electrical_phase_angle`。
- Control ISR 不切 source。

验收：

- `iq_target` 的写入路径集中到 Control Policy。
- Source 与 Control Policy 可分别遥测。

## 八、阶段 6：特殊模式状态机迁移

目标：把 COGGING_CALIB / REINIT 纳入 PWM ISR 特殊输出流程。

主要改动：

- 为 cogging/reinit 状态机增加或复用 `foc_phase_output_state_t`。
- Control ISR 推进状态机并更新特殊输出状态。
- PWM ISR 特殊输出流程消费特殊输出状态并应用输出。
- 移除状态机中直接写 SVPWM / direct duty 的运行时输出路径，必要时保留为 init-time calibration 专用函数。

边界：

- 特殊输出状态不是协议 command。
- 运行时特殊模式仍直接读物理 `sensor.mech_angle_rad`。
- 入口由 `encoder_services` 门控；init-time calibration 是例外。

验收：

- COGGING_CALIB / REINIT 不覆盖 NORMAL active source state。
- 特殊模式结束回到 NORMAL 后，由下一次 NORMAL PWM ISR 恢复 source publish。

## 九、阶段 7：初始化与 LS 约束

目标：按合法 source 组合初始化，不再把编译期约束散落到运行时。

主要改动：

- `foc_config.h` / `foc_compile_limits.h`：
  - 派生 source/policy enable 宏
  - 用 `#error` 阻断非法 source pair
  - 移除旧 `STARTUP_OPENLOOP` 语义或改成 OpenLoop source/policy enable
- `FOC_App_Init` / `FOC_App_Start`：
  - 保证 init 阶段 control tick / PWM update / runtime interrupt 全部关闭
  - `FOC_App_Start` 才启动运行时中断
- 初始化校验：
  - 无 Encoder 无感组合不因 `encoder_valid == 0` 失败
  - 必须校验 `pole_pairs/direction/current sensing`
  - `current_loop_ready` 只在校验和初始 source 可用时置位

边界：

- init-time calibration 是初始化专用 direct-output 特殊过程，不属于运行时 `control_phase` 特殊流程。
- 初始化可以读取物理 Encoder 做标定，但不改变初始 active source。

验收：

- OpenLoop → SMO 初始 `control_phase = NORMAL`。
- 无 Encoder 的无感组合可通过初始化校验，但缺少必要 motor 参数时被阻断。

## 十、阶段 8：协议、遥测与可见性

目标：旧协议体系接入新状态，不直接参与控制链。

主要改动：

- `P/C/S/Y` 命令保持语义：
  - `P` 写目标
  - `C` 写配置
  - `S` 写使能
  - `Y` 写请求或 phase
- 新增或复用只读遥测：
  - active source id/state
  - control_region
  - encoder_services
  - source switch counter
  - special phase output state
  - applied output snapshot
- 协议不得写 active source，不得选择 source pair，不得写 PWM 输出。

验收：

- 原有目标/PID/使能命令仍可用。
- 新 source/control 状态可观测。

## 十一、阶段 9：清理旧结构与命名

目标：删除旧架构残留，避免并行事实。

主要改动：

- 删除或重命名旧 startup 路由相关文件/API。
- 删除运行时 Estimator Selector 的 active source 选择职责。
- 删除 `ctrl_input` / Bridge 在新架构中的真实数据职责。
- 清理 `STARTUP_OPENLOOP`、`FOC_CONTROL_PHASE_STARTUP`、`OpenLoop estimator` 等术语。
- 更新工程源列表，移除废弃 `.c`，加入新增 `.c`。

验收：

- 全文搜索不再有架构冲突术语。
- 新模块名与职责一致。

## 十二、阶段 10：构建与回归

目标：完成编译和最低行为回归。

验证命令：

```powershell
.\tools\build_gd32f303.ps1
```

验收：

- 0 error。
- 不新增 warning。
- L2 不包含 `L1_Orchestration/` 头文件。
- 宏裁剪链路声明/定义/调用一致。
- Encoder 全速域路径不回归。
- OpenLoop → SMO 路径可编译。
- COGGING_CALIB / REINIT 编译开启时无未定义符号。

## 十三、风险与处理顺序

高风险点：

- `foc_ctrl_types.h` 结构变化会触发大量编译错误。
- `FOC_CONTROL_PHASE_STARTUP` 删除会暴露旧启动路径依赖。
- 特殊模式输出迁移容易和 SVPWM direct/runtime 输出冲突。
- 工程源列表不同步会导致链接错误。

处理策略：

- 先让类型和空实现编译，再逐步接入行为。
- 每移除一个旧职责，立即全文搜索旧调用点。
- 每新增 `.c`，同阶段更新构建入口。
- 特殊模式迁移前先建立 `foc_phase_output_state_t`，再改状态机输出。

## 十四、执行检查清单

- [ ] 阶段 0：基线确认
- [ ] 阶段 1：类型与状态结构落地
- [ ] 阶段 2：Source 模块与 Source Manager
- [ ] 阶段 3：OpenLoop 拆分
- [ ] 阶段 4：PWM ISR 标准流程重构
- [ ] 阶段 5：Control ISR 与 Control Policy Selector
- [ ] 阶段 6：特殊模式状态机迁移
- [ ] 阶段 7：初始化与 LS 约束
- [ ] 阶段 8：协议、遥测与可见性
- [ ] 阶段 9：清理旧结构与命名
- [ ] 阶段 10：构建与回归
