# 下一阶段任务（NEXT_MISSION）

## 0. 目标与边界

- 当前稳定基线：`v1.3.3`
- 下一目标版本：`v1.3.4`
- 本轮主线：P0/P1/P2/P3 已闭环完成，进入下一轮功能演进前的稳定维护阶段。
- 本轮定位：重构式调整，不做兼容式堆叠；优先长期可读性与可维护性。
- 暂缓项：齿槽补偿（Cogging）参数调优与效果优化，仅保留框架。

## 1. 强制约束（本轮执行基线）

### 1.1 总体原则

1. 单电机有感 FOC 是唯一业务边界。
2. 所有改动必须可编译、可回滚、可文档追溯。
3. 结构治理优先于功能扩展。
4. 不得新增告警（no newly introduced warnings）。

### 1.2 可读性与可维护性约束（必须可验收）

1. 主函数只允许调用 `L1` 入口和 `L5` 硬件初始化，不允许横向绕层。
2. 禁止跨层依赖和反向依赖，保持严格单向依赖。
3. 算法层（`L3` 及以下）以传参/结构体交互为主；结构体职责必须单一，禁止“一个结构体服务多个不相关算法链”。
4. 函数和宏必须语义命名，禁止缩写堆叠和硬件细节泄漏到上层语义接口。
5. 裁剪分为两类：算法裁剪仅作用 `L3`，功能裁剪仅作用 `L2`；同类裁剪开关必须集中定义，禁止分散。
6. 禁止无意义变量引入；L2 允许极少量受控 inline（仅模块内私有且可检索），L3 业务逻辑默认去内联并落到 `.c` 分层模块（`L41_Math` 数学 LUT 例外）。
7. 限幅/归一化只保留“必要且有依据”的节点，禁止在同一计算链重复叠加。
8. 非必要不新增文档；优先更新现有事实源文档。
9. 这是重构，不做兼容层叠；禁止为“兼容旧路径”保留多层跳转结构。
10. 回归验证采用“代码逻辑审查 + 编译验证”双轨；每一轮改动后都要复核关键文件。
11. 当上下文被压缩或会话切换后，必须重新阅读相关文档与代码，不依赖历史记忆结论。

### 1.3 安全性边界说明

1. 算法主链不添加泛化空指针防御分支，避免污染实时路径。
2. 接口边界（平台桥接层、对外 API 入口）必须保留必要输入有效性约束。
3. 任何“去防御化”改动都必须明确其调用前置条件与责任边界。

## 2. 分阶段任务

### P0 文档与流程基线固化（已完成）

目标：把结构事实、流程规则与验收口径统一为“单一事实源 + 可执行检查项”。

状态：已完成（2026-04-15）。

交付物：
1. `docs/architecture.md` 作为唯一结构说明文档（SSOT）。
2. 删除结构重复文档，不保留兼容式跳转页。
3. `AI_INITIALIZATION.md`、`copilot-instructions.md`、`.github` 工作流文档与当前代码保持一致。
4. 编译/调试经验同步写入 `docs/development.md` 与仓库长期记忆。
5. 库级文档移除硬编码控制频率表述，统一改为“由配置宏决定的控制周期/调度槽”。
6. 在任务文档中显式记录“可读性拒收条件”。

验收：
1. 文档索引无冲突引用、无过期链接。
2. 结构事实只有一个维护入口。
3. 版本口径与当前基线一致。
4. 检索库级文档后，不存在“固定控制频率即事实”的描述。

拒收条件（任一命中即不通过）：
1. 出现重复事实源或多文档并行描述同一结构细节。
2. 文档仍保留兼容跳转叙述而非直接事实。
3. 约束条款无法映射到具体检查动作。

### P1 架构重构准备（已完成）

状态：已完成（2026-04-15）。

交付物：
1. 控制链路与协议链路边界清单（当前状态 + 目标状态）。
2. 配置宏归属清单（`foc_cfg_symbol_defs` / `feature_switches` / `init_values` / `compile_limits`）。
3. 平台 API 最小集清单（保留、候选移除、必须新增）。
4. 关键时序点表（控制周期、PWM 更新中断、采样触发）与影响分析。

验收：
1. 每一项边界调整都能映射到文件级位置。
2. 不改算法也能完成独立评审。

### P2 结构重排实施（已完成）

状态：已完成（2026-04-15）。

目标：一次性完成职责收敛与目录重排，不分批引入兼容层。

实施项：
1. 协议解析与运行状态管理边界收敛。
2. 控制算法内部接口收敛（保持 init/runtime 严格分离）。
3. 平台桥接与 `L5` 驱动依赖可视化整理。
4. 文件细分与目录重组织同步完成。

验收：
1. `unify_builder --rebuild` 通过。
2. 不新增 warning。
3. 文档同次迭代同步。

### P3 回归验证与发布准备（已完成）

状态：已完成（2026-04-15）。

交付物：
1. 编译日志与资源占用对比。
2. 启动标定链路验证记录（电角锁定 -> 机械零位采样 -> 方向/极对数估计）。
3. 运行链路验证记录（运行态 SVPWM 插值链）。
4. `CHANGELOG.md` 发布条目草案。

本轮结果记录：
1. 构建命令：`unify_builder.exe -p build/GD32F30X_CL/builder.params --rebuild`。
2. 构建结果：`0 error, 1 warning(L6914W)`，无新增 warning。
3. 体积结果：`Code=43592, RO=6168, RW=268, ZI=2636`。

## 3. 执行顺序

1. 先 P0，再 P1，后 P2/P3。
2. P2 前必须完成对应 P1 清单确认。
3. 若 P2 触发接口变化，必须回写 P0 文档并复核。

## 4. 已确认决议

1. 层级命名固定为 `LS + L1~L5`，停止旧术语并行映射。
2. 协议链采用“L2 运行态适配 + L3 纯处理内核”重组方向：帧提取/语法解析/命令归一化/文本格式构建等纯处理逻辑可下沉 L3；多源收发、状态回执、参数池与故障统计保留 L2。
3. `debug_stream` 严格保持只读观察通道，采用快照或只读指针接入，禁止写回控制状态。
4. 平台 API 中断生命周期仅保留统一入口，不开放分散开关接口。
5. 实例目录维持现有 `Utilities` 粒度，本阶段不拆分。
6. L3 控制算法采用“一个整体 + 编号分层文件 + 单向依赖链”推进；层内链路命名不使用 `L` 前缀，统一使用 `Cxx` 功能链编号表达依赖层级与职责。

## 5. 暂缓项（明确不做）

1. 齿槽补偿参数整定与效果冲刺。
2. 无感 FOC 路线开发。
3. 多电机场景扩展。

## 6. 参考入口

1. 结构事实源：`docs/architecture.md`
2. 开发流程：`docs/development.md`
3. 规则集合：`docs/engineering/dev-guidelines/rules/cn/`、`docs/engineering/dev-guidelines/rules/en/`

## 7. v1.3.4 实施任务清单（M 编号单轨版）

### 7.1 执行前约束复核（每次任务开始前）

1. 同时复核 `NEXT_MISSION.md`、`docs/development.md`、`docs/engineering/layered-architecture-temp-plan-2026-04-16.md`，不得依赖口头约定。
2. 临时方案文档在 M11 完成并验收前是有效约束源，不得删除、替换或弱化。
3. 严禁把运行态平台读写逻辑整体搬入 L3；L3 仅承载纯算法/纯处理逻辑。
4. 裁剪分层必须保持：结构治理先于裁剪治理；算法裁剪归 L3，功能裁剪归 L2。
5. 层内链路命名不得使用 `L` 前缀，统一使用功能链编号（示例：`C01..C05`）。
6. 去内联约束：L2 允许少量受控 inline（模块内私有、可检索、不可跨模块传播）；L3 业务逻辑默认完全去内联。

### 7.2 任务闭环定义（全部必选）

1. 每个 M 任务必须单次闭环：代码改动、检索验证、构建验证、文档回写、回归结论同次完成。
2. 每个 M 任务必须可独立验收：不允许“先留接口、下一任务补实现”。
3. 每个 M 任务对应一次本地提交，不跨任务打包。
4. 构建验证首选 `phase-d-verify-build`；若环境缺少 `eide` 命令，使用 `phase-d-rebuild` 并记录原因。

### 7.3 已完成里程碑（归档）

1. M1：L2 控制门面建立并替换 L1 直连 L3（已完成，2026-04-16）。
2. M2：L2 协议门面建立并替换 L1 协议细节调用（已完成，2026-04-16）。
3. M3：参数应用链下沉至 L2（已完成，2026-04-16）。
4. M4：L1 去 L3 include 依赖（已完成，2026-04-16）。
5. M5：L2 控制服务调用面定型与协议链基线冻结（已完成，2026-04-16）。

### 7.4 结构阶段（先结构，后裁剪）

#### M6：L2 结构链重排（运行编排 -> 分发 -> 存储查询 -> 诊断）

目标：先把 L2 从“中心化超大文件”重排为可解释单向结构链，不在本任务做裁剪。

文件级细化：
1. `foc/src/L2_Service/command_manager.c`
2. `foc/src/L2_Service/command_manager_dispatch.c`
3. `foc/src/L2_Service/command_manager_diag.c`
4. `foc/include/L2_Service/command_manager.h`
5. `foc/include/L2_Service/command_manager_dispatch.h`
6. `foc/include/L2_Service/command_manager_diag.h`
7. 必要时新增：`foc/src/L2_Service/command_manager_store.c`、`foc/src/L2_Service/command_manager_query.c`（及对应头）

函数级细化：
1. 将 `CommandManager_WriteParam/ReadParam/WriteState/ReadState/ReportAllParams/ReportAllStates` 迁出主编排文件。
2. `CommandManager_Init/Process` 仅保留运行编排职责。
3. `CommandManager_DispatchExecute` 仅保留命令分发与执行编排。
4. `CommandManager_OutputParam/OutputState/OutputDiag` 保持诊断桥接职责单一。

功能级细化：
1. 协议主链行为保持一致（不改协议语义）。
2. 参数池/状态池读写路径行为保持一致。
3. 结构链方向可解释且无循环 include。

验收：
1. 主编排文件不再承载参数存储与查询细节实现。
2. include 检索无循环依赖。
3. 构建通过且无新增 warning。

#### M7：L3 控制算法结构链重排（`foc_control` 整体，`Cxx` 命名）

目标：将 `foc_control.c` 重排为“同一算法整体 + 分层单向文件链”，先做结构收口，不做裁剪策略变更。

文件级细化（命名模板）：
1. `foc/src/L3_Algorithm/foc_control_c01_entry.c`
2. `foc/src/L3_Algorithm/foc_control_c02_cfg_state.c`
3. `foc/src/L3_Algorithm/foc_control_c03_outer_loop.c`
4. `foc/src/L3_Algorithm/foc_control_c04_current_loop.c`
5. `foc/src/L3_Algorithm/foc_control_c05_actuation.c`
6. 配套头文件同名放置于 `foc/include/L3_Algorithm/`
7. 保留并复用：`foc_control_softswitch.c`、`foc_control_compensation.c`（必要时并入 `c05`）

函数级细化：
1. `FOC_ControlOuterLoopStep` 与外环相关 helper 归入 `c03`。
2. `FOC_CurrentControlStep` 与快环相关 helper 归入 `c04`。
3. `FOC_ControlSet*`、`FOC_ControlConfigResetDefault`、状态访问接口归入 `c02`。
4. `FOC_ControlApplyElectricalAngleRuntime/Direct`、执行后处理归入 `c05`。
5. `FOC_OpenLoopStep`、统一入口编排归入 `c01`。

功能级细化：
1. 对 L2 导出接口保持稳定（`motion_control_iface.h`、`control_config_iface.h`）。
2. 文件间调用方向固定为 `C01 -> C02 -> C03 -> C04 -> C05`。
3. 不并发拆分 `sensor.c`、`svpwm.c`。

验收：
1. include 检索不存在 `C04/C05` 反向依赖 `C01/C02`。
2. 控制链行为回归通过。
3. 构建通过且无新增 warning。

#### M8：L2/L3 边界与命名固化（结构收口完成）

目标：固化 L2<->L3 最小接口面与命名规范，消除结构阶段遗留歧义。

文件级细化：
1. `foc/include/L2_Service/motor_control_service.h`
2. `foc/src/L2_Service/motor_control_service.c`
3. `foc/include/L3_Algorithm/motion_control_iface.h`
4. `foc/include/L3_Algorithm/control_config_iface.h`
5. 新增/调整的 `foc_control_c0*.h`

函数级细化：
1. `MotorControlService_RunControlTask` 作为控制执行主入口。
2. `RunOpenLoop/RunOuterLoop/RunCurrentLoop` 若仅兼容壳则转私有或删除对外导出。
3. `ApplyPendingConfig` 继续保持唯一 `FOC_ControlSet*` 调用入口。

功能级细化：
1. L2 不感知 L3 内部分层细节。
2. 命名规范统一为 `Cxx`，不再使用 `L3x` 风格链内编号。

验收：
1. L2 对 L3 调用面仅通过 `*_iface.h` 暴露接口。
2. 命名与依赖链描述在文档中一致。
3. 构建通过且无新增 warning。

### 7.5 裁剪阶段（结构完成后再裁剪）

#### M9：L2 功能裁剪闭环

目标：在 L2 结构稳定后，完成功能开关全链裁剪，不回退到结构改造。

文件级细化：
1. `foc/src/L2_Service/command_manager_store.c`（或承接文件）
2. `foc/src/L2_Service/command_manager_query.c`（或承接文件）
3. `foc/src/L2_Service/motor_control_service.c`
4. `foc/include/L2_Service/command_manager.h`

函数级细化：
1. 围绕 `ANGLE_PID_TUNING`、`SPEED_PID_TUNING`、`CONTROL_FINE_TUNING` 对应 getter/setter 族进行裁剪。
2. 同步裁剪 `MotorControlService_ApplyPendingConfig` 中对应应用调用点。

功能级细化：
1. 关闭宏后字段、导出接口、调用点三者同时清零。
2. 参数脏标记与应用链行为保持一致。

验收：
1. 宏关闭后无残留 getter 导出与上游调用。
2. 协议参数写入与控制应用链回归通过。
3. 构建通过且无新增 warning。

#### M10：L3 算法裁剪闭环（`foc_control_c0*`）

目标：在 L3 结构链稳定后，完成算法开关闭环裁剪，并落实 L3 完全去业务内联。

文件级细化：
1. `foc/src/L3_Algorithm/foc_control_c03_outer_loop.c`
2. `foc/src/L3_Algorithm/foc_control_c04_current_loop.c`
3. `foc/src/L3_Algorithm/foc_control_c02_cfg_state.c`
4. 对应头文件 `foc/include/L3_Algorithm/foc_control_c0*.h`

函数级细化：
1. `FOC_CurrentControlStep` 与相关 helper 完成 `FOC_CURRENT_LOOP_PID_ENABLE` 闭环裁剪。
2. `FOC_ControlRequiresCurrentSample` 与外环入口保持宏一致性。
3. 检查并移除可替代的业务 inline 逻辑（数学 LUT 例外）。

功能级细化：
1. 宏关闭后状态变量、导出接口、调用路径同步收敛。
2. 快速链与外环链行为保持可回归。

验收：
1. 目标宏关闭时无残留无效导出和调用。
2. L3 业务层新增 inline 为 0。
3. 构建通过且无新增 warning。

### 7.6 收口阶段

#### M11：文档、工程清单与临时方案收口

目标：完成代码任务后统一文档与工程清单，关闭执行漂移风险。

文件级细化：
1. `NEXT_MISSION.md`
2. `docs/architecture.md`
3. `docs/development.md`
4. `docs/engineering/layered-architecture-temp-plan-2026-04-16.md`
5. `examples/GD32F303_FOCExplore/software/build/GD32F30X_CL/builder.params`
6. `examples/GD32F303_FOCExplore/software/.eide/eide.yml`
7. 必要 Keil 工程文件

函数级细化：
1. 文档中主链函数名、入口函数名、接口头映射到当前代码。

功能级细化：
1. 三条主链（控制、快速、协议）文档映射一致。
2. 临时方案文档在 M11 验收后再决定归档状态。

验收：
1. 文档事实源无冲突。
2. 构建通过且无新增 warning。

### 7.7 v1.3.4 完成标准（关口）

1. L1 对 L3 的直接 include 与直接函数调用保持为 0。
2. 控制链、快速链、协议链均经 L2 门面进入 L3。
3. 结构阶段（M6~M8）先于裁剪阶段（M9~M10）完成并验收。
4. L2 功能裁剪与 L3 算法裁剪各至少完成一个样板闭环，并保留检索与构建记录。
5. L3 控制算法采用 `Cxx` 命名形成单向依赖链；L2 不要求完全去内联，L3 业务逻辑完全去内联。
6. M6~M11 每项任务均满足“代码 + 检索 + 构建 + 文档 + 回归”五要素，不存在半任务遗留。
