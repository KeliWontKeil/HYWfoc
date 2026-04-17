# 下一阶段任务（NEXT_MISSION）

## 0. 目标与边界

- 当前稳定基线：`v1.3.4`
- 下一目标版本：`v1.3.5`
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

#### M8：L2/L3 链级收口重开（禁止兼容共存）

状态：执行中

目标：按“严格层层单向 + 同层可并行”重建 L2/L3 文件依赖结构，彻底消除兼容并存路径。

R2 合并执行口径：将“C22 收口 + 初始化分层重排 + 协议纯处理单文件化”并入同一轮任务闭环，避免跨轮次漂移；R2 仅做 L3 内重排，不做 L1/L2 大层级提升。

R2 优先执行约束（本轮锁定）：
1. 代码实施与验收仅以本节 R2 细化条目为准；与其他文档存在口径冲突时，统一以本节为唯一执行约束。
2. 在 R2 验收闭环前，R3/R4/R5 的层级提升、命名迁移与范围扩展不进入实施面，只保留为后续规划信息。
3. R2 动作必须保持“同次删除 + 同次清单同步 + 同次构建验证”，禁止拆批执行。

新增评估结论（2026-04-17，驱动任务重排）：
1. 采纳：当前 `C33`（软切换）并入电流环主链，避免同层过细分支导致路径分叉。
2. 采纳：当前 `C34` 职责收口为齿槽补偿算法，统一承接补偿查询与初始化侧补偿准备逻辑（执行时同步重命名）。
3. 采纳：从 `C12` 拆分“电机参数学习算法”为新 `C33`，`C12` 保留初始化组织与标定原语。
4. 采纳：`C11` 去除对当前 `C41` 的直接依赖，执行输出流程改为通过电流环组织路径收口。
5. 修正：R3 采用“两步法”——先做层级合并（旧 `C2` 并入旧 `C3`），再做整体命名提升（旧 `C3/C4` 统一提升为新 `C2/C3`）。

任务顺延规则（本轮生效）：
1. 新增 `R3-L3-Restructure`：承接上述 5 项结构改造，并执行“先合并后提升”的整体编号迁移。
2. 原 `R3-L2-Chain` 顺延为 `R4-L2-Chain`。
3. 原 `R4-List+Build` 顺延为 `R5-List+Build`。

文件级细化：
1. L3 控制算法链必须全量落在 `foc_control_cxy_*` 体系，不允许旧非链文件并存：
	- 入口/上行唯一文件：`foc/src/L3_Algorithm/foc_control_c11_entry.c`。
	- 初始化路径：`foc/src/L3_Algorithm/foc_control_c12_init.c`。
	- 配置状态：`foc/src/L3_Algorithm/foc_control_c21_cfg_state.c`。
	- 过渡分发壳：`foc/src/L3_Algorithm/foc_control_c22_runtime_dispatch.c`（已并入 `C11` 并删除）。
	- 外环分支：`foc/src/L3_Algorithm/foc_control_c31_outer_loop.c`。
	- 电流环分支：`foc/src/L3_Algorithm/foc_control_c32_current_loop.c`。
	- 并行算法分支（R3 前状态）：`foc/src/L3_Algorithm/foc_control_c33_softswitch.c`、`foc/src/L3_Algorithm/foc_control_c34_compensation.c`。
	- R3 目标：当前 `C33` 并入 `C32`；从 `C12` 拆出“电机参数学习算法”为新 `C33`；当前 `C34` 收口为齿槽补偿算法。
	- R3 层级合并：旧 `C2`（配置状态层）并入旧 `C3`（控制算法层），不再保留独立 `C2` 层。
	- R3 命名提升：在完成“旧 `C2->C3` 合并”后，旧 `C3/C4` 整体提升为新 `C2/C3`（例如 `C34->C24`、`C41->C31`）。
	- 执行输出（R3 目标）：`C11` 不再直连执行层（旧 `C41` / 新 `C31`），电角应用/执行后处理收口到电流环组织链。
	- 必删：`foc_control.c`、`foc_control.h`（禁止兼容壳）。
2. L3 非控制链算法（协议纯处理）必须单文件：
	- 合并结果：`foc/src/L3_Algorithm/protocol_core.c`。
	- 已收敛来源：`protocol_core_parser.c`、`protocol_core_normalize.c`、`protocol_text_codec.c`（源文件删除，功能并入单实现入口）。
3. L2 链式与并列模块细化：
	- 控制门面：`foc/src/L2_Service/motor_control_service.c`（统一入口保留，兼容壳删除）。
	- 协议主链：`foc/src/L2_Service/protocol_service.c` -> `foc/src/L2_Service/protocol_parser.c` -> `foc/src/L2_Service/command_manager.c`。
	- 命令执行链：`command_manager.c` -> `command_manager_dispatch.c` -> `command_manager_store/query` -> `command_manager_diag.c`。
	- 非链同域聚合目标：`command_manager_store.c` 与 `command_manager_query.c` 合并为单文件实现。
4. 工程清单强制同步：`examples/GD32F303_FOCExplore/software/build/GD32F30X_CL/builder.params`、`examples/GD32F303_FOCExplore/software/.eide/eide.yml`。

函数级细化：
1. L3 控制链函数归属（全链迁移，不得留在旧文件）：
	- C11（R2 后）：`FOC_ControlOuterLoopStep`、`FOC_OpenLoopStep`，并吸收原 `C22` 的运行分发与执行桥接职责；作为 L2 唯一上行调用入口文件。R3 目标：移除对当前 `C41` 的直接头文件依赖。
	- C12：`FOC_MotorInit`、`FOC_CalibrateElectricalAngleAndDirection`（初始化算法原语）。
	- C21（R3 前状态）：`FOC_ControlConfigResetDefault`、全部 `FOC_ControlSet*`、`FOC_PIDInit`、控制状态访问函数（运行配置/状态库）；R3 中并入旧 C3 族，不再保留独立 C2 层。
	- C31（R3 前状态）：`FOC_SpeedOuterLoopStep`、`FOC_SpeedAngleOuterLoopStep`、机械角累积与速度误差相关 helper；R3 后提升为新 `C21`。
	- C32（R3 前状态）：`FOC_CurrentControlStep`、`FOC_ControlRequiresCurrentSample`、电流环闭环融合逻辑；R3 后提升为新 `C22`。
	- C33（R3 中间态）：承接从 `C12` 拆出的电机参数学习算法；完成整体提升后归入新 `C23`。
	- C34（R3 前状态）：补偿并行分支；R3 后提升为新 `C24`。
	- 当前 C33（软切换）在 R3 并入 C32，不再保留独立并行文件。
	- C41（R3 前状态）：`FOC_ControlApplyElectricalAngleRuntime/Direct`、电角到占空比执行与后处理；R3 后提升为新 `C31`，且不再作为 C11 直连依赖点。
	- C21/C12 评估结论：不做整文件合并；`C21` 保持运行态配置/状态库，`C12` 保持初始化算法原语，避免生命周期耦合。
2. R3 层级合并与命名提升映射（冻结，执行时按此迁移）： 
	- 第一步（层级合并）：旧 `C2` 并入旧 `C3`，旧 `C2` 不再作为独立层存在。
	- 第二步（整体提升）：旧 `C3 -> 新 C2`，旧 `C4 -> 新 C3`。
	- 映射示例（强制）：`C31->C21`、`C32->C22`、`C33->C23`、`C34->C24`、`C41->C31`。
	- 旧 `C21` 不做一对一改号，按职责拆分并入新 `C2x` 文件族。
3. L3 内部依赖规则：
	- 禁止在 `Cxx` 链中使用 `foc_control_internal.h` 作为公共穿透头。
	- 允许新增 `foc_control_cxx_*_private.h` 作为最小私有声明，但不得跨两级传播。
4. L2 控制服务函数收口：
	- 保留：`MotorControlService_RunControlTask`、`MotorControlService_ApplyPendingConfig`。
	- 删除外露兼容壳：`RunOpenLoop`、`RunOuterLoop`、`RunCurrentLoop`（若仅转发）。
5. L2 命令域函数收口：
	- 合并文件后仍保留函数边界：`WriteParam/ReadParam/WriteState/ReadState/ReportAllParams/ReportAllStates`。
	- `CommandManager_DispatchExecute` 只负责分发，不得回流写入细节。

功能级细化：
1. 行为保持不变：协议语义、控制实时路径与保护策略不改变。
2. 结构一致性优先：调用链唯一、无回流、无跨级 include。
3. 向上传递单文件约束：L3 控制链对 L2 的运行入口只允许一个上行实现文件（`C11`），其余链内文件仅链内可见。
4. 初始化职责拆分（R2）：算法组织级（流程编排、步骤切换、失败回退）与 `C11` 同层保留在 L3；控制算法级（锁定/采样/估计/补偿学习）固化在 L3 初始化原语层。
5. 删除动作与工程清单更新必须同次提交闭环。

验收：
1. 检索不存在 `foc_control.c`、`foc_control.h` 在控制链工程清单或源码依赖中。
2. `Cxx` 源码检索不存在通过 `foc_control_internal.h` 做跨级公共入口的用法。
3. L3 协议纯处理仅保留单文件实现入口。
4. L2 `command_manager_store/query` 完成同域合并，接口函数保持。
5. L3 对 L2 的运行入口仅保留单文件（C11）调用面。
6. `foc_control_c22_runtime_dispatch.*` 不再保留独立构建条目（职责并入 C11 后删除）。
7. R2 完成后初始化组织级与运行组织级同层（均在 L3-C1），且不发生 L1/L2 层级提升。
8. R3 完成后，`C11` 源码与头文件依赖中不再出现对当前 `C41` 的直接 include/直接调用。
9. R3 完成后，当前 `C33`（软切换）独立实现文件不再保留，相关逻辑并入电流环主链。
10. R3 完成后，旧 `C2` 已并入旧 `C3`，源码中不再保留独立 C2 层边界。
11. R3 完成后，旧 `C3/C4` 已整体提升为新 `C2/C3`，至少满足 `C34->C24`、`C41->C31` 映射。
12. R3 完成后，从 `C12` 拆分出的电机参数学习算法最终归入新 `C23`，职责与齿槽补偿分离。
13. `phase-d-rebuild` 通过且无新增 warning。

### 7.4.1 宏裁剪前前置任务（新增）

#### M8A：L2 调用链结构化（类 Cxy 方式）

目标：在不改变协议/控制行为的前提下，把 L2 层内耦合文件收敛为“入口编排 -> 分发执行 -> 存储查询 -> 诊断输出”的单向调用链，并形成可检索的链级编号口径。

文件级细化：
1. `foc/src/L2_Service/command_manager.c`（L2-C11：入口编排）
2. `foc/src/L2_Service/command_manager_dispatch.c`（L2-C21：分发执行）
3. `foc/src/L2_Service/command_manager_store.c`（L2-C22：存储/查询统一承载）
4. `foc/src/L2_Service/command_manager_diag.c`（L2-C23：诊断输出）
5. `foc/src/L2_Service/motor_control_service.c`（L2-C31：控制门面）
6. `foc/include/L2_Service/motor_control_service.h`

函数级细化：
1. 删除对外兼容壳：`MotorControlService_RunOpenLoop/RunOuterLoop/RunCurrentLoop`（仅保留 `RunControlTask` 统一入口）。
2. `CommandManager_DispatchExecute` 仅保留分发，不回流参数存储实现。
3. `CommandManager_Read*/ReportAll*` 并入 `store` 统一文件后，保持接口名不变。

验收：
1. L2 主链方向可检索：`command_manager.c -> dispatch -> store -> diag`。
2. `motor_control_service.h` 不再导出兼容壳函数声明。
3. `phase-d-rebuild` 通过且无新增 warning。

#### M8B：L3 结构残留清理（宏裁剪前）

目标：在宏裁剪前完成 L3 结构化残留清理，优先清理“可删除但无行为价值”的文件边界与冗余 helper，避免在 M10 混入结构改造。

文件级细化：
1. `foc/src/L3_Algorithm/protocol_core.c`
2. `foc/include/L3_Algorithm/protocol_core_parser.h`
3. `foc/include/L3_Algorithm/protocol_core_normalize.h`
4. `foc/include/L3_Algorithm/protocol_text_codec.h`
5. `foc/src/L3_Algorithm/foc_control_c22_current_loop.c`
6. `foc/src/L3_Algorithm/foc_control_c24_compensation.c`

函数级细化：
1. 识别并清理仅单调用点且不承载语义边界的冗余 wrapper/helper。
2. 保持 `C11 -> C22` 桥接边界，不回退为 `C11` 直连执行层。
3. 头文件职责最小化：避免“一个实现文件 + 多个过细头文件”长期并行且跨层扩散。

验收：
1. 不存在孤儿 `.c/.h`（工程清单外残留实现或无引用头）。
2. 构建无新增“声明未引用/不可达”告警。
3. 文档函数归属与源码一致。

### 7.5 裁剪阶段（结构完成后再裁剪）

前置条件：M8A 与 M8B 验收通过。

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

#### M10：L3 算法裁剪闭环（`foc_control_cxy_*`）

目标：在 L3 结构链稳定后，完成算法开关闭环裁剪，并落实 L3 完全去业务内联。

文件级细化：
1. `foc/src/L3_Algorithm/foc_control_c21_outer_loop.c`
2. `foc/src/L3_Algorithm/foc_control_c22_current_loop.c`
3. `foc/src/L3_Algorithm/foc_control_c25_cfg_state.c`
4. 对应头文件 `foc/include/L3_Algorithm/foc_control_cxy_*.h`

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

### 7.7 v1.3.5 完成标准（关口）

1. L1 对 L3 的直接 include 与直接函数调用保持为 0。
2. 控制链、快速链、协议链均经 L2 门面进入 L3。
3. 结构阶段（M6~M8B）先于裁剪阶段（M9~M10）完成并验收。
4. L2 功能裁剪与 L3 算法裁剪各至少完成一个样板闭环，并保留检索与构建记录。
5. L3 控制算法采用 `C11/C12 -> C2x -> C31` 命名形成单向依赖链；L2 不要求完全去内联，L3 业务逻辑完全去内联。
6. M6~M11 每项任务均满足“代码 + 检索 + 构建 + 文档 + 回归”五要素，不存在半任务遗留。
