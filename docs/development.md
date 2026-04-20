# 开发流程指南

## 适用范围

本文件定义仓库级开发流程与验证准则，适用于可复用库（`foc/`）与文档治理。
实例专有构建/烧录细节请看 `examples/<instance>/DEVELOPMENT.md`。

## 工作区职责

- 根工作区 `Project.code-workspace`：仓库管理、代码审查、文档维护。
- 实例工作区 `examples/<instance>/software/Project.code-workspace`：唯一构建/烧录/调试入口。

## 标准开发流程

### 1. 任务确认

1. 先读 `NEXT_MISSION.md` 明确目标与边界。
2. 在 v1.3.5 阶段，执行 M9 前需同时读取 `docs/engineering/layered-architecture-temp-plan-2026-04-16.md`，并以其阶段性约束作为实施前复核基线。
3. 逐项确认任务属性：改代码 / 仅设计 / 暂缓。
4. 评估影响层级与回滚策略。

### 2. 实施

1. 遵循 `docs/engineering/dev-guidelines/rules/`。
2. 默认在 `main` 直接开发，除非用户明确要求分支。
3. `L1/L2/L3` 不得直接依赖板级驱动。
4. 可配置参数必须先落到 `foc_cfg_*.h`，再在 `.c` 使用。
5. L3 控制算法重构按“功能链编号文件（Cxx）+ 单向依赖链”推进，禁止先拆文件后补依赖方向说明。

### 3. 验证

1. 编译必须满足：0 error，且不得引入新增 warning。
2. 检查层级边界、包含关系、接口签名。
3. 涉及运行行为变更时，补充实例级硬件验证。
4. 会话上下文被压缩或切换后，必须重新读取关键文档与改动代码再继续实施。

### 4. 文档同步

1. 架构/接口/时序变化：同步更新 `docs/architecture.md`。
2. 工作流变化：同步更新 `.github/` 与 `copilot-instructions.md`。
3. 版本与里程碑：同步 `CHANGELOG.md` 与 `NEXT_MISSION.md`。
4. 库级文档不得写死控制频率，统一使用“配置宏决定的控制周期”表述。
5. 非必要不新增文档；优先更新现有事实源。
6. 涉及协议链分层口径调整时，`docs/architecture.md`、`NEXT_MISSION.md` 与临时方案文档必须同次迭代对齐。
7. 临时方案文档 `docs/engineering/layered-architecture-temp-plan-2026-04-16.md` 保留至 M11 完成并验收后再归档。
8. 涉及 L3 控制算法编号重构（`foc_control_c01~c05_*`）时，必须同次更新 `docs/architecture.md` 与 `NEXT_MISSION.md` 的命名/依赖链描述。

## P0 可读性与可维护性验收

### 必须满足

1. `main` 只调用 `L1` 库入口和 `L5` 硬件初始化入口。
2. 无跨层/反向依赖，`L1/L2/L3` 访问硬件仅通过 `foc_platform_api`。
3. 裁剪开关分层明确：算法裁剪集中在 `L3`，功能裁剪集中在 `L2`。
4. 关键约束可落地到检查动作（代码阅读、包含关系检查、构建验证）。

### 拒收条件

1. 同一结构事实出现在多个文档并行维护。
2. 为兼容旧结构保留嵌套跳转文档。
3. 限幅/归一化在同链路重复叠加且无行为收益说明。
4. 引入无意义中间变量导致链路可读性下降。

### 5. 提交治理

1. 每个完整修改周期后执行本地 `git commit`。
2. 默认禁止 `git push`，仅用户明确要求时允许。
3. 若用户要求修改当前提交，优先 `git commit --amend`。
4. 版本语义采用 `MAJOR.MINOR.PATCH`；本地迭代按约定递增修订号。

## 编码与依赖约束

### 分层依赖

1. 控制与业务层访问硬件，仅能通过 `foc_platform_api`。
2. 公共头文件不得暴露 `gd32f30x_*`。
3. 共享类型统一放在 `foc_shared_types.h`。

### L3 控制链文件规则

1. `foc_control` 相关实现按功能链编号命名：`c01`（入口编排）-> `c02`（配置状态）-> `c03`（外环）-> `c04`（快环）-> `c05`（执行后处理）。
2. 仅允许高层依赖低层，禁止低层 include 高层头文件。
3. 去内联约束分层处理：L2 允许少量受控 inline（模块内私有且可检索）；L3 除 `L41_Math` 外不新增业务 inline 头，业务共享逻辑必须落到 `.c` 分层文件。

### 配置收敛

1. `foc/include/LS_Config/foc_config.h` + `foc_cfg_*.h` 是唯一配置源。
2. 禁止在 `.c` 文件散落默认值和编译约束。

### 命名规则

- 函数：`ModuleName_FunctionName`
- 变量：`camelCase`
- 宏：`UPPER_CASE`
- 类型：`type_name_t`

## 编译与调试经验（长期沉淀）

以下条目为已验证经验，文档与仓库记忆需保持同一表述。

1. `eide.project.build` 在 VS Code 里可能拿不到完整终端输出；必要时用 `terminal_last_command` 回看最后构建日志。
2. 手动运行 `unify_builder.exe` 可能需要设置 `DOTNET_ROLL_FORWARD=Major`。
3. 若出现 `Not found any source files`，优先检查 `build/GD32F30X_CL/builder.params` 的 `sourceList` 是否仍指向旧路径。
4. `get_errors` 可能残留过期 IntelliSense 告警；以 `unify_builder --rebuild` 或基于 `compile_commands.json` 的真实编译结果为准。
5. 新增配置宏时，若源文件未通过 `foc_config.h` 接入，容易出现宏未定义问题。
6. 关闭某功能宏后，受控变量与函数声明/定义/调用应在同一条件编译块中，避免“声明但未引用”告警。
7. 当 L2 不再直接调用控制层 setter 时，影响控制行为的状态写入（例如电流软切换使能）必须置 `params_dirty`，由 L1 在统一参数应用阶段下发，避免“状态改变但运行行为未刷新”。

## 质量门禁

1. 不得新增告警（no newly introduced warnings）。
2. 文档必须与代码在同次迭代内同步。
3. 接口变更需评估实例适配影响。

## v1.3.3 回归验证记录（P3）

### 编译与告警

1. 验证方式：`unify_builder.exe -p build/GD32F30X_CL/builder.params --rebuild`。
2. 验证结果：`0 error`，`1 warning`。
3. warning 明细：`L6914W: option rwpi ignored when using --scatter.`（历史已存在，不属于本轮新增）。

### 资源占用

1. Program Size：`Code=43592`、`RO-data=6168`、`RW-data=268`、`ZI-data=2636`。
2. ROM 估算：`50028 bytes`（`Code + RO-data + RW-data`）。
3. RAM 估算：`2904 bytes`（`RW-data + ZI-data`）。
4. 本轮结论：本次结构重排后的资源增量处于可接受范围，且未引入新增告警。

### 启动标定链路核查

1. 链路顺序：电角锁定 -> 机械零位采样 -> 方向/极对数估计。
2. 代码落点：`foc_control_init.c` 中 `FOC_MotorInit`、`FOC_CalibrateElectricalAngleAndDirection`、`FOC_EstimateDirectionAndPolePairs`。
3. 结论：初始化链路保持 direct 占空比下发，不依赖运行态 ISR 插值路径。

### 运行链路核查

1. 链路顺序：传感器采样 -> 外环计算 -> 快速电流环目标发布 -> PWM Update ISR 插值/电流环执行。
2. 代码落点：`foc_app.c` 中 `Motor_Control_Loop`、`FOC_App_RunControlAlgorithm`、`FOC_App_OnPwmUpdateISR` 与 `svpwm.c` 中 `SVPWM_InterpolationISR`。
3. 结论：运行态 SVPWM 插值链保持独立，未回退为初始化态直通路径。

## v1.3.5 M6/M8（R3）结构落地验证记录（2026-04-17）

### M6（L2 结构链）

1. L2 命名已统一为 `runtime_c1_entry.c` 到 `runtime_c5_output_adapter.c`，运行链固定为 `C1->C2->C3->C4->C5`。
2. `C1` 仅保留运行编排；`C2` 负责帧源接入与解析；`C3` 负责状态机策略；`C4` 负责命令执行与运行态存储；`C5` 负责协议输出适配。
3. 检索验证通过：`C3` 不再直接读写 `C4` 运行态结构体字段，改为调用 `C4` 语义 API。

### M8（L3 Cxx 控制链，R3 收口）

1. `foc_control` 运行实现已稳定在 `C11/C12 -> C21/C22/C23/C24/C25 -> C31` 分层链路。
2. 软切换独立文件已并入电流环主链；补偿分支与电机参数学习分支已按新层级拆分。
3. 检索验证通过：`C11` 不再直连执行层，执行输出经电流环组织路径收口。

### 构建结果

1. `phase-d-verify-build` 因本机缺少 `eide` 命令不可用。
2. 按约束改用 `phase-d-rebuild`，构建通过（0 error）。
3. Warning 维持历史项：`L6914W`（linker rwpi + scatter），无新增 warning。

## 参考文档

- `../examples/GD32F303_FOCExplore/DEVELOPMENT.md`
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`
