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
2. 逐项确认任务属性：改代码 / 仅设计 / 暂缓。
3. 评估影响层级与回滚策略。

### 2. 实施

1. 遵循 `docs/engineering/dev-guidelines/rules/`。
2. 默认在 `main` 直接开发，除非用户明确要求分支。
3. `L1/L2/L3` 不得直接依赖板级驱动。
4. 可配置参数必须先落到 `foc_cfg_*.h`，再在 `.c` 使用。

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

### 配置收敛

1. `foc/include/config/foc_config.h` + `foc_cfg_*.h` 是唯一配置源。
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

## 质量门禁

1. 不得新增告警（no newly introduced warnings）。
2. 文档必须与代码在同次迭代内同步。
3. 接口变更需评估实例适配影响。

## 参考文档

- `../examples/GD32F303_FOCExplore/DEVELOPMENT.md`
- `../examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md`
