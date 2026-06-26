# AI 项目初始化（仓库级）

## 你当前所处仓库

你正在 `HYWfoc（何易位FOC）` 仓库工作，核心结构如下：

- `foc_core/`：可复用控制库（平台无关）
- `examples/`：实例工程（板级与工具链相关）
- `docs/`：库级文档与开发流程

本文件是"仓库级初始化约束"，不是实例说明书。

## 作用边界（必须遵守）

本文件负责：

1. 仓库级开发流程
2. 协作与提交治理
3. 文档同步规则

本文件不承载：

1. 管脚映射
2. 板级连线
3. 单实例构建输出路径
4. 特定实例通信通道绑定

以上内容必须写在 `examples/<instance>/` 对应文档中。

## 实施前必读

1. `.clinerules/hywfoc-project-rules.md`
2. `docs/development.md`
3. `docs/architecture.md`
4. `NEXT_MISSION.md`
5. 需要专项审查时查看 `.github/AGENTS.md`

## 全局工作流（AI开发）

1. 先读任务与规则，再动代码。
2. 默认在 `main` 开发，除非用户明确要求分支。
3. 构建与验证在实例工作区完成。
4. 代码改动与文档改动同迭代提交。
5. 严格执行本地提交治理。

## 工作区职责

1. 根工作区 `Project.code-workspace`：代码审查、架构治理、文档维护。
2. 实例工作区（如 `examples/GD32F303_FOCExplore/software/Project.code-workspace`）：构建、烧录、调试。

## 强制分层约束

1. `L1/L2/L3` 访问硬件只能走 `foc_platform_api`。
2. 公共头文件不暴露 `gd32f30x_*` 设备头。
3. 配置参数只能从 `foc_config.h` + `foc_cfg_*.h` 获取，禁止散落 `.c`。
4. 结构事实源以 `docs/architecture.md` 为唯一主文档。

## 配置收敛规则

1. 默认值：`foc_cfg_init_values.h`
2. 功能裁剪：`foc_cfg_feature_switches.h`
3. 编译约束：`foc_compile_limits.h`
4. 符号与兼容定义：`foc_symbol_defs.h`

## 提交与版本治理（强制）

### 提交规则

1. 每次完整修改周期结束后，只做本地 `git commit`。
2. 未经用户明确指令，不执行 `git push`。

### 版本规则

采用 `MAJOR.MINOR.PATCH` 语义；其中 PATCH 可作为本地修订序号。

### 修订规则

若用户要求修改当前提交，优先使用 `git commit --amend`，不新增独立提交。

### 生效基线

本治理在 `>= 1.0.0` 基线下持续生效。

## 通用技术要求

1. 命名、宏、分层遵循仓库规则。
2. 使用 `NULL` 时确保包含 `<stddef.h>`。
3. ISR 路径禁止阻塞操作。
4. 保持 ROM/RAM 与实时路径成本可控。
5. 嵌入式行为以硬件验证为最终闭环。

## 构建与验证

### 构建要求

1. 构建命令：实例内的重构建命令。
2. 结果要求：0 error，且不引入新增 warning。
3. 关注资源：ROM ≤ 256KB，RAM ≤ 96KB（当前 GD32F303CC 实例限制）。跨平台移植建议 ROM ≤ 64KB，RAM ≤ 16KB。

### 调试经验要点

1. VS Code 任务有时不显示完整日志，优先查看终端完整构建输出。
2. `unify_builder.exe` 可能需要设置 `DOTNET_ROLL_FORWARD=Major`。
3. 遇到 `Not found any source files`，先查 `builder.params` 的 `sourceList` 是否过期。
4. `get_errors` 可能有残留诊断，最终以真实编译结果为准确。

## 文档入口

1. `README.md`：项目总览
2. `docs/README.md`：库文档索引
3. `docs/architecture.md`：唯一结构说明（SSOT）
4. `docs/development.md`：开发流程与经验沉淀
5. `examples/<instance>/README.md`：实例入口

结论：该仓库优先级始终是"稳定性、时序确定性、硬件可验证性"。