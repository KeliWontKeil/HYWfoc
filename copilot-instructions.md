---
description: "HYWfoc 仓库级开发说明。用于控制算法开发、外设接入、构建排错、架构审查与提交治理。"
---

# Copilot 仓库级开发说明

## 1. 项目定位

`HYWfoc` 采用“核心库 + 实例工程”组织方式：

- `foc/`：平台无关可复用库
- `examples/<instance>/`：板级工程与驱动实现
- `docs/`：库级文档与流程

当前稳定基线：`v1.3.2`

## 2. 工作区职责

1. 根工作区（仓库根 `Project.code-workspace`）仅用于管理与文档治理。
2. 实例工作区（如 `examples/GD32F303_FOCExplore/software/Project.code-workspace`）是构建/烧录/调试入口。

## 3. 强制分层约束

1. `L1/L2/L3` 访问硬件只能走 `foc_platform_api`。
2. 公共头文件不暴露 `gd32f30x_*` 设备头。
3. 配置参数只能从 `foc_config.h` + `foc_cfg_*.h` 获取，禁止散落 `.c`。
4. 结构事实源以 `docs/architecture.md` 为唯一主文档。

## 4. 配置收敛规则

1. 默认值：`foc_cfg_init_values.h`
2. 功能裁剪：`foc_cfg_feature_switches.h`
3. 编译约束：`foc_cfg_compile_limits.h`
4. 符号与兼容定义：`foc_cfg_symbol_defs.h`

## 5. 构建与验证

### 构建要求

1. 构建命令：`eide.project.build` 或实例内重构建命令。
2. 结果要求：0 error，且不引入新增 warning。
3. 关注资源：ROM ≤ 256KB，RAM ≤ 96KB。

### 调试经验要点

1. VS Code 任务有时不显示完整日志，可用 `terminal_last_command` 回看最后构建输出。
2. `unify_builder.exe` 可能需要 `DOTNET_ROLL_FORWARD=Major`。
3. 遇到 `Not found any source files`，先查 `builder.params` 的 `sourceList` 是否过期。
4. `get_errors` 可能有残留诊断，最终以真实编译结果为准。

## 6. 提交治理

1. 默认在 `main` 直接开发。
2. 每次完整修改后仅做本地 `git commit`。
3. 默认不 `git push`，仅在用户明确要求时执行。
4. 若用户要求修订当前提交，优先 `git commit --amend`。

## 7. 文档同步要求

1. 架构、依赖、时序变化：必须更新 `docs/architecture.md`。
2. 工作流变化：必须同步 `.github` 与本文件。
3. 版本目标变化：同步 `NEXT_MISSION.md` 与 `CHANGELOG.md`。

## 8. 代理与执行入口

仓库当前代理：

1. `@primary-developer`：常规开发与落地
2. `@foc-algorithm-review`：控制算法问题与调参
3. `@architecture-review`：结构、依赖、分层治理
4. `@documentation-compliance`：文档一致性审查

当前不维护独立提示词目录，统一按 `NEXT_MISSION.md` + `.github/AGENTS.md` 执行。
