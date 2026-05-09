# Copilot Instructions

## 当前版本基线：v1.7.0

This file provides Copilot with project-context awareness.

### 核心规则

1. 请参考 `.clinerules/` 下的规则文件获取项目完整上下文
2. 当前稳定基线为 v1.7.0，语义化版本
3. 项目采用"核心库 `foc/` + 实例工程 `examples/`"组织方式
4. 依赖方向严格单向：`LS → L1 → L2 → L3 → L41 → L42 → L5`
5. 控制算法和硬件平台通过 `foc_platform_api` 解耦
6. 配置常量收敛在 `foc_cfg_*.h`，类型定义收敛在 `foc_*_types.h`

### 文档同步要求

每次结构/依赖变化时需同步更新：
- `docs/architecture.md` — 唯一结构说明
- `docs/development.md` — 开发流程
- `CHANGELOG.md` — 变更记录
- `NEXT_MISSION.md` — 下一阶段目标

### 其他

遵循 `.clinerules/hywfoc-project-rules.md` 中的完整项目规则。
