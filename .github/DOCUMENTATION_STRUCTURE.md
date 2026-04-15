# 文档结构与归属说明

本文件定义仓库文档归属边界与维护规则。

## 文档归属表

| 范围 | 路径 | 责任 |
|---|---|---|
| 项目入口与发布信息 | `README.md`、`CHANGELOG.md`、`NEXT_MISSION.md`、`LICENSE` | 项目介绍、版本记录、任务规划、许可证 |
| AI 全局治理 | `copilot-instructions.md`、`AI_INITIALIZATION.md` | AI 开发规则、流程约束、协作口径 |
| AI 工作流编排 | `.github/AGENTS.md`、`.github/WORKFLOW_CHECKLIST.md`、`.github/agents/*.agent.md` | 代理职责、执行清单、执行入口 |
| 库级技术文档 | `docs/*.md` | 架构、流程、协议、评估记录 |
| 实例级文档 | `examples/<instance>/**/*.md` | 板级硬件、实例构建、通道映射 |

## 结构事实源（SSOT）

1. 结构与依赖唯一主文档：`docs/architecture.md`
2. 不保留结构类兼容跳转文档；出现重复结构文档时直接合并或删除。

## 更新规则

1. 同一主题只保留一个事实源，其他文档只链接不重复叙述。
2. 架构/接口/时序/依赖变化：同次迭代更新 `docs/architecture.md`。
3. 工作流或提示词变化：同次迭代更新 `.github` 文档与 `copilot-instructions.md`。
4. 实例硬件或驱动映射变化：更新对应实例目录文档，不写入库级文档。

## 推荐阅读顺序

1. `README.md`
2. `docs/README.md`
3. `docs/architecture.md`
4. `docs/development.md`
5. `NEXT_MISSION.md`
