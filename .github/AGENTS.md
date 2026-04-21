# Agent 协同说明

本文件定义仓库内各 Agent 的职责与调用时机。

## 1. Agent 列表与适用场景

### `primary-developer`

适用：常规功能开发、缺陷修复、配置更新、重构落地、构建验证。

转交条件：

1. 控制效果或算法正确性问题 -> `foc-algorithm-review`
2. 分层耦合或目录结构问题 -> `architecture-review`
3. 文档一致性问题 -> `documentation-compliance`

### `foc-algorithm-review`

适用：振荡、发散、迟滞、标定链路、PID 调参与控制周期假设核查。

重点文件：

1. `foc/src/L3_Algorithm/foc_control*.c`
2. `foc/src/L3_Algorithm/svpwm.c`
3. `foc/src/L1_Orchestration/foc_app.c`
4. `foc/include/LS_Config/foc_cfg_*.h`

### `architecture-review`

适用：层间越界、头文件耦合、依赖环、配置散落、文件归属优化。

重点文件：

1. `foc/include/**/*.h`
2. `foc/include/L42_PAL/foc_platform_api.h`
3. `examples/**/Application/Source/foc_platform_api.c`
4. `examples/**/Utilities/**`

### `documentation-compliance`

适用：代码与文档漂移、版本口径冲突、重复文档治理、缺失说明补齐。

重点文件：

1. `docs/**/*.md`
2. `README.md`
3. `NEXT_MISSION.md`
4. `CHANGELOG.md`

## 2. 调用决策

1. 需求不明确或混合任务：先用 `primary-developer`。
2. 涉及算法行为变化：并行或后续调用 `foc-algorithm-review` 复核。
3. 涉及结构重排：调用 `architecture-review`。
4. 任何接口/流程/版本变化：最后调用 `documentation-compliance` 做收口。

## 3. 推荐协作链

1. 代码改造链：`primary-developer` -> `architecture-review`（可选） -> `foc-algorithm-review`（可选） -> `documentation-compliance`
2. 文档治理链：`documentation-compliance` -> `primary-developer`（仅当发现需改代码）
3. 算法问题链：`foc-algorithm-review` -> `primary-developer` -> `documentation-compliance`

## 4. 统一收尾标准

1. 编译通过且不得新增 warning。
2. 分层约束不被破坏。
3. `docs/architecture.md` 与实际结构一致。
4. `NEXT_MISSION.md` 与当前阶段一致。

