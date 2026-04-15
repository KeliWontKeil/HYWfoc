# Copilot 工作流检查清单

## 1. 编码前检查

- [ ] 阅读 `NEXT_MISSION.md`，确认当前阶段目标
- [ ] 阅读 `copilot-instructions.md` 与 `docs/development.md`
- [ ] 若上下文被压缩或会话切换，重新阅读关键文档与关键代码文件
- [ ] 判定任务类型：改代码 / 仅设计 / 暂缓
- [ ] 选择 Agent（见 `.github/AGENTS.md`）
- [ ] 使用正确工作区（实例工作区用于构建/调试）

## 2. 编码中检查

- [ ] 命名与宏风格符合规范
- [ ] 新增可调参数先进入 `foc_cfg_*.h`
- [ ] 公共头文件不引入 `gd32f30x_*`
- [ ] `L1/L2/L3` 访问硬件仅走 `foc_platform_api`
- [ ] 禁止为兼容旧结构增加跨层桥接或文档跳转壳
- [ ] 限幅/归一化链路无重复叠加且有必要性

## 3. 构建验证流程

### 第一步：常规构建

执行 `eide.project.build`，要求：0 error + 不新增 warning。

### 第二步：问题复核

若日志不全或结果可疑：

1. 用 `terminal_last_command` 回看终端构建结果。
2. 需要时运行 `unify_builder --rebuild` 作为最终判定。
3. 若报 `Not found any source files`，检查 `builder.params` 中 `sourceList`。

### 第三步：资源与链接

- [ ] ROM/RAM 未超预算
- [ ] 无未定义符号与重复符号
- [ ] 无新增层级越界引用

## 4. 文档同步检查

- [ ] 结构/依赖变化已更新 `docs/architecture.md`
- [ ] 流程变化已更新 `.github/*` 与 `copilot-instructions.md`
- [ ] 版本或里程碑变化已更新 `NEXT_MISSION.md` 和必要的 `CHANGELOG.md`
- [ ] 库级文档未写死控制频率，统一使用“配置宏决定”口径
- [ ] 不存在重复结构事实源或兼容跳转页

## 5. 提交前检查

- [ ] 代码可编译且无新增 warning
- [ ] 文档已同步
- [ ] 本次提交范围清晰，未夹带无关生成物

提交治理：

1. 完成周期后本地 `git commit`
2. 默认不 `git push`
3. 当前提交修订优先 `git commit --amend`

## 6. 常见排障要点

1. `get_errors` 可能有过期告警，最终以真实编译为准。
2. 新增配置宏时报未定义，优先检查是否通过 `foc_config.h` 接入。
3. 功能裁剪宏关闭后，应把受控声明/定义/调用放入同一 `#if` 块。

## 7. 任务驱动说明

1. 当前仓库不维护独立提示词目录，统一以 `NEXT_MISSION.md` + `.github/AGENTS.md` 驱动执行。
2. 若后续恢复提示词文件，需在同次迭代同步更新本清单与 `.github/DOCUMENTATION_STRUCTURE.md`。
