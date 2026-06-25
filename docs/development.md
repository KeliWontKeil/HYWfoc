# 开发流程指南

## 适用范围

本文件定义仓库级开发、验证与文档同步规则，适用于：

1. 可复用库：`foc_core/`
2. 工作流文档：`docs/`、`.github/`

实例专属构建与烧录细节请看：`examples/<instance>/DEVELOPMENT.md`。

## 工作区职责

1. 根工作区 `Project.code-workspace`：代码审查、架构治理、文档维护。
2. 实例工作区 `examples/<instance>/software/Project.code-workspace`：构建、烧录、调试。

## 标准流程

### 1. 任务确认

1. 先读 `NEXT_MISSION.md`，确认范围和验收口径。
2. 明确任务属性：改代码 / 仅设计 / 暂缓。
3. 评估影响层级与回滚点。

### 2. 实施

1. 遵循分层约束：`LS → L1 → L2 → L3 → L5`（L2 下分 Control/Protocol/Runtime，各块间不直接调用）。
2. 默认在 `main` 分支工作，除非用户明确要求新分支。
3. 所有可配置参数先进入 `foc_core/include/LS_Config/foc_cfg_*.h`，再在 `.c` 中使用。
4. 运行时主循环由 L1 编排，三个任务段顺序无关，无固定管线链：
   - Monitor 段：调试流生成器逐行输出 → 入 TX 队列
   - Service 段：RX 出队 → 协议单帧处理 → 编排结果（状态码直写、摘要入 TX 队列、配置脏检查）
   - TX 消费段：TX 队列出队 → 平台发送
5. L2/Control 模块命名：`foc_ctrl_executor/cfg/init/outer_loop/current_loop/param_learn/compensation/cogging_calib/reinit/actuation`。
   齿槽标定（cogging_calib）和重初始化（reinit）以非阻塞状态机形式由 L1 控制任务通过 control_phase 路由调用，不嵌入 RunCycle。
6. L2/Runtime 新增工具模块（如队列）需同步更新 `builder.params`。
7. L2 层不持有任何队列实例，队列存储由 L1 在 `foc_runtime_ctx_t` 中分配。

### 3. 验证

1. 构建必须 `0 error`，并且不得新增 warning。
2. 验证 include 边界、接口签名和跨层依赖（特别是 L2 不得包含 `L1_Orchestration/` 头文件）。
3. 涉及行为变化时，补充实例级硬件验证（命令链路、状态反馈、关键控制路径）。
4. 若会话上下文被压缩或切换，继续改动前需重读关键文件。

### 4. 文档同步

1. 结构、依赖、时序变化：更新 `docs/architecture.md`。
2. 流程或协作变化：更新 `copilot-instructions.md` 与 `.github/*.md`。
3. 版本基线与任务阶段变化：更新 `NEXT_MISSION.md` 与 `CHANGELOG.md`。
4. 协议命令、裁剪开关、默认值变化：更新 `docs/protocol-parameters-bilingual.md` 与实例协议文档。
5. 不新增"平行事实源"文档，优先更新已有主文档。

## P0 可维护性验收

### 必须满足

1. `main` 仅调用 `L1` 库入口与 `L5` 初始化入口。
2. `L1/L2/L3` 不直接依赖设备驱动头。
3. 宏裁剪链路一致：声明、定义、调用在同一条件编译语义下。
4. 关键约束可映射到可执行检查动作（检索/构建/日志）。
5. L2 层不持有队列实例，不包含 L1 头文件。

### 拒收条件

1. 同一结构事实在多个文档并行维护。
2. 为兼容旧结构新增跳转壳文档。
3. 关键路径出现无收益的重复限幅/归一化或绕路中转。
4. 引入降低可读性的冗余中间抽象。

## 提交治理

1. 每次完整修改周期后执行本地 `git commit`。
2. 默认不 `git push`，仅在用户明确要求时执行。
3. 若用户要求修改当前提交，优先 `git commit --amend`。

## 编译与调试经验

1. VS Code 任务输出不完整时，优先查看终端完整构建日志。
2. 手动调用 `unify_builder.exe` 可能需要设置 `DOTNET_ROLL_FORWARD=Major`。
3. 若出现 `Not found any source files`，优先检查 `builder.params` 的 `sourceList` 路径。
4. `get_errors` 可能残留过期诊断，最终以真实编译/链接结果为准。
5. 功能宏关闭后，务必同步收口受控函数的声明/定义/调用，避免"裁掉定义但未裁掉调用"。

## 质量门禁

1. no newly introduced warnings。
2. 代码与文档必须同次迭代同步。
3. 公共接口变更需评估实例适配影响。