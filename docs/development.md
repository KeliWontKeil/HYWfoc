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

1. 先读 `AI_INITIALIZATION.md` 与 `NEXT_MISSION.md`，确认范围和验收口径。
2. 明确任务属性：改代码 / 仅设计 / 暂缓。
3. 评估影响层级与回滚点。

### 2. 实施

1. 遵循分层约束。详细分层见 `docs/architecture.md`（L2 下分 Control/Protocol/Runtime，各块间不直接调用）。
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
2. 流程或协作变化：更新 `AI_INITIALIZATION.md` 与 `.github/*.md`。
3. 版本基线与任务阶段变化：更新 `NEXT_MISSION.md` 与 `CHANGELOG.md`。
4. 协议命令、裁剪开关、默认值变化：更新 `docs/protocol-parameters.md` 与实例协议文档。
5. 不新增"平行事实源"文档，优先更新已有主文档。

## P0 可维护性验收

### 必须满足

1. `main` 仅调用 `L1` 库入口与板级外设初始化函数。
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

### 构建命令

```batch
set DOTNET_ROLL_FORWARD=Major
"C:\Users\MSI-NB\.vscode\extensions\cl.eide-3.26.9\res\tools\win32\unify_builder\unify_builder.exe" --rebuild -p "examples\GD32F303_FOCExplore\software\build\GD32F30X_CL\builder.params"
```

- `--rebuild`：强制全量重建
- `-p`：指定 `builder.params` 路径

辅助脚本（自动定位最新 EIDE 扩展版本，避免硬编码版本路径）：

```powershell
.\tools\build_gd32f303.ps1
```

### 构建目标约束

- 编译器：ARM Compiler 5 (AC5)
- 语言标准：C99
- 优化等级：level-1（CL target）
- microLIB 启用，one-elf-section-per-function
- 0 error，不新增 warning
- ROM ≤ 256KB，RAM ≤ 96KB（GD32F303CC 实例限制）
- 跨平台移植预留：建议 ROM ≤ 64KB，RAM ≤ 16KB

### 常见错误与解决方案

| 错误 | 根因 | 解决方案 |
|------|------|----------|
| `Not found any source files` / 文件未编译 | `builder.params` 中 `sourceList` 路径错误或文件缺失 | 检查 `.c` 文件是否存在，路径基于 `rootDir` 的相对引用 |
| `L6218E: Undefined symbol`（链接错误）| 某 `.c` 文件未加入 `sourceList`，或函数名拼写差异 | 检查 `builder.params.sourceList` 是否包含该文件；检查声明/定义/调用三者一致 |
| `#20: identifier undefined` / 类型未定义 | 头文件依赖缺失或 `#include` 路径不对 | 检查 `builder.params.incDirs` 是否包含所需头文件目录 |
| `#147-D: declaration is incompatible` | 函数声明与定义参数不匹配 | 检查 `.h` 与 `.c` 的函数签名一致 |
| 条件编译宏裁掉函数定义但未裁掉调用 | 宏裁剪链路不同步 | 功能宏关闭后，必须同步收口所有引用点的声明/定义/调用 |

### 构建日志查看

- 完整实时日志：`unify_builder.exe` 的 stdout 输出
- 历史日志：`build/GD32F30X_CL/unify_builder.log`
- 仅当 VS Code 任务输出不完整时，优先查看终端完整构建日志

### 调试要点

1. 手动调用 `unify_builder.exe` 可能需要设置 `DOTNET_ROLL_FORWARD=Major`。
2. 若出现 `Not found any source files`，优先检查 `builder.params` 的 `sourceList` 是否与 `rootDir` 匹配。
3. `get_errors` 可能残留过期诊断，最终以真实编译/链接结果为准。
4. 功能宏关闭后，务必同步收口受控函数的声明/定义/调用，避免"裁掉定义但未裁掉调用"。

## 质量门禁

1. 不得新增 warning（0 error，不新增 warning）。
2. 代码与文档必须同次迭代同步。
3. 公共接口变更需评估实例适配影响。