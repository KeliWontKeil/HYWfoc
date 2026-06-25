## Brief overview

HYWfoc（何易位FOC）是一个磁场定向控制（FOC）项目，采用"核心库 + 实例工程"组织方式。核心库 `foc/` 是平台无关的，实例工程在 `examples/<instance>/` 中。主要实例工程基于 GD32F303CC 。

## Project structure

- `foc/` — 平台无关可复用控制库
  - `foc/include/` — 头文件（按功能分层：`LS_Config/`、`L1_Orchestration/`、`L2/{Control,Protocol, Runtime}/`、`L3/`）
  - `foc/src/` — 源文件（同上层级结构）
- `examples/<instance>/` — 板级工程与驱动实现
  - `software/Application/` — 实例应用层
  - `software/Utilities/` — 外设驱动
  - `hardware/` — 管脚等硬件文档
- `docs/` — 库级文档
  - `docs/architecture.md` — 唯一结构说明书（SSOT）
  - `docs/development.md` — 开发流程
  - `docs/protocol-parameters-bilingual.md` — 协议参数中英对照
- `docs/engineering/dev-guidelines/rules/` — 仓库规则文件（cn/en 双语）
- `.github/` — 工作流治理

## Layer architecture

依赖方向严格单向：`LS` → `L1` → `L2` → `L3` → `L5`

| 层级 | 位置 | 职责 | 实例化职责 |
|---|---|---|---|
| `LS` | `foc/include/LS_Config/` | 符号定义、功能开关、默认值、编译期约束、类型定义、数据表 | 无实例（纯宏与类型） |
| `L1` | `foc/src/L1_Orchestration/` | 启动流程、主循环编排、实例化核心数据结构 | **持有所有运行时实例**（foc_motor_t、foc_system_t、队列缓冲区） |
| `L2/Control` | `foc/src/L2/Control/foc_ctrl_*.c` | 控制算法（执行器/配置/初始化/外环/电流环/参数学习/补偿/执行输出） | 不持实例，操作传入的 `foc_motor_t` 指针 |
| `L2/Protocol` | `foc/src/L2/Protocol/foc_protocol_*.c` | **单帧处理**：解析一帧→修改 motor→返回结果。不读帧、不入队、不轮询 | 不持实例，指针由 L1 传入 |
| `L2/Runtime` | `foc/src/L2/Runtime/` | 调度器、环形队列（**纯方法模块**）、调试流生成器（逐行生成，不写队列） | 队列类型可实例化，但实例在 L1 分配 |
| `L3` | `foc/src/L3/` | 数学变换、LUT、平台抽象API、传感器采样、SVPWM | 无实例（纯函数） |
| `L5` | `examples/.../software/Utilities/` | 外设驱动与芯片库实现 | 芯片固有实例 |

**关键约束**：
- `L2` 各块访问硬件只能走 `L3` 平台 API（`foc_pal.h`）
- 公共头文件不得暴露 `gd32f30x_*` 设备头
- `L5` 不得反向依赖 `foc/src/*`
- 配置常量必须收敛在 `foc_cfg_*.h`，禁止在业务 `.c` 中散落默认值
- **L2 任何模块不得包含 `L1_Orchestration/` 头文件**
- **L2 任何模块不得持有队列实例**——队列存储由 L1 在 `foc_runtime_ctx_t` 中分配，L2 通过指针参数操作

## L2 Control module naming

L2/Control 控制链模块统一按 `foc_ctrl_XX_name.c/.h` 命名：

- `C11` — 算法入口（外环/内环/开环/补偿入口）
- `C12` — 初始化与标定
- `C13` — 配置状态管理（软切换、齿槽补偿、PID 初始化、fine-tuning setter）
- `C21` — 速度/位置外环
- `C22` — 电流内环
- `C23` — 电机参数学习
- `C24` — 齿槽补偿
- `C31` — 执行输出（SVPWM 驱动）

**注**：C13 是从旧 C25 提升而来，因为它无 L3 内部模块依赖，仅操作结构体字段和宏配置。

## Configuration macro & type management

LS_Config 文件分为三大类：

**A. 配置宏文件（业务代码通过 `foc_config.h` 唯一入口）**
- `foc_symbol_defs.h` — 符号与兼容定义
- `foc_cfg_feature_switches.h` — 功能裁剪开关
- `foc_cfg_init_values.h` — 默认值
- `foc_compile_limits.h` — 编译期约束检查（不单独 `#include`，由 `foc_config.h` 按序包含）

**B. 类型定义文件（通过 `foc_shared_types.h` 聚合或直接引用）**
- `foc_math_types.h` — kalman_filter_t, foc_pid_t
- `foc_motor_types.h` — motor 聚合结构体
- `foc_scheduler_types.h` — FOC_TaskRate_t 调度枚举
- `foc_protocol_types.h` — protocol_command_t, 解析器枚举
- `foc_runtime_types.h` — runtime_step_signal_t, init_check / fault 码
- `foc_snapshot_types.h` — 快照/视图结构体

**C. 数据表文件**
- `foc_cogging_table.h` — 静态齿槽补偿默认表

**约束**：
- 宏头文件之间不交叉依赖（不互相 `#include`）
- 统一入口：`foc_config.h`（业务代码只包含此文件） + `foc_shared_types.h`（类型聚合）
- 已无 `cfg` 前缀的非设置项文件（`foc_symbol_defs.h` 已去 `cfg`）
- 各 `.c` 文件中枚举/结构体定义已全部收敛至 LS 层对应类型文件


## Coding conventions

- **大括号风格**：Allman 风格（大括号单独占一行）
- **函数命名**：`Module_FunctionName`（如 `FOC_MotorInit`、`SVPWM_Update`）
- **宏命名**：`UPPERCASE_WITH_UNDERSCORES`
- **类型命名**：`xxx_t` 后缀
- **条件编译**：宏裁剪链路必须声明/定义/调用三者一致，关闭宏后必须同步收口所有引用点
- **指针校验**：函数入口检查指针是否为 NULL，返回 0 或直接 return
- **ISR 路径**：禁止阻塞操作，避免浮点运算；优先转发到模块处理函数

## 运行时主循环结构

L1 主循环（`FOC_App_Loop`）编排三个独立任务段，顺序无关：

1. **Monitor 段**：调试流生成器逐行输出 → 入 TX 队列
2. **Service 段**：RX 队列出队 → 协议单帧处理（`FOC_Protocol_ProcessSingle`）→ 编排结果（状态码直写、摘要入 TX 队列、配置脏检查）
3. **TX 消费段**：TX 队列出队 → 平台发送

### 双输出路径

- **快路径（直写）**：状态码、参数行等短数据在协议内部直接调用 L3 平台 API，ISR 或协议函数中执行
- **慢路径（队列）**：摘要、遥测、示波器帧等大数据入 TX 队列，主循环统一出队发送

无固定管线链，L2 各块间通过 L1 统一传递数据，不直接跨块调用。

## Version control practices

- 语义化版本 `MAJOR.MINOR.PATCH`（当前 `v1.7.5`）
- 默认在 `main` 直接开发，不创建新分支
- 每次完整修改后仅做本地 `git commit`
- 默认不 `git push`，仅在用户明确要求时执行
- 若用户要求修订当前提交，优先 `git commit --amend`
- 推送门禁：仅 `MINOR` 或 `MAJOR` 变更时允许推送

## Documentation synchronization

代码与文档必须同一次迭代同步。结构/依赖变化时必须更新：
1. `docs/architecture.md` — 唯一结构说明
2. `docs/development.md` — 开发流程
3. `docs/README.md` — 版本基线
4. `copilot-instructions.md` — 版本基线
5. `CHANGELOG.md` — 变更记录
6. `NEXT_MISSION.md` — 下一阶段目标
7. `docs/engineering/dev-guidelines/rules/*` — 规则文件（如分层描述变化）

**禁止**：新增"平行事实源"文档，优先更新已有主文档。

## Communication protocol

帧格式：`a<driver_id><cmd><subcmd><param>b`

- 命令通道：`P`（参数通道）、`S`（状态通道）、`Y`（系统通道）
- 默认本地 ID：`0x61`（`'a'`）
- 返回状态码：`O`(成功)、`E`(格式错误)、`P`(参数无效)、`I`(命令无效)、`T`(超时)
- 固定最小集（不可裁剪）：`P:A/R/S/D`、`S:M`、`Y:R/C`
- 协议裁剪开关：`FOC_PROTOCOL_ENABLE_*` 系列宏

## Current mission

如果有，下一目标在 `NEXT_MISSION.md` 中定义，否则根据用户prompt开发。

## Build constraints

- 编译后端：ARM Compiler 5 (AC5)
- 目标：0 error，不新增 warning
- 主要实例中ROM ≤ 256KB，RAM ≤ 96KB，实际应考虑不同平台的可移植性，建议ROM ≤ 64KB，RAM ≤ 16KB
- 实例构建入口：`examples/<instance>/software/` 下的 EIDE 工程
- 根工作区 `Project.code-workspace` 仅用于管理与文档治理

## Build experience

### 构建命令

```batch
set DOTNET_ROLL_FORWARD=Major
"C:\Users\MSI-NB\.vscode\extensions\cl.eide-3.26.9\res\tools\win32\unify_builder\unify_builder.exe" --rebuild -p "examples\GD32F303_FOCExplore\software\build\GD32F30X_CL\builder.params"
```

- `--rebuild`：强制全量重建
- `-p`：指定 `builder.params` 路径

### 常见错误与解决方案

| 错误 | 根因 | 解决方案 |
|------|------|----------|
| `Not found any source files` / 文件未编译 | `builder.params` 中 `sourceList` 路径错误或文件缺失 | 检查 `.c` 文件是否存在，路径基于 `rootDir` 的相对引用 |
| `L6218E: Undefined symbol` (链接错误) | 某 `.c` 文件未加入 `sourceList`，或函数名拼写差异 | 检查 `builder.params.sourceList` 是否包含该文件；检查声明/定义/调用三者一致 |
| `#20: identifier undefined` / 类型未定义 | 头文件依赖缺失或 `#include` 路径不对 | 检查 `builder.params.incDirs` 是否包含所需头文件目录 |
| `#77-D: has no storage class` / `#65: expected ";"` | 代码在函数体外部（通常是缺少 `{` 导致函数体提前闭合） | 检查该文件最近的 `if/for/while` 是否缺少左大括号 |
| `#147-D: declaration is incompatible` | 函数声明与定义参数不匹配 | 检查 `.h` 与 `.c` 的函数签名一致 |
| 条件编译宏裁掉函数定义但未裁掉调用 | 宏裁剪链路不同步 | 功能宏关闭后，必须同步收口所有引用点的声明/定义/调用 |
| `#1-D: last line of file ends without a newline` | 文件末尾缺少换行符 | 在文件末尾添加一个空行 |
| `not found any source files` | `sourceList` 路径与 `rootDir` 不匹配 | 优先检查 `sourceList` 路径是否正确 |

### 构建日志查看

- 完整实时日志：`unify_builder.exe` 的 stdout 输出
- 历史日志：`build/GD32F30X_CL/unify_builder.log`
- 仅当 VS Code 任务输出不完整时，应优先查看终端完整构建日志

### 注意
- 如果你是deepseek,思考过程请使用中文
- `get_errors` 可能残留过期诊断，最终以真实编译/链接结果为准
- 禁止写过多注释，在代码具有自解释性的情况下不写注释，注释请使用一句话注释（如果特意要求，再写详细注释）
- 如果你认为prompt有什么地方说不通，请不要使用自认为的答案然后继续修改，而是停下向我确认。
- 修改代码后请勿提交git，除非要求。未经审核的关于代码的git提交是不可容忍的。
- 计划过程中，请考虑和分析：调用链影响、数据流组织方式、算法调用结构等底层根因，不要仅处理现象。

### 项目规则/架构约束
- 本处用于记载项目的架构约束/编码规范，或是调试过程中形成的经验型约束
#### 协议裁剪宏不得影响控制算法行为
- `FOC_PROTOCOL_ENABLE_*` 系列宏仅控制协议命令的可见性与参数读写通道，不得用于保护控制算法中的逻辑分支。
- 控制算法行为应仅由 `FOC_*_ENABLE` 系列宏（如 `FOC_CURRENT_SOFT_SWITCH_ENABLE`）裁剪。
- 协议裁剪宏若需影响算法默认值，应在 `foc_cfg_init_values.h` 中通过默认值间接实现，而非在 `motor_control_service.c` 或 `runtime_c4_runtime_core.c` 中用 `#if` 保护控制路径。
