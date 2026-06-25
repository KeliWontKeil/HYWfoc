# 更新后执行计划（基于评估收敛版）

> 本文件记录了宏定义/结构体迁移和协议链路重组的评估结论与执行计划。
> 评估时间：2026-06-24

---

## 评估结论

### 1. LS 层保留内容（维持现状）

| 文件 | 评估结论 |
|------|----------|
| `foc_config.h` | ✅ 统一入口，保留 |
| `foc_shared_types.h` | 已不存在（类型已收敛到各 L2 子类型文件） |
| `foc_symbol_defs.h` | ✅ 符号定义、状态码、协议命令字符，保留 |
| `foc_cfg_feature_switches.h` | ✅ 功能裁剪开关，保留 |
| `foc_cfg_init_values.h` | ✅ 默认值，保留。**新增**队列配置宏 |
| `foc_compile_limits.h` | ✅ 编译期约束，保留 |
| `foc_cogging_table.h` | ✅ 静态齿槽表，保留 |

### 2. 结构体重组（放弃无收益项）

| 原计划项 | 结论 | 理由 |
|----------|------|------|
| 搬迁 `foc_ctrl_types.h` 到 L2/Control/ | ❌ 放弃 | L1/L2/协议/运行时/L3 均依赖此文件，搬迁引入非必要路径耦合 |
| 统一控制句柄 | ❌ 放弃 | 当前协议命令直接写 `motor->xxx` 字段，引入句柄无收益 |
| 快照与控制解耦 | ❌ 放弃 | `foc_snapshot_types.h` 仅含遥测策略，与控制算法无关 |
| 结构体按功能字段分组 | ❌ 放弃 | 散落浮点字段保持直接访问 |

### 3. 输出管线（放弃专用快通道）

| 原计划项 | 结论 | 理由 |
|----------|------|------|
| 快速输出通道 | ❌ 放弃 | 单 UART 同步模型下双通道无实质差异 |
| 队列输出通道 | ✅ 保留 | 示波器/语义遥测/运行时摘要走队列 |
| 直写通道 | ✅ 保留 | 回报/状态字节/错误诊断/初始化日志直写 |

统一策略：
- **直写通道** (`WriteDirect`)：立即发送。用于回报帧、状态字节、错误诊断、初始化日志、齿槽 Dump/Export。
- **队列通道** (`WriteQueue`)：入队等待主循环消费。用于示波器帧、语义遥测、`Y:R` 运行时摘要。
- **状态字节** (`WriteStatus`)：独立通道，直接写入物理 UART。

### 4. 输入管线（维持现状）

当前模式：`主循环轮询读取帧 → 完整解析 → 命令执行`
- 不引入输入队列：解析已在主循环执行，不会阻塞 ISR。
- ISR 职责限制为 DMA 接收和帧完成标志设置。

---

## 已完成工作项

### ✅ P3 — 队列配置 LS 化（2026-06-24 完成）

**修改文件：**
- `foc/include/LS_Config/foc_cfg_init_values.h`：新增 `FOC_OUTPUT_QUEUE_DEPTH`(8)、`FOC_OUTPUT_FRAME_MAX_LEN`(96)、`FOC_OUTPUT_MAX_PER_CYCLE`(4)
- `foc/include/L1_Orchestration/foc_system_types.h`：移除本地 `#ifndef` 默认值，添加 `#include "LS_Config/foc_config.h"`
- `foc/include/L1_Orchestration/foc_output_mgr.h`：移除 `FOC_OUTPUT_MAX_PER_CYCLE` 本地定义，添加 `#include "LS_Config/foc_config.h"`

**收益：** 队列配置默认值收敛到 LS 层，用户可在构建前通过 `#define FOC_OUTPUT_QUEUE_DEPTH 16` 覆盖

### ✅ P1 — 输出管线接入（代码审查确认已完成）

通过审查确认以下输出路径已正确使用输出管理器：
- `foc_debug_stream.c`：示波器帧 → `FOC_OutputMgr_WriteQueue` ✅
- `foc_debug_stream.c`：语义遥测 → `FOC_OutputMgr_WriteQueue` ✅  
- `foc_protocol_handler.c`：`Y:R` 运行时摘要 → `FOC_OutputMgr_WriteQueue` ✅
- 其他路径（回报、诊断、日志）→ `WriteDirect` ✅

---

## 待确认工作项

### ⏸ P2 — 协议 static 消除

- `foc_protocol_handler.c` 的 `g_sys_cfg_ptr` 通过 `FOC_Protocol_Init` 注入，是合理的 L1→L2 依赖注入模式
- 当前项目只支持单电机实例，此约束不构成实际阻塞
- **建议搁置**，除非多实例需求明确

---

## 版本基线

- 当前版本：**v1.8.0**
- 本迭代目标：宏定义/结构体迁移和协议链路重组的评估与收敛执行
- 迭代后版本：**v1.8.1**