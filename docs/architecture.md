# 架构与结构总览（唯一结构说明）

## 文档定位

本文件是仓库结构、分层、依赖方向和控制链路的唯一事实源（SSOT）。

约束：

1. 不写死具体控制频率，频率以 `foc/include/LS_Config/foc_cfg_*.h` 为准。
2. 架构事实必须可映射到真实文件。
3. 代码与文档冲突时，以代码为准并同次修正文档。

## 仓库结构（当前）

```text
FOC_VSCODE/
├── foc/
│   ├── include/
│   │   ├── LS_Config/
│   │   ├── L1_Orchestration/
│   │   ├── L2_Service/
│   │   ├── L3_Algorithm/
│   │   ├── L41_Math/
│   │   └── L42_PAL/
│   ├── src/
│   │   ├── L1_Orchestration/
│   │   ├── L2_Service/
│   │   ├── L3_Algorithm/
│   │   └── L41_Math/
│   └── port/
├── examples/GD32F303_FOCExplore/
│   ├── hardware/
│   └── software/
├── docs/
└── .github/
```

## 分层模型（代码与职责）

| 层级 | 主要位置 | 职责 |
|---|---|---|
| `LS` 配置层 | `foc/include/LS_Config/foc_cfg_*.h` | 符号定义、功能开关、默认值、编译期约束 |
| `L1` 运行编排层 | `foc/src/L1_Orchestration/foc_app.c`、`foc/src/L1_Orchestration/control_scheduler.c` | 启动流程、任务调度、控制入口 |
| `L2` 服务与协议运行时层 | `runtime_c1_entry.c`、`runtime_c2_frame_source.c`、`runtime_c3_runtime_fsm.c`、`runtime_c4_runtime_core.c`、`runtime_c5_output_adapter.c`、`debug_stream.c`、`motor_control_service.c` | 协议帧接入、状态机、命令执行与存储、输出适配、控制服务桥接 |
| `L3` 应用算法层 | `foc_control_c11_entry.c`、`foc_control_c12_init.c`、`foc_control_c13_cfg_state.c`、`foc_control_c21_outer_loop.c`、`foc_control_c22_current_loop.c`、`foc_control_c23_motor_param_learn.c`、`foc_control_c24_compensation.c`、`foc_control_c31_actuation.c`、`sensor.c`、`svpwm.c`、`protocol_core.c` | 控制算法（入口、初始化标定、配置状态管理、外环、电流内环、参数学习、齿槽补偿、SVPWM 执行）、采样处理、协议纯处理内核 |
| `L41` 数学复用层 | `foc/include/L41_Math/*`、`foc/src/L41_Math/*` | 与平台无关的数学与 LUT |
| `L42` 平台抽象层 | `foc/include/L42_PAL/foc_platform_api.h` + 实例 `foc_platform_api.c` | 库到板级驱动的唯一桥接接口 |
| `L5` 板级驱动层 | `examples/.../software/Utilities/*`、`Firmware/*` | 外设驱动与芯片库实现 |

## 当前主链路

### L2 协议运行时链（生产主链）

固定链路：`C1 -> C2 -> C3 -> C4 -> C5`

1. `C1`：运行编排入口与步进预算。
2. `C2`：多源帧读取、协议解析接入、快照透传。
3. `C3`：系统状态机、故障与系统命令流程。
4. `C4`：参数/状态读写、运行存储、快照构建。
5. `C5`：状态字与文本输出适配。

说明：仓库中存在 `runtime_c32_command_router.[ch]` 并行实现文件，当前未纳入实例构建清单，不作为生产主链。

### L3 控制运行链（已落地）

1. 初始化链：`C12` + `C23` + `C11` 初始化桥接。
2. 运行外环：`C11 -> C21`（速度/位置外环）。
3. 运行内环：`C11 -> C22 -> C31`（电流环与执行输出）。
4. 配置与补偿：`C13`（运行态配置）、`C24`（齿槽补偿，可裁剪）。

## 宏裁剪口径（与代码一致）

### 算法特性开关（LS）

定义位置：`foc/include/LS_Config/foc_cfg_feature_switches.h`

1. 电流环与软切换特性：`FOC_CURRENT_LOOP_PID_ENABLE`、`FOC_CURRENT_SOFT_SWITCH_ENABLE`
2. 齿槽补偿特性：`FOC_COGGING_COMP_ENABLE`（补偿使能）、`FOC_COGGING_CALIB_ENABLE`（运行时手动标定使能）、`FOC_COGGING_INIT_LEARN_ENABLE`（旧版初始化自动标定，默认关闭）、`FOC_COGGING_DEBUG_DUMP_ENABLE`
3. 采样滤波特性：`FOC_SENSOR_KALMAN_*`、`FOC_SENSOR_ANGLE_LPF_ENABLE`

### 协议裁剪开关（L2/L3 协同）

1. 定义位置：`foc/include/LS_Config/foc_cfg_feature_switches.h`
2. 固定最小集（不可裁剪）：`P:A/R/S/D`、`S:M`、`Y:R/C`
3. 可选组：`FOC_PROTOCOL_ENABLE_*`
4. 当前已接入的齿槽协议链路：状态子命令 `S:G`（`COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE`）；系统子命令 `Y:G`（`COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_CALIB`，触发运行时标定）、`Y:D`（`COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_DUMP`，串口输出补偿表）、`Y:T`（`COMMAND_MANAGER_SYSTEM_SUBCMD_COGGING_EXPORT`，串口输出可嵌入代码的 C 数组格式补偿表）

说明：符号头中预留了齿槽参数子命令符号（`J/K/N`），当前运行时实现尚未接入对应参数读写链路。

### 编译期约束与提示策略

定义位置：`foc/include/LS_Config/foc_cfg_compile_limits.h`

1. 开关合法性与范围约束：使用 `#error` 阻断非法配置。
2. 跨开关冲突提示：使用编译提示（ARMCC5 下通过 `#warning` 分支）。
3. 典型硬约束：调度分频整除、PWM/ISR 频率整除、初始化标定关闭时默认方向/极对必须定义。

## 依赖方向约束（强制）

1. `L1/L2/L3` 访问硬件只能通过 `L42` 平台 API。
2. 公共头文件不得暴露 `gd32f30x_*`。
3. `L5` 不得反向依赖 `foc/src/*` 业务逻辑。
4. 配置常量必须收敛在 `foc_cfg_*.h`，禁止在业务 `.c` 中散落默认值。
5. L2 主链保持单向，禁止跨层逆向调用。

## 控制时序（抽象）

```text
控制节拍源
├── 调度器回调（服务任务、监测任务、控制主循环）
└── 控制主循环入口

PWM 更新中断源（高速路径）
├── SVPWM 插值
└── （可选）电流环快路径

采样触发源
└── 与 PWM 对齐的电流/角度采样
```

具体定时器映射与引脚归属由实例文档维护。

## 维护规则

1. 结构/依赖变化必须同步更新本文件。
2. 更新本文件后，同次检查：`docs/README.md`、`.github/DOCUMENTATION_STRUCTURE.md`、`docs/development.md`。
3. 禁止新增并行结构文档作为“兼容跳转页”。