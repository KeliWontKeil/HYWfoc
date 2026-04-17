# 架构与结构总览（唯一结构说明）

## 文档定位

本文件是仓库“结构与依赖”的唯一主文档（SSOT）。

- 结构、分层、依赖方向、时序与控制链路，以本文件为准。
- 本文件禁止写死控制频率，控制周期与调度槽均以配置宏和代码实现为准。

## 仓库结构（当前基线）

```text
FOC_VSCODE/
├── foc/                                         # 可复用核心库
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
│   └── port/                                    # 平台API空模板
├── examples/GD32F303_FOCExplore/               # 板级实例
│   ├── hardware/
│   └── software/
├── docs/                                        # 库级文档
└── .github/                                     # AI工作流与提示词
```

## 分层模型（代码与职责）

当前统一采用“LS + L1~L5”描述，旧分层术语不再作为并行口径维护。

| 层级 | 主要位置 | 职责 |
|---|---|---|
| `LS` 配置层 | `foc/include/LS_Config/foc_cfg_*.h` | 开关、默认值、编译期约束、符号定义 |
| `L1` 运行编排层 | `foc/src/L1_Orchestration/foc_app.c`、`foc/src/L1_Orchestration/control_scheduler.c` | 启动时序、任务调度、运行入口 |
| `L2` 逻辑功能层 | `foc/src/L2_Service/command_manager.c`、`foc/src/L2_Service/command_manager_dispatch.c`、`foc/src/L2_Service/command_manager_store.c`、`foc/src/L2_Service/command_manager_query.c`、`foc/src/L2_Service/command_manager_diag.c`、`foc/src/L2_Service/debug_stream.c`、`foc/src/L2_Service/protocol_service.c`、`foc/src/L2_Service/protocol_parser.c` | 协议运行时适配（多源收发、状态回执、故障统计）、参数存储/查询、诊断输出、调试输出 |
| `L3` 应用算法层 | 控制链：`foc_control_c01_entry.c`、`foc_control_c02_cfg_state.c`、`foc_control_c03_outer_loop.c`、`foc_control_c04_current_loop.c`、`foc_control_c05_actuation.c`、`foc_control_softswitch.c`、`foc_control_compensation.c`、`foc_control_init.c`；采样与调制：`sensor.c`、`svpwm.c`；协议纯处理：`protocol_core_parser.c`、`protocol_core_normalize.c`、`protocol_text_codec.c` | 承载可裁剪算法与纯处理内核；v1.3.5 已落地 `C01->C02->C03->C04->C05` 单向结构链（一个整体、多级文件、禁止反向依赖） |
| `L4-2` 平台接口桥 | `foc_platform_api.h`、实例 `foc_platform_api.c` | 将库调用桥接到具体外设实现 |
| `L4-1` 纯算法复用层 | `foc/include/L41_Math/*`、`foc/src/L41_Math/*` | 不含板级依赖、可跨平台复用 |
| `L5` 外设驱动层 | `examples/.../software/Utilities/` + `Firmware/` | 定时器、PWM、ADC、USART、I2C、芯片库 |

## 依赖约束（强制）

1. `L1/L2/L3` 访问硬件仅可通过 `foc/include/L42_PAL/foc_platform_api.h`。
2. `L1/L2/L3` 头文件不得暴露 `gd32f30x_*` 设备头。
3. `L5` 不得反向依赖 `foc/src/` 业务逻辑。
4. 可配置常量必须汇聚到 `foc_cfg_*.h`，禁止散落在 `.c`。

## 可维护性约束（强制）

1. 模块说明必须对应真实文件位置，禁止抽象名词堆叠但无法落到代码。
2. 同一职责只允许一个主入口文档，禁止多文档重复叙述同一结构事实。
3. 涉及层级、接口、时序的描述必须可映射到检查动作（例如：头文件包含检查、API 调用点检查、构建验证）。
4. 若代码与文档冲突，以代码为准并在同次迭代修正文档。

## P1 清点结果（v1.3.4）

### 控制链路与协议链路边界清单（当前状态 + 目标状态）

| 链路边界 | 当前状态（已实现） | 目标状态（本阶段） | 文件级落点 |
|---|---|---|---|
| 协议帧采集与源仲裁 | L2 保留多源 ready/read、状态回执；L3 已承载帧提取、语法解析、命令归一化与文本格式构建 | 维持“L2 运行态适配 + L3 纯处理内核”双段式分层，并避免纯处理逻辑回流到 L2 | `foc/src/L2_Service/protocol_service.c`、`foc/src/L2_Service/protocol_parser.c`、`foc/src/L3_Algorithm/protocol_core_parser.c`、`foc/src/L3_Algorithm/protocol_core_normalize.c`、`foc/src/L3_Algorithm/protocol_text_codec.c` |
| 参数/状态/系统命令执行 | `command_manager` 负责运行编排/分发，存储查询下沉到 store/query 子模块 | 与协议语法解析保持解耦，只消费结构化命令结果 | `foc/src/L2_Service/command_manager.c`、`foc/src/L2_Service/command_manager_store.c`、`foc/src/L2_Service/command_manager_query.c` |
| 应用运行编排 | `foc_app` 负责调度触发、通信处理步进、故障降级与控制入口 | 不承载协议字段解析与参数存储职责 | `foc/src/L1_Orchestration/foc_app.c` |
| 初始化控制链路 | `foc_control_init` 负责电角锁定、机械零位采样、方向/极对数估计 | 严格保持 direct 占空比下发路径，不混入运行态插值更新 | `foc/src/L3_Algorithm/foc_control_init.c` |
| 运行态控制链路 | `foc_control_c01~c05` + `svpwm` + PWM Update ISR 完成目标占空比更新与插值下发 | 保持 runtime 路径独立，不回退到初始化态直通策略；v1.3.5 已完成 `C01->C02->C03->C04->C05` 单向依赖链重排 | `foc/src/L3_Algorithm/foc_control_c01_entry.c`、`foc/src/L3_Algorithm/foc_control_c02_cfg_state.c`、`foc/src/L3_Algorithm/foc_control_c03_outer_loop.c`、`foc/src/L3_Algorithm/foc_control_c04_current_loop.c`、`foc/src/L3_Algorithm/foc_control_c05_actuation.c`、`foc/src/L3_Algorithm/svpwm.c`、`foc/src/L1_Orchestration/foc_app.c` |

### 配置宏归属清单

| 配置头文件 | 归属职责 | 典型内容 |
|---|---|---|
| `foc_cfg_symbol_defs.h` | 基础符号与跨模块枚举/字面量约束 | 协议命令字、状态码、通用常量符号 |
| `foc_cfg_feature_switches.h` | 功能裁剪与算法裁剪开关 | 协议参数通道裁剪、诊断输出开关、算法特性开关 |
| `foc_cfg_init_values.h` | 默认初始化值与标定/控制初值 | 调度频率、PWM频率、PID默认值、标定步进参数 |
| `foc_cfg_compile_limits.h` | 编译期约束与跨宏一致性检查 | 开关合法性校验、时序分频约束、默认值合法性约束 |

### 平台 API 最小集清单

| 分类 | 保留（当前最小集） | 候选移除（后续评审） | 本轮必须新增 |
|---|---|---|---|
| 运行与时基 | `RuntimeInit`、`ControlTickSourceInit`、`SetControlTickCallback`、`StartControlTickSource`、`SetControlRuntimeInterrupts`、`SetPwmUpdateCallback` | 无 | 无 |
| 采样与执行 | `SensorInputInit`、`SetSensorSampleOffsetPercent`、`ReadPhaseCurrentAB`、`ReadPhaseCurrentABFast`、`ReadMechanicalAngleRad`、`PWMInit`、`PWMStart`、`PWMSetDutyCycleTripleFloat` | 无 | 无 |
| 通信与诊断 | `CommInit`、`CommSource{1..4}_IsFrameReady/ReadFrame`、`WriteDebugText`、`WriteStatusByte`、`EnableCycleCounter`、`ReadCycleCounter` | `SetHeartbeatIndicator`（当前无调用点） | 无 |
| 保护钩子 | `UndervoltageProtect` | 无 | 无 |

### 关键时序点与影响分析

| 时序点 | 关键配置宏 | 执行位置 | 影响分析 |
|---|---|---|---|
| 控制调度周期 | `FOC_SCHEDULER_TICK_HZ`、`FOC_SCHEDULER_*_DIVIDER` | `ControlScheduler_RunTick` | 影响外环、服务任务与监测任务节拍；分频必须整除，避免任务抖动与漏触发 |
| PWM 更新中断 | `FOC_PWM_FREQ_KHZ`、`FOC_CURRENT_LOOP_ISR_FREQ`、`FOC_CURRENT_LOOP_ISR_DIVIDER` | `FOC_App_OnPwmUpdateISR` + `SVPWM_InterpolationISR` | 影响运行态占空比插值与快速电流环执行频率；采用整除约束保证固定 N 分频 |
| 采样触发相位 | `FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT` | `Sensor_ADCSampleTimeOffset` -> 平台 `Timer3_SetSampleOffsetPercent` | 影响 PWM 周期内采样时刻；偏移越接近边沿越可能放大采样噪声与死区耦合误差 |

## P2 结构重排落地记录（v1.3.4）

1. 协议解析与运行状态管理在 v1.3.4 基线保持双模块边界；v1.3.5 起按“L2 运行态适配 + L3 纯处理内核”推进分层收敛。
2. `command_manager` 内部分拆为 dispatch/diag 子模块：命令分发与诊断文本职责物理解耦。
3. `foc_control` 已在 v1.3.5 收敛为 `C01~C05` 编号分层单向链，softswitch/compensation 作为配套策略模块复用，保持“同一算法整体 + 可读依赖层级”。
4. 纯数学与 LUT 头/源迁移到 `foc/include/L41_Math` 与 `foc/src/L41_Math`，从算法目录职责中解耦。
5. 初始化标定链路中的三角函数调用统一到 LUT 路径（`FOC_MathLut_Sin`、`FOC_MathLut_Atan2`），保持与运行态数学路径一致。
6. 控制算法保持 init/runtime 严格分离：初始化链走 direct 占空比路径，运行链走 runtime 目标 + ISR 插值路径。
7. 平台中断生命周期收敛到统一开关入口：运行态控制相关中断通过 `FOC_Platform_SetControlRuntimeInterrupts(enable)` 管理，`FOC_App_Init` 显式保持初始关闭，`FOC_App_Start` 统一开启。
8. L4-2 到 L5 依赖保持单向：实例 `foc_platform_api.c` 仅桥接 `Utilities/*` 驱动，不回调 L1/L2/L3 业务实现。

## v1.3.5 结构准备（M6~M8）

1. L2 按“运行编排 -> 分发执行 -> 参数状态存储/查询 -> 诊断输出”收敛为层内单向调用链，不再维持中心化超大实现文件。
2. L3 控制算法按功能链编号命名推进：`foc_control_c01_*`（入口编排）-> `c02_*`（配置与状态）-> `c03_*`（外环）-> `c04_*`（快环）-> `c05_*`（执行与后处理）。
3. 去内联约束分层处理：L2 允许少量受控 inline（模块内私有且可检索）；L3 业务逻辑完全去内联（除 `L41_Math` 既有数学 inline）。
4. `sensor.c` 与 `svpwm.c` 本轮保持职责边界，不做拆分/重命名并发改造。
5. 若出现单函数、单调用者的过细算法文件，允许按单向层级并回上一级实现文件，避免文件粒度碎片化。

## 控制时序（抽象）

```text
控制节拍源
├── 调度器回调（慢速任务：协议、状态、监测）
└── 控制主循环入口

PWM更新中断源（高速）
├── 运行态SVPWM插值
└── （可选）电流环快速路径

采样触发源
└── 与PWM对齐的电流/角度采样触发
```

说明：具体定时器映射、通道管脚和中断向量归实例文档维护。

## 控制链路（初始化与运行严格分离）

### 初始化链路（直接输出链）

```text
初始化状态机
-> 锁定电角0
-> 采样机械零位
-> 估计方向与极对数
-> 直接SVPWM占空比下发（无运行态插值）
```

### 运行链路（插值输出链）

```text
传感器采样
-> 坐标变换与控制计算
-> SVPWM运行态目标更新
-> PWM更新ISR执行插值下发
```

## 启停与中断生命周期

1. 初始化阶段允许外设启动，但延后运行态控制中断使能。
2. 应用进入运行态后，再统一调用平台接口开启运行时中断。
3. 运行期中断开关通过统一平台 API 管理，避免分散控制。

## 迁移与适配入口

1. 复制并实现 `foc/port/foc_platform_api_empty.c` 中空接口。
2. 在实例工程 `Application/Source/foc_platform_api.c` 完成桥接。
3. 保持接口签名不变，减少库升级迁移成本。

## 维护规则

1. 结构/依赖变化必须同步更新本文件。
2. 本文件变化后，需同步检查：
    - `docs/README.md` 索引项
    - `.github/DOCUMENTATION_STRUCTURE.md` 文档归属说明
3. 禁止通过新增“历史跳转文档”维持兼容结构；结构重复内容应直接合并或删除。
