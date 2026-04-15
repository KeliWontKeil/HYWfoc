# GD32F303_FOCExplore 示例工程

本目录是 HYWfoc（何易位FOC）的 GD32F303 平台参考实例，面向“拿到工程后如何构建、烧录、验证”的用户文档入口。

如果你需要了解库本身的架构与通用规则，请先阅读仓库根文档 [../../README.md](../../README.md) 与 [../../docs/README.md](../../docs/README.md)。

## 1. 适用平台与前提

### 硬件前提
- GD32F303CC 微控制器
- ST-LINK 调试器（SWD）
- 电流采样前端
- AS5600 磁编码器
- 兼容三相驱动级与电机

### 开发环境前提
- Keil uVision 5（ARM Compiler 5 / AC5）或VS Code + EIDE 扩展
- ST-LINK 驱动与下载工具链

## 2. 目录职责

- [hardware/](hardware/)：板级文档、接线与管脚速查。
- [software/](software/)：可直接打开的实例工程包（构建/烧录/调试入口）。
- [DEVELOPMENT.md](DEVELOPMENT.md)：实例构建与调试详细说明。
- [PROTOCOL_ADAPTATION.md](PROTOCOL_ADAPTATION.md)：实例通信通道映射与快速验证命令。

## 3. 首次打开工程

你可以任选一种方式：

### 方式 A：VS Code + EIDE
1. 打开 [software/Project.code-workspace](software/Project.code-workspace)。
2. 执行 `build` 或 `eide.project.build`。
3. 确认无新增告警。
4. 执行 `build and flash` 或 `eide.project.buildAndFlash`。

### 方式 B：Keil
1. 打开 [software/Project.uvprojx](software/Project.uvprojx)。
2. 选择目标 `GD32F30X_CL`。
3. 编译并下载到板卡。

构建产物默认位于 [software/build/GD32F30X_CL/](software/build/GD32F30X_CL/)（典型文件：`Project.axf`、`Project.hex`）。

## 4. 上电后快速验证

1. 连接串口并确认通信参数（见 [PROTOCOL_ADAPTATION.md](PROTOCOL_ADAPTATION.md)）。
2. 默认本机 driver_id 为 `0x61`（`'a'`），可先用 `aaPXb` 验证参数读回。
3. 继续执行 `aaPP3.0b`、`aaSM1b`、`aaYRb`、`aaYCb` 等快速命令，确认状态输出和故障清除路径。

说明：本页只给最小验证路径，具体通道绑定与命令语义以 [PROTOCOL_ADAPTATION.md](PROTOCOL_ADAPTATION.md) 与 [../../docs/protocol-parameters-bilingual.md](../../docs/protocol-parameters-bilingual.md) 为准。

## 5. 常见问题（最小排查）

- 构建失败：先确认你打开的是实例工作区而非根工作区。
- 烧录失败：检查 ST-LINK 连接、电源与 SWD 接线。
- 串口无输出：核对通道与参数，参考 [PROTOCOL_ADAPTATION.md](PROTOCOL_ADAPTATION.md)。
- 电机无响应：先完成命令链路验证，再排查硬件连接，参考 [hardware/hardware.md](hardware/hardware.md)。

## 6. 进一步阅读

- 实例构建/调试细节： [DEVELOPMENT.md](DEVELOPMENT.md)
- 实例协议映射与验证： [PROTOCOL_ADAPTATION.md](PROTOCOL_ADAPTATION.md)
- 硬件管脚速查： [hardware/hardware.md](hardware/hardware.md)
- 库级架构与通用规则： [../../docs/architecture.md](../../docs/architecture.md)、[../../docs/development.md](../../docs/development.md)

## 7. 设计边界（重要）

- 板级驱动与厂商固件保持在本实例 `software/` 内。
- 核心控制库统一来源于仓库根目录 `foc/`（单一事实来源）。
- 板级平台 API 适配应集中在 `software/Application/Source/foc_platform_api.c`。
