# HYW FOC

### 🚧 项目正在锐意开发中！ 
### 诸位点个Star好不好，~~求求你了~求求你了~帮帮可莉吧~~

- 项目名称：何易位FOC
- English Name：How Y-axis Works FOC

## 何意味？（这个项目是什么）

### 何易位FOC，意为“电机转子如何改变自己的位置”

HYW FOC 是一个可裁剪、结构清晰、可移植、扩展性强的单电机 FOC 可复用库，集成了指令控制系统。  
即：把“控制算法”和“硬件平台”尽量解耦的 FOC 工程实践仓库。

使用 AI 辅助开发（非 VibeCoding）🤖  
~~感谢copilot对本项目的大力支持~~

---

## 何以为？（这个项目可以用来做什么）

### 探索关于 FOC 的一切！🔍

- 作为可移植 FOC 核心库，集成到你自己的嵌入式工程。
- 直接使用示例代码 + 硬件参考实例，做板级验证与二次开发。
- 探索FOC算法实现方式，优化相关算法
- 学习“平台适配接口 + 控制核心解耦”的分层开发方式（不属于任何标准，作者自己摸索出来的，借鉴了相关思想，请勿将其作为某种标准化的开发方式）。
- 探索 AI 辅助嵌入式开发的新工作流。

### 当前能力一览

| 项目 | 当前状态 |
|---|---|
| 单电机有感 FOC | ✅ 可用 |
| 指令化查询/控制 | ✅ 可用 |
| 无感 FOC | 🧪 规划中 |
| 多电机驱动 | ❌ 不支持，以后可能也不会支持 |

---

## 何易位？（究竟是怎么控制电机的）

### 当然是 FOC 算法啦 ⚙️

当前有感控制链路包含：
- 电流采样 + 机械角度反馈
- 数学变换链（Clarke / Park / 反变换）
- SVPWM 调制输出（三相占空比）
- 外环控制（速度 / 角度）与内环电流控制

采用中断+状态机的方式完成相关控制任务


当前协议帧格式：
- `a<driver_id><cmd><subcmd><param>b`
- 单播 driver_id 范围：`0x32-0x7E`（可以打出来的字符）
- 广播地址：`0xFF`
- 默认本机 driver_id：`0x61`（`'a'`）

具体命令与参数请看 [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md)。

> 现阶段：有感 FOC，持续进行控制效果优化。  
> 后续计划：无感 FOC（完成有感FOC的大功率测试和算法优化后）

---

## 何依偎？（库的架构和依赖关系是怎样的）

```text
FOC_VSCODE/
├── foc/                       # 核心可复用库（L1-L3 + 接口层）
│   ├── include/
│   │   ├── interface/
│   │   ├── algorithm/
│   │   └── config/
│   ├── src/
│   │   ├── interface/
│   │   └── algorithm/
│   └── port/                  # 新平台适配模板
├── examples/                  # 平台/板级参考实例
├── docs/                      # 库级文档（与实例文档分层）
└── .github/                   # 工作流与协作规则
```

项目采用“核心库 + 示例工程”分层组织：
- 核心库保持平台无关，聚焦控制逻辑与接口契约。
- 示例工程负责具体硬件平台的驱动适配、构建与烧录流程。
- 通过平台 API 边界连接具体芯片与驱动实现。

---

## 何易为？（如何简单开始使用该项目）

### ~~当然是直接让 AI 来做啦~~ 🚀

#### 路径 A：一步步了解项目（古法开发）
1. 阅读 [docs/README.md](docs/README.md) 了解文档分层与阅读顺序。
2. 阅读 [docs/architecture.md](docs/architecture.md) 理解分层架构与接口边界。
3. 阅读 [docs/development.md](docs/development.md) 了解通用开发与验证流程。
4. 阅读 [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md) 了解协议与参数语义。
5. 阅读完文档后，你就可以开始阅读代码了，然后慢慢的一步一步尝试把这玩意整到你自己的板子上面去......~~真有人是这么干的吗~~

#### 路径 B：从 GD32F303_FOCExplore 开始上手
1. 准备硬件。我在hardware文件夹里直接放了整个嘉立创EDA专业版的工程，买元件，嫖板子，然后把它装起来！当然你要觉得我画的板子不咋地~~确实不咋地~~，可以参考原理图自己从头设计！
2. 打开 [examples/GD32F303_FOCExplore/software/Project.code-workspace](examples/GD32F303_FOCExplore/software/Project.code-workspace) 或 [examples/GD32F303_FOCExplore/software/Project.uvprojx](examples/GD32F303_FOCExplore/software/Project.uvprojx)。你可以选择你喜欢和熟悉的IDE。
3. 编译并烧录。
4. 按 [examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md) 指导，发送命令并进行验证/观察现象
5. 如果一切正常的话，去做你想做的吧，摸索代码/二次开发/优化算法都行。

#### 捷径（极其推荐）：把 AI_INITIALIZATION.md 甩给 AI
- [AI_INITIALIZATION.md](AI_INITIALIZATION.md) 会告诉 AI 如何初始化理解这个项目。
- 也可使用各家 AI 的项目初始化功能，让它先建立上下文再协助你开发。
- 然后有任何问题和想法全部交给AI吧
- ~~绝对不是我懒得写教程~~

---

## 何移位？（如何移植这个库）

想把它移植到新芯片/新板子，核心流程如下：
1. 在你的工程中引入 `foc/` 核心库。
2. 参考示例在主流程中完成初始化、调度与控制循环调用。
3. 实现平台 API（时钟、中断、PWM、采样、通信、指示灯等）。
4. 对照协议文档完成参数通道联调。

建议优先阅读：
- [foc/port/foc_platform_api_empty.c](foc/port/foc_platform_api_empty.c)
- [foc/include/interface/foc_platform_api.h](foc/include/interface/foc_platform_api.h)
- [examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c](examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c)

#### 主包主包，太麻烦了怎么办？
全~ 都~交给AI吧!从外设初始化到 API 实现，只需验证硬件行为即可!

---

## 何异谓？（如果想要提出意见该怎么做）

直接提 Issue 就好啦，欢迎：
- Bug 报告
- 文档纠错/重写(文档这一块跟AI斗智斗勇，力竭了)
- 新平台移植经验
- 控制效果优化建议
- ~~狠狠的辱骂这个库，写的什么垃圾玩意！~~

---

## 开发计划

- 当前稳定基线：v1.0.0
- 下一目标版本：v1.0.1（开源后稳定性与文档迭代）
- 当前定位：单电机 FOC 驱动库，不包含多电机扩展规划

- ~~把这个库开发成世界上最牛逼的FOC库~~（做梦中）

详细里程碑见 [NEXT_MISSION.md](NEXT_MISSION.md)。 

---

## 文档导航

### 库级文档（通用、可复用）
- [docs/README.md](docs/README.md)：库文档索引与边界规则
- [docs/architecture.md](docs/architecture.md)：架构、数据流、依赖关系
- [docs/development.md](docs/development.md)：通用开发流程与约束
- [docs/structure-and-dependency-tree.md](docs/structure-and-dependency-tree.md)：结构与依赖快照
- [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md)：协议与参数定义

### 实例级文档（平台相关、用户上手）
- [examples/GD32F303_FOCExplore/README.md](examples/GD32F303_FOCExplore/README.md)：实例入口文档
- [examples/GD32F303_FOCExplore/DEVELOPMENT.md](examples/GD32F303_FOCExplore/DEVELOPMENT.md)：实例构建/调试细节
- [examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md)：实例通信通道映射
- [examples/GD32F303_FOCExplore/hardware/README.md](examples/GD32F303_FOCExplore/hardware/README.md)：硬件目录说明
- [examples/GD32F303_FOCExplore/hardware/hardware.md](examples/GD32F303_FOCExplore/hardware/hardware.md)：硬件管脚速查

---

## 工作区使用说明

- 根工作区 [Project.code-workspace](Project.code-workspace)：用于仓库管理、文档治理、跨目录检视。
- 实例工作区 [examples/GD32F303_FOCExplore/software/Project.code-workspace](examples/GD32F303_FOCExplore/software/Project.code-workspace)：用于该实例构建/烧录/调试。

## 开源协作说明

- AI请遵循 [docs/engineering/dev-guidelines/rules/](docs/engineering/dev-guidelines/rules/) 下的规则文件。
- 请保持项目结构的清晰，确保它的可维护性。AI是好东西，但是请不要让它在项目上拉屎
- 请保持“库文档”和“实例文档”职责边界，不在同一文档混写。
- 每次改动若涉及行为、参数或流程，需在同次迭代同步更新相关文档。

## 许可证

MIT License，详见 [LICENSE](LICENSE)。

第三方组件许可证声明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

---

## 作者的碎碎念
### 欢迎各位大佬来提建议，或是直接为本项目添砖加瓦 🙌

这个项目是一边学一边做的，目的是学习以及积累项目经验，同时摸索AI在中型嵌入式项目中的使用  
作者本身理论知识不是很扎实，可能很多地方存在问题。  

~~没人看没人开发也没关系，我会在角落里默默开发的......~~


顺便推一下作者的另一个小项目：PortOSC  
https://github.com/KeliWontKeil/PortOSC