# HYW FOC

### 🚧 项目正在锐意开发中！ 
### 诸位点个Star好不好，~~求求你了~求求你了~帮帮可莉吧~~

- 项目名称：何易位FOC
- English Name：How Y-axis Works FOC（强行翻译）

- 当前项目状态：单电机有感 FOC 驱动库
- 当前稳定基线：v1.4.1
- 下一活跃目标版本：v1.4.2

## 何意味？（这个项目是什么）

### 何易位FOC，意为“电机转子如何改变自己的位置”

HYW FOC 是一个可裁剪、结构清晰、可移植、扩展性强的单电机 FOC 可复用库。
- FOC算法无刷电机驱动
- 集成指令控制系统和多种保护机制
- 纯C语言实现，不依赖任何实时系统/运行环境
- 控制算法和硬件平台解耦
- 部分功能可使用宏裁剪

使用 AI 辅助开发（非 VibeCoding）🤖  
- 开发过程中使用了多种AI插件开发，copilot、cline
~~感谢copilot对本项目的大力支持~~  
~~先别支持了狗日的微软把我的额度干废了~~  
~~还是看看远处的deepseek v3.4吧家人们~~  
-项目结构进一步调整中，AI留下的屎山还没完全收拾完

演示视频：暂时没拍  
硬件开源地址：暂时没有，硬件仍处于前期验证阶段，目前想要的话直接去examples/GD32F303_FOCExplore/hardware文件夹里找，是个嘉立创EDA的工程，还没开源出去

---

## 何以为？（这个项目可以用来做什么）

### 探索关于 FOC 的一切！🔍

- 作为可移植 FOC 核心库，集成到你自己的嵌入式工程。
- 直接使用示例代码 + 硬件参考实例，进行板级验证与二次开发。
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

本库采用中断+状态机的方式完成相关控制任务  
其中，控制链路完全存在于中断中，确保控制的实时性和可靠性
其他协议解析，状态判断等逻辑则是以状态机的形式在主循环中运行，依靠相关中断实现状态的改变

当前有感控制链路包含：
- 电流、机械角度采样及数据处理、滤波
- 数学变换链（Clarke / Park / 反变换）
- SVPWM 调制输出（三相占空比）
- 外环控制（速度 / 角度）与内环电流控制
- 电流环开环/闭环软切换抑制小电流下噪声

电机可使用协议帧控制，当前协议帧格式：
- `a<driver_id><cmd><subcmd><param>b`
- 单播 driver_id 范围：`0x32-0x7E`（可以打出来的字符）
- 广播地址：`0xFF`
- 默认本机 driver_id：`0x61`（`'a'`）
具体命令与参数请看 [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md)。

> 现阶段：有感 FOC，持续进行控制效果优化。  
> 后续计划：无感 FOC（完成有感FOC的大功率测试和算法优化后）

---

## 何依偎？（库的架构和依赖关系是怎样的）

### 具体代码架构描述仍有待进一步补充（等我AI额度恢复了再说这些文档吧）

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
│   └── port/                  # 新平台适配模板(接口层)
├── examples/                  # 平台/板级参考实例
├── docs/                      # 库级文档（与实例文档分层）
└── .github/                   # 工作流与协作规则
```

项目采用“核心库 + 示例工程”分层组织：
- 核心库保持与平台完全无关，聚焦控制逻辑与接口契约。
- 通过平台 API 及相关中断回调连接具体的外设相关驱动和实现（L4层）。
- 示例工程演示了具体硬件平台的驱动适配方式和构建流程。

项目代码可裁剪，裁剪对象包括：
- 部分可选的滤波器
- 部分用于调节参数的通信协议
- 控制算法
具体协议裁剪开关见 [foc/include/config/foc_cfg_feature_switches.h](foc/include/config/foc_cfg_feature_switches.h)，裁剪映射关系见协议文档的“Build-Time Protocol Trimming”小节。
---

## 何易为？（如何简单开始使用该项目）

### ~~当然是直接让 AI 来做啦~~ 🚀

#### 路径 A：人力理解+古法开发
1. 阅读 [docs/README.md](docs/README.md) 了解文档分层与阅读顺序。
2. 阅读 [docs/architecture.md](docs/architecture.md) 理解分层架构与接口边界。
3. 阅读 [docs/development.md](docs/development.md) 了解通用开发与验证流程。
4. 阅读 [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md) 了解协议与参数语义。
5. 阅读完文档后，你就可以开始阅读代码了，然后慢慢的一步一步尝试把这玩意整到你自己的板子上面去......~~什么年代了还在搞传统开发~~

#### 路径 B：从 GD32F303_FOCExplore 开始上手
1. 准备硬件。我在hardware文件夹里直接放了整个嘉立创EDA专业版的工程，买元件，嫖板子，然后把它装起来！当然你要觉得我画的板子不咋地~~确实不咋地~~，可以参考原理图自己从头设计。
2. 打开 [examples/GD32F303_FOCExplore/software/Project.code-workspace](examples/GD32F303_FOCExplore/software/Project.code-workspace) 或 [examples/GD32F303_FOCExplore/software/Project.uvprojx](examples/GD32F303_FOCExplore/software/Project.uvprojx)。你可以选择你喜欢和熟悉的IDE。
3. 调整相关宏定义设置相关参数和算法裁剪，编译并烧录。
4. 按 [examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md) 指导，发送命令并进行验证/观察现象
5. 如果一切正常的话，去做你想做的吧，摸索代码/二次开发/优化算法都行。

#### 捷径（极其推荐）：把 AI_INITIALIZATION.md 甩给 AI
- [AI_INITIALIZATION.md](AI_INITIALIZATION.md) 会告诉 AI 如何初始化理解这个项目。
- 也可使用各家 AI 的项目初始化功能，让它先建立上下文再协助你开发。
- 然后有任何问题和想法全部交给AI吧
- ~~绝对不是我懒得写教程~~

---

## 何移位？（如何移植这个库）

由于具体的硬件读取相关逻辑，如采样/发送/协议解析完全依赖api接口实现，故库的核心内容并不包含例如：串口DMA、PWM同步电流采样、I2C协议、传感器读取等硬件层相关内容
- 也就是说，你不能指望把这个库拿来就用
- 你需要手动实现相关的硬件和芯片外设驱动
- 同时，你实现这些外设的方式决定了这个库的效果，如：电流采样是否与PWM同步？使用什么传感器和算法获取机械角度？使用DMA进行通信还是直接CPU硬通信？电流采样噪声如何？

故如果想把它移植到新芯片/新板子，核心流程如下：
1. 在你的工程中引入 `foc/` 核心库。
2. 参考示例在主流程中完成初始化、调度与控制循环调用。
3. 实现平台 API，比如相关时钟、中断、PWM、采样、通信、指示灯等（这是最麻烦的一步），我在空API实现中给出了实现的提示和注意事项
4. 对照协议文档完成参数通道联调。

建议研究一下这几个文件：
- [foc/port/foc_platform_api_empty.c](foc/port/foc_platform_api_empty.c)：一个完全空白的API实现
- [foc/include/interface/foc_platform_api.h](foc/include/interface/foc_platform_api.h)：所有需要实现的相关API
- [examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c](examples/GD32F303_FOCExplore/software/Application/Source/foc_platform_api.c)：具体API实现的一个工程实例

#### 主包主包，太麻烦了怎么办？
全~都~交给AI吧!从外设初始化到 API 实现，只需验证硬件行为即可!（而且效果应该还不错，我自己写GD32标准库的初始化就是AI干的，只有一点小问题要改）

---

## 何异谓？（如果有不同意见该怎么做）

直接提 Issue 就好啦，欢迎：
- Bug 报告
- 文档纠错/重写(文档这一块跟AI斗智斗勇，力竭了)
- 新平台移植经验
- 控制效果与算法优化建议
- ~~狠狠的辱骂这个库，写的什么垃圾玩意！~~

---

## 何以谓？（作者想说什么）
### 欢迎各位大佬来提建议，或是直接为本项目添砖加瓦 🙌

这个项目是一边学一边做的，目的是学习以及积累项目经验，同时摸索AI在嵌入式项目中的使用。  
作者本身理论知识不是很扎实，可能很多地方存在问题。  
很多AI开发的完全不能细看，看起来还行细看一坨，最近几个版本修了很多重要的地方，其他看着有点乱的地方尽力了，后面开发的时候再慢慢改吧

~~项目没人看没人开发也没关系，我会在角落里默默开发的......~~

顺便推一下作者的另一个很垃圾的上位机小项目：PortOSC  
https://github.com/KeliWontKeil/PortOSC

---
## 其他开发相关
### 开发计划

下版本目标：
1. 继续完善文档
2. 抗齿槽效应前馈补偿算法调整（目前不可用）

长期目标：
1. 完成无感FOC开发
2. ~~把这个库开发成世界上最牛逼的FOC库~~（做梦中）


详细里程碑见 [NEXT_MISSION.md](NEXT_MISSION.md)。 


### 文档导航

#### 库级文档（驱动库通用文档）
- [docs/README.md](docs/README.md)：库文档索引与边界规则
- [docs/architecture.md](docs/architecture.md)：架构、数据流、依赖关系
- [docs/development.md](docs/development.md)：通用开发流程与约束
- [docs/protocol-parameters-bilingual.md](docs/protocol-parameters-bilingual.md)：协议与参数定义

#### 实例级文档（以具体项目为准的具体工程实例文档）
- [examples/GD32F303_FOCExplore/README.md](examples/GD32F303_FOCExplore/README.md)：实例入口文档
- [examples/GD32F303_FOCExplore/DEVELOPMENT.md](examples/GD32F303_FOCExplore/DEVELOPMENT.md)：实例构建/调试细节
- [examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md](examples/GD32F303_FOCExplore/PROTOCOL_ADAPTATION.md)：实例通信通道映射
- [examples/GD32F303_FOCExplore/hardware/README.md](examples/GD32F303_FOCExplore/hardware/README.md)：硬件目录说明
- [examples/GD32F303_FOCExplore/hardware/hardware.md](examples/GD32F303_FOCExplore/hardware/hardware.md)：硬件管脚速查


### 工作区使用说明

- 该项目可直接在VSCODE完整打开
- 根工作区 [Project.code-workspace](Project.code-workspace)：用于仓库管理、文档治理、跨目录检视。
- 实例工作区 [examples/GD32F303_FOCExplore/software/Project.code-workspace](examples/GD32F303_FOCExplore/software/Project.code-workspace)：用于该实例构建/烧录/调试。

### 开源协作说明

- AI请遵循 [docs/engineering/dev-guidelines/rules/](docs/engineering/dev-guidelines/rules/) 下的规则文件。
- 请保持项目结构和代码逻辑的清晰，确保它的可维护性。
- 请保持“库文档”和“实例文档”职责边界，不在同一文档混写。且每次改动需在同步更新相关文档。
- AI是好东西，但是请不要让它在项目上拉屎。该项目所有代码必须经过人工审查。


### 许可证

MIT License，详见 [LICENSE](LICENSE)。

第三方组件许可证声明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

---