# Next Mission

> 当前稳定基线：**v2.1.0**
> 下一活跃目标：**v2.2.0（待定）**

---
## v2.0.6:函数重构与结构解耦和清晰化

## v2.0.7:算法优化（已完成）
- [x] 所有重复定义的算法函数去重/精简，如`FOC_NormalizeDt` / `wrap_2pi` / `OpenLoop_Wrap2Pi` 去重
- [x] 对实际逻辑没有任何影响的重复赋值
- [x] 计数变量/累加值设上限防溢出
- [x] 函数重入与数据竞争（去重部分）：删除 64 位运算（`double`/`%f`），消除双字撕裂隐患
- [x] 参数调整行为收敛：删除运行时 cfg_dirty 链；PID 增益写入不清积分；`P:W` 开放运行时可写

## v2.1.0:API 接口优化（已完成）
- [x] 平台 API 契约重构：接口面稳定 + 行为自适应 + 三档契约标注（必须/按需/可选）
- [x] 参数传递统一：编译期固定配置平台内部读宏，初始化接口统一无参（PWMInit/SensorInputInit/ControlTickSourceInit）
- [x] 回调类型统一为 `FOC_Platform_IsrCallback_t`（删除 TickCallback/PwmIsrCallback 冗余双类型）
- [x] 通信源收敛为枚举 + 单 API `CommSource_ReadFrame`；删除孤儿 `IsFrameReady`×4、`UndervoltageProtect`
- [x] 区块重组与描述修正（WaitMs/MemoryBarrier 归 Runtime、ReadVbusVoltage 归 Sensor、守卫注释、注释中文化）
- [x] `SetControlRuntimeInterrupts` 更名为 `SetControlInterruptsEnabled`

## v2.2.0（待定）
1. 速域切换问题
- 速域切换扰动过于敏感，降级策略过于激进：open->ENCODER,负载上升后速度稍微下降
- 切换速度跳跃问题
- 切换SMO抖震
2. SMO算法问题
- SMO算法优化，变量重算
- SMO估计滞后问题
- SMO调试（有感->SMO）

## 开发分支：新硬件
- 新硬件适配
- 电流采样频率问题:不能修改，修改后直接控制出问题

## 持续调优和开发（任意版本均可进行）
- 控制效果优化与参数调优
- 文档审计与补充

## 长期规划
- **无感 FOC**：完成有感 FOC 的大功率测试和算法优化后开始
- **优化**：inline函数替换，定点运算支持
- **测试用例**：相关模块测试用例
- **上位机**：开发图形化参数修改上位机
- **多电机**：当前不支持多电机，后续可能会支持（存疑）

---
