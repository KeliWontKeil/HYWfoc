# Next Mission

## 重要说明
- 本文件用于描述本次迭代的需求与约束，所有开发工作应以本文件为目标。
- 在开始编码前，请先阅读 `dev-guidelines/rules/` 目录下对应语言（`cn/` 或 `en/`）的规则文件。

## 版本
- **当前版本**：v0.2.2-dev

## 参考文档
- 规则文件（核心）：`dev-guidelines/rules/cn/`、`dev-guidelines/rules/en/`
- 项目结构说明：`docs/README.md`
- 开发流程说明：`docs/development.md`

---

## 本次目标
v0.2.x:完成无刷电机驱动核心功能实现：SVPWM,低速有感FOC,usart2参数控制。

### 任务1:
在application文件夹中添加新文件用于存储相关数学算法（两种核心变化，后面可能的观测器等），目前只需要加入FOC其中的两个核心变换：clerk变换和park变换及其反变换。

### 任务 2：
调整SVPWM的输入参数，要求输入直接为反clerk变换后相电压的输出。同时要能根据输入的设定电压与电源电压的比例调整SVPWM的幅值。

### 任务 3：
在application文件夹中添加新文件用于存储FOC算法（之后可能会包括开环，有感低速，无感高速等算法），并实现开环FOC，即设置一个恒定的Uq，通过反变换输出SVPWM控制电机。

---

## 备注（可选）
- 已完成首版落地：
	- 新增 `math_transforms` 模块，包含 Clarke/Park 及其反变换
	- SVPWM 输入改为三相电压（反Clarke输出），并按 `set_voltage/vbus` 比例缩放
	- 新增 `foc_control` 模块，实现恒定 Uq 的开环FOC链路并接入1kHz控制回调
