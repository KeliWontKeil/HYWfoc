# Next Mission

## 重要说明
- 本文件用于描述本次迭代的需求与约束，所有开发工作应以本文件为目标。
- 在开始编码前，请先阅读 `dev-guidelines/rules/` 目录下对应语言（`cn/` 或 `en/`）的规则文件。

## 版本
- **当前版本**：v0.2.6-dev

## 参考文档
- 规则文件（核心）：`dev-guidelines/rules/cn/`、`dev-guidelines/rules/en/`
- 项目结构说明：`docs/README.md`
- 开发流程说明：`docs/development.md`

---

## 本次目标
v0.2.x:完成无刷电机驱动核心功能实现：SVPWM,低速有感FOC,usart2参数控制。

### 任务1:
请基于已有的力矩控制函数加入角度环，实现电机位置控制。请使用一个新的函数来存储角度算法。可以在电机结构体中添加一个新的参数来存储累计位置。


---

## 备注（可选）
