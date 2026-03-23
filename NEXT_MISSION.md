# Next Mission

## 重要说明
- 本文件用于描述本次迭代的需求与约束，所有开发工作应以本文件为目标。
- 在开始编码前，请先阅读 `dev-guidelines/rules/` 目录下对应语言（`cn/` 或 `en/`）的规则文件。

## 版本
- **当前版本**：v0.2.7-dev

## 参考文档
- 规则文件（核心）：`dev-guidelines/rules/cn/`、`dev-guidelines/rules/en/`
- 项目结构说明：`docs/README.md`
- 开发流程说明：`docs/development.md`

---

## 本次目标
v0.2.x:完成无刷电机驱动核心功能实现：SVPWM,低速有感FOC,usart2参数控制。

### 任务1:
电流环修改为前馈PID：首先使用电阻粗算开环Id，然后再使用PID修正电流，以减小PID对采样误差的放大作用。电流过低时（<0.1A）关闭PID作用,仅开环。函数传入参数为iq。并在示波器参数打印中加入iq的值。

---

## 备注（可选）
