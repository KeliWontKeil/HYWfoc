# Next Mission

> 当前稳定基线：**v1.7.x**
> 下一活跃目标：**v1.8.0**

---

## v1.7.4

---
## [1.7.5] - 2026-06-10
- 电周期平均动态偏置补偿：新增 `FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE`，在电流环中通过累计电周期平均值提取并补偿低测采样零点漂移

## v1.8.x 新增状态观测器
- 电流采样措施，抑制PWM纹波漂移导致的电流重建误差（**v1.7.5 已实现**）

## v2.0.0 规划：有感 FOC 收尾版本

### 控制效果优化
- [ ] 电流环抗饱和策略在不同电机上的适用性验证
- [ ] 低电流软切换效果最终定型

### 协议
- [ ] 协议文档完整性和准确性终审

### 文档与工程
- [ ] 文档全面审计，消除所有代码-文档不一致
- [ ] 补充缺失的 API 文档和模块说明
- [ ] 实例工程依赖/构建流程文档完善

---

## 长期规划

- **无感 FOC**：完成有感 FOC 的大功率测试和算法优化后开始
- **优化**：inline函数替换，定点运算支持
- **测试用例**：相关模块测试用例
- **上位机**：开发图形化参数修改上位机（参见 PortOSC: https://github.com/KeliWontKeil/PortOSC）
- **多电机**：当前不支持多电机，后续可能会支持（存疑）

---

## 开发说明

- 完成当前迭代任务后，记得更新以下文件：
  - `CHANGELOG.md`（变更记录）
  - `NEXT_MISSION.md`（当前迭代任务和下一步计划）
  - `copilot-instructions.md`（版本基线）
  - `docs/README.md`（版本基线）
  - `README.md`（稳定基线与下一版本号）
  - `.clinerules/hywfoc-project-rules.md`（版本基线）
  - `docs/architecture.md`（如结构/依赖有变化）
  - `docs/protocol-parameters-bilingual.md`（如协议有变化）
