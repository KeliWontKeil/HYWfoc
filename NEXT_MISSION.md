# Next Mission

> 当前稳定基线：**v1.7.0**
> 下一活跃目标：**v2.0.0**

---

## v1.7.0 已完成

- [x] **参数标定重初始化接入协议**
  - 新增系统子命令 `Y:I` 触发运行时重初始化
  - 完整的 reinit_pending 传递链（L2 各层同步添加 ClearReinit 接口）
  - `FOC_App_ReInitMotor()` + `g_reinit_in_progress` ISR/控制路径栅保护
  - 提取 `FOC_App_InitMotorHardware()` 公共初始化路径
- [x] **齿槽补偿边界不连续修复增强**
  - `COGGING_BOUNDARY_Q15_THRESHOLD` 从局部宏提升为配置宏（100，原1200）
  - 新增 `FOC_COGGING_BOUNDARY_BLEND_WIN` 窗口宽度配置（默认3）
  - Stage 2 窗口边界漂移渐进校正算法
- [x] **初始化标定功能恢复**（`FOC_INIT_CALIBRATION_ENABLE` → ENABLE）
- [x] **齿槽标定默认值优化**（gain_k 0.05→0.03, speed 0.8→0.6, iq 0.50→0.30, passes 2→1）
- [x] **电机初始化默认值调整**（标定启用后方向/零位改为 UNDEFINED）
- [x] **齿槽查询优化**（移除冗余角度归一化和输出钳位）
- [x] **协议帧入口时序修复**（`RunPwmInterpolationIsr` 移入 `fast_current_loop_enabled` 检查之后）
- [x] **快照结构体新增** `reinit_pending` 字段

---

## v2.0.0 规划：有感 FOC 收尾版本

### 内存/性能优化
- [ ] 状态机中部分二值化的状态量改为状态位，一个 `uint8` 或 `uint16` 存储多个布尔状态
- [ ] 检查 ROM/RAM 占用，评估是否有进一步裁剪空间
- [ ] 协议参数表从结构体转换为更紧凑的运行时访问模式

### 控制效果优化
- [ ] 完善速度环和角度环的 PID 参数整定
- [ ] 齿槽补偿在各种工况下的效果验证与参数微调
- [ ] 电流环抗饱和策略在不同电机上的适用性验证
- [ ] 低电流软切换效果最终定型

### 协议体验
- [ ] 完善示波器通道输出，新增更多可观测变量
- [ ] 协议文档完整性和准确性终审

### 文档与工程
- [ ] 文档全面审计，消除所有代码-文档不一致
- [ ] 补充缺失的 API 文档和模块说明
- [ ] 实例工程依赖/构建流程文档完善

---

## 长期规划

- **无感 FOC**：完成有感 FOC 的大功率测试和算法优化后开始
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
