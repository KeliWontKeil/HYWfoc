# L2/Control 重构计划

## 阶段 0：扩展 motor 结构体
- [x] 新增 per-motor 状态字段（prev_control_mode, prev_control_mode_valid, prev_control_mode_check, speed_err_accum_rad, prev_mech_signed_rad, speed_state_valid, svpwm_lpf_state_valid, svpwm_lpf_phase_a/b/c）

## 阶段 1：创建 foc_ctrl_executor（替代 foc_ctrl_fast）
- [x] 创建 `foc_ctrl_executor.h`
- [x] 创建 `foc_ctrl_executor.c`（合并 foc_ctrl_fast.c + foc_ctrl_entry.c 的外环调度）

## 阶段 2：拆分 foc_ctrl_entry 的逻辑
- [ ] FOC_Control_ApplyConfig 移入 foc_ctrl_cfg.c/h
- [ ] 齿槽标定转发函数删除 → 调用者直接使用 compensation.h API
- [ ] 删除 foc_ctrl_entry.c/h

## 阶段 3：静态全局变量迁移到 motor 结构体
- [ ] foc_ctrl_outer_loop.c 的 3 个 static 变量
- [ ] foc_ctrl_actuation.c 的 SVPWM LPF 4 个 static 变量

## 阶段 4：纯化 foc_ctrl_init.c + 扩展 foc_service_handler.c
- [ ] 纯化 FOC_MotorInit（移除校准/齿槽装载）
- [ ] 扩展 FOC_Service_InitMotor（添加校准/齿槽装载）

## 阶段 5：SVPWM + Sensor 移入 L3
- [ ] 移动 4 个文件
- [ ] 更新所有 #include 路径

## 阶段 6：删除 foc_ctrl_fast
- [ ] 确认无引用后删除 foc_ctrl_fast.c/h

## 阶段 7：构建配置与文档
- [ ] 更新 builder.params
- [ ] 更新 docs/architecture.md
- [ ] 构建验证