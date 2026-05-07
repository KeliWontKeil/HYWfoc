# 下一阶段任务（NEXT_MISSION）

## 0. 目标与边界

- 当前稳定基线：`v1.6.0`
- 下一目标版本：`v1.6.1`

## 1. 本阶段目标（v1.6.0）

### 已完成
- ✅ **文档全面同步**：以实际代码为准更新所有文档，版本基线统一为 v1.6.0
- ✅ **修复文档-代码默认值不一致**：协议参数文档中的 sensor_sample_offset_percent、PID 增益、齿槽补偿参数、软切换自动阈值等默认值与代码完全对齐
- ✅ **修复 architecture.md**：修正 J/K/N 的参数归属描述（PID 角度调谐，非齿槽预留）；移除 `.github/DOCUMENTATION_STRUCTURE.md` 死引用
- ✅ **增补缺失符号定义**：`foc_symbol_defs.h` 中增补 `COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ` 和 `COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ`，实现代码使用的符号完成定义

## 2. 下阶段工作

### 近期（v1.6.x）
1. 参数标定重初始化接入协议
2. 状态机中部分二值化的状态量可以变为状态位，即使用一个8位的uint8_t储存8中状态，评估这样修改的收益，给出修改计划。

### 长期任务

1. 无感 FOC 算法实现。
2. 上位机实现（技术栈待定）。

## 其他
