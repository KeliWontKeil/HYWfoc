#ifndef FOC_CFG_FEATURE_SWITCHES_H
#define FOC_CFG_FEATURE_SWITCHES_H

#include "LS_Config/foc_symbol_defs.h"

//1.架构级配置

/*ISR 架构模式：按需配置3ISR/2ISR模式*/
#define FOC_CURRENT_LOOP_ISR_MODE FOC_ISR_MODE_3ISR

/* PWM 插值功能开关：两种 ISR 模式均可独立裁剪 */
#define FOC_SVPWM_INTERP_ENABLE FOC_CFG_DISABLE

/* ── 控制策略：低速/高速算法对 ── */
#define FOC_CONTROL_LOW_SOURCE   FOC_CONTROL_SRC_ENCODER
#define FOC_CONTROL_HIGH_SOURCE  FOC_CONTROL_SRC_SMO

/* 控制模式选择 */
#define FOC_BUILD_CONTROL_ALGO_SET FOC_CTRL_ALGO_BUILD_SPEED_ONLY

/* ── 传感器硬件控制 ── */
#define FOC_SENSOR_ENCODER_ENABLE FOC_CFG_ENABLE
#define FOC_SENSOR_ANGLE_FAST_ENABLE FOC_CFG_DISABLE
/*
 * Current sensing feature switch.
 *   FOC_CURRENT_SENSE_NONE (0)  - no current sensor; iq_measured = iq_target
 *   2U                         - two-phase sampling + C-phase reconstruction
 *   3U                         - three-phase direct sampling
 */
#define FOC_CURRENT_SENSE_PHASES 2U

/* 调试输出功能裁剪. */
#define DEBUG_STREAM_ENABLE_SEMANTIC_REPORT FOC_CFG_ENABLE
#define DEBUG_STREAM_ENABLE_OSC_REPORT FOC_CFG_ENABLE
/* Diagnostics feature switches. */
#define FOC_FEATURE_DIAG_OUTPUT FOC_CFG_ENABLE

/* 欠压保护 */
#define FOC_FEATURE_UNDERVOLTAGE_PROTECTION FOC_CFG_ENABLE
/* 电流环电压基准来源：SETPOINT=设定值(params->vbus_voltage) / MEASURED=实测母线电压(sensor.vbus.filtered) */
#define FOC_CURRENT_LOOP_VOLTAGE_BASE_SOURCE FOC_VOLTAGE_BASE_SETPOINT

/* 高频/任意频率注入（被动工具：由调用方模块 Configure 后 SetEnable 生效，自身不含策略） */
#define FOC_INJECTION_ENABLE FOC_CFG_ENABLE
#define FOC_INJECTION_MODE FOC_INJECTION_MODE_BOTH_AUTO
#define FOC_INJECTION_ENABLE_AXIS_D FOC_CFG_ENABLE
#define FOC_INJECTION_ENABLE_AXIS_Q FOC_CFG_ENABLE
/* 轴裁剪掩码（编译期可用轴集合）与兜底轴：请求轴与掩码相与为 0 时收敛到兜底轴，
 * 保证配置完成后轴掩码非零，ISR 侧无需再做合法性判定 */
#define FOC_INJECTION_AXIS_MASK \
    (((FOC_INJECTION_ENABLE_AXIS_D == FOC_CFG_ENABLE) ? FOC_INJECTION_AXIS_D : 0U) | \
     ((FOC_INJECTION_ENABLE_AXIS_Q == FOC_CFG_ENABLE) ? FOC_INJECTION_AXIS_Q : 0U))
#define FOC_INJECTION_AXIS_FALLBACK \
    ((FOC_INJECTION_ENABLE_AXIS_D == FOC_CFG_ENABLE) ? FOC_INJECTION_AXIS_D : FOC_INJECTION_AXIS_Q)

/* 声学回报（蜂鸣器级）：RTTTL 铃声表 → dq 电压叠加。单一功能宏，无运行时禁能
 * （是否发声由触发决定，裁剪即整体移除）；AXIS 决定叠加到 d 轴（默认，几乎不产生
 * 转矩脉动）或 q 轴。音频波形与电流环同拍生成，无独立音频定时器/缓冲。 */
#define FOC_ACOUSTIC_ENABLE FOC_CFG_ENABLE
#define FOC_ACOUSTIC_AXIS FOC_ACOUSTIC_AXIS_D

/* 控制行为裁剪 */
#define FOC_CURRENT_LOOP_PID_ENABLE FOC_CFG_ENABLE
#define FOC_CURRENT_SOFT_SWITCH_ENABLE FOC_CFG_ENABLE
#define FOC_ZERO_VECTOR_CLAMP_ENABLE FOC_CFG_DISABLE

/* 标定及齿槽补偿 */
/* 有感对齐/标定状态机总开关：上电由 STARTUP 控制阶段自动执行，协议 aaYI 命令复用同一状态机。
 * 原 FOC_INIT_CALIBRATION_ENABLE（上电阻塞标定）与 FOC_REINIT_ENABLE（非阻塞重初始化）合并为
 * 单一能力宏，使该能力只有一份实现（禁止再出现两份并存）。 */
#define FOC_ALIGN_ENABLE FOC_CFG_ENABLE

#define FOC_COGGING_COMP_ENABLE FOC_CFG_DISABLE
#define FOC_COGGING_CALIB_ENABLE FOC_CFG_DISABLE

/* 特殊控制状态退出：当存在任何非 NORMAL phase 功能时自动启用 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE) || (FOC_ALIGN_ENABLE == FOC_CFG_ENABLE)
#define FOC_SPECIAL_PHASE_ABORT_ENABLE FOC_CFG_ENABLE
#else
#define FOC_SPECIAL_PHASE_ABORT_ENABLE FOC_CFG_DISABLE
#endif
/*
 * Protocol command trimming switches.
 *
 * The minimal control protocol set is always enabled and not guarded by these
 * macros: P:A/R/S/D, S:M, Y:R/C.
 *
 * The following protocol features are controlled by their corresponding
 * feature macro (FOC_*_ENABLE) rather than a separate protocol macro:
 *   - COGGING_COMP   — uses FOC_COGGING_COMP_ENABLE
 *   - SOFT_SWITCH    — uses FOC_CURRENT_SOFT_SWITCH_ENABLE
 *   - SAMPLE_OFFSET  — uses FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE
 */
#define FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT FOC_CFG_ENABLE
#define FOC_PROTOCOL_ENABLE_BATCH_READ FOC_CFG_ENABLE
#define FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING FOC_CFG_ENABLE
#define FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING FOC_CFG_DISABLE
#define FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING FOC_CFG_DISABLE
#define FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING FOC_CFG_DISABLE

#endif /* FOC_CFG_FEATURE_SWITCHES_H */
