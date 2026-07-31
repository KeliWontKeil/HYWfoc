#ifndef FOC_CFG_FEATURE_SWITCHES_H
#define FOC_CFG_FEATURE_SWITCHES_H

#include "LS_Config/foc_symbol_defs.h"

/* Telemetry output feature switches. */
#define DEBUG_STREAM_ENABLE_SEMANTIC_REPORT FOC_CFG_ENABLE
#define DEBUG_STREAM_ENABLE_OSC_REPORT FOC_CFG_ENABLE
/* Diagnostics feature switches. */
#define FOC_FEATURE_DIAG_OUTPUT FOC_CFG_ENABLE
/* Safety feature switches. */
#define FOC_FEATURE_UNDERVOLTAGE_PROTECTION FOC_CFG_ENABLE

/* ── 控制策略：低速/高速算法对 ── */
#define FOC_CONTROL_LOW_SOURCE   FOC_CONTROL_SRC_OPENLOOP
#define FOC_CONTROL_HIGH_SOURCE  FOC_CONTROL_SRC_SMO

/* 有感控制模式选择 */
#define FOC_BUILD_CONTROL_ALGO_SET FOC_CTRL_ALGO_BUILD_FULL

/* ── 传感器硬件使能 ── */
#define FOC_SENSOR_ENCODER_ENABLE         FOC_CFG_DISABLE
/*
 * Current sensing feature switch.
 *   FOC_CURRENT_SENSE_NONE (0)  - no current sensor; iq_measured = iq_target
 *   2U                         - two-phase sampling + C-phase reconstruction
 *   3U                         - three-phase direct sampling
 */
#define FOC_CURRENT_SENSE_PHASES 2U

/* Sensing feature switches. */
#define FOC_SENSOR_ANGLE_FAST_ENABLE FOC_CFG_DISABLE

#define FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE FOC_CFG_DISABLE /* 预留：电周期动态零偏抑制（当前未实现） */

/* Modulation and control feature switches. */
#define FOC_CURRENT_LOOP_PID_ENABLE FOC_CFG_ENABLE
#define FOC_CURRENT_SOFT_SWITCH_ENABLE FOC_CFG_ENABLE
#define FOC_SVPWM_PRE_LPF_ENABLE FOC_CFG_DISABLE
#define FOC_ZERO_VECTOR_CLAMP_ENABLE FOC_CFG_DISABLE

/* Initialization and calibration feature switches. */
#define FOC_INIT_CALIBRATION_ENABLE FOC_CFG_DISABLE
#define FOC_REINIT_ENABLE FOC_CFG_DISABLE //重初始化，内存占用高，功能意义不大，不建议开启

#define FOC_COGGING_COMP_ENABLE FOC_CFG_DISABLE
#define FOC_COGGING_CALIB_ENABLE FOC_CFG_DISABLE

/* 特殊控制状态退出：当存在任何非 NORMAL phase 功能时自动启用 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE) || (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
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
