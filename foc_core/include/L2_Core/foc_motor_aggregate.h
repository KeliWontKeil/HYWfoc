#ifndef FOC_MOTOR_AGGREGATE_H
#define FOC_MOTOR_AGGREGATE_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_source_mgr.h"
#include "L2_Core/Control/foc_ctrl_outer_loop.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Control/foc_ctrl_estim_smo.h"
#include "L2_Core/Control/foc_ctrl_estim_hfi.h"
#include "L2_Core/Control/foc_ctrl_openloop.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_sens_cogging_calib.h"
#include "L2_Core/Control/foc_ctrl_sens_reinit.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"

/* ========== Mode transition tracking ========== */
typedef struct {
    uint8_t prev_control_mode;
    uint8_t prev_control_mode_valid;
    uint8_t prev_control_mode_check;
} foc_mode_transition_t;

/* ========== ISR 测速 ========== */
typedef struct {
    uint8_t  fast_current_div_counter;
    uint32_t current_loop_cycles;
    uint32_t pwm_isr_cycles;
} foc_isr_timing_t;

/* ========== Motor aggregate state ========== */
/* 内存布局保持与 v2.0.6 一致：成员顺序不得调整 */
typedef struct foc_motor_t {
    /* ─── 传感器（L3）─── */
    sensor_data_t sensor;

    /* ─── 控制运行时（ISR 数据总线）─── */
    foc_control_runtime_t ctrl;
    foc_alpha_beta_phase_t alpha_beta;

    /* ─── 源管理器 ─── */
    foc_active_source_state_t active_source_state;
    foc_source_mgr_state_t source_mgr_state;
    foc_source_switch_state_t source_switch_state;

    /* ─── SVPWM ─── */
    svpwm_interp_state_t svpwm;

    /* ─── 输出快照与状态 ─── */
    foc_applied_output_state_t applied_output;
    foc_phase_output_state_t phase_output_state;
    foc_motor_state_t state;

    /* ─── ISR 测速 ─── */
    foc_isr_timing_t isr_timing;

    /* ─── 电机物理参数（冷路径，只读）─── */
    foc_motor_params_t params;

    /* ─── PID（冷路径）─── */
    foc_pid_t torque_current_pid;
    foc_pid_t speed_pid;
    foc_pid_t angle_pid;

    /* ─── 控制配置（冷路径）─── */
    foc_control_cfg_t cfg;

    /* ─── 服务与辅助 ─── */
    foc_encoder_services_state_t encoder_services;
    foc_outer_loop_private_t outer_loop;
    foc_mode_transition_t       mode_transition;

    /* ─── 条件编译区 ─── */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    foc_estim_smo_state_t estim_smo_state;
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    foc_estim_hfi_state_t estim_hfi_state;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    foc_openloop_state_t openloop_state;
#endif
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    foc_current_soft_switch_status_t current_soft_switch_status;
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_comp_status_t cogging_comp_status;
    int16_t cogging_comp_table_q15[FOC_COGGING_LUT_POINT_COUNT];
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_calib_state_t cogging_calib_state;
#endif
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
    foc_reinit_state_t reinit_state;
#endif

    /* ─── 控制参考（控制 ISR 单点发布，电流环 ISR 原子获取） ─── */
    foc_control_ref_t ctrl_ref;
    volatile uint8_t ctrl_ref_ready;
} foc_motor_t;

#endif /* FOC_MOTOR_AGGREGATE_H */
