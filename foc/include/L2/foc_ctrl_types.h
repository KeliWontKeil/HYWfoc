#ifndef FOC_CTRL_TYPES_H
#define FOC_CTRL_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3/foc_math_types.h"

/* ========== Alpha-beta / three-phase voltage state ========== */
typedef struct {
    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
} foc_alpha_beta_phase_t;

/* ========== Outer-loop runtime state ========== */
typedef struct {
    float  speed_err_accum_rad;
    float  prev_mech_signed_rad;
    uint8_t speed_state_valid;
} foc_outer_loop_state_t;

/* ========== Control mode transition tracking ========== */
typedef struct {
    uint8_t prev_control_mode;
    uint8_t prev_control_mode_valid;
    uint8_t prev_control_mode_check;
} foc_mode_transition_t;

/* ========== SVPWM LPF filter state ========== */
typedef struct {
    uint8_t  valid;
    float    phase_a;
    float    phase_b;
    float    phase_c;
} foc_svpwm_lpf_state_t;

/* ========== Iq LPF filter state ========== */
typedef struct {
    uint8_t  valid;
    float    state;
} foc_iq_lpf_state_t;

/* ========== SVPWM output snapshot type ========== */
typedef struct {
    uint8_t sector;
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_output_t;

/* ========== 故障码枚举（per-motor） ========== */
typedef enum {
    FOC_FAULT_NONE = 0U,
    FOC_FAULT_SENSOR_ADC_INVALID = 1U,
    FOC_FAULT_SENSOR_ENCODER_INVALID = 2U,
    FOC_FAULT_UNDERVOLTAGE = 3U,
    FOC_FAULT_PROTOCOL_FRAME = 4U,
    FOC_FAULT_PARAM_INVALID = 5U,
    FOC_FAULT_INIT_FAILED = 6U
} foc_fault_code_t;

/* ========== 运行时状态（per-motor） ========== */
typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t reinit_pending;
    uint8_t last_fault_code;
    uint8_t cfg_dirty;
    uint8_t motor_enabled;
    uint8_t control_mode;
    uint8_t pending_system_action;
    uint8_t current_loop_ready;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
} foc_motor_state_t;

/* ========== 系统动作枚举（pending_system_action） ========== */
#define FOC_SYSACTION_NONE           0U
#define FOC_SYSACTION_COGGING_START  1U
#define FOC_SYSACTION_COGGING_DUMP   2U
#define FOC_SYSACTION_COGGING_EXPORT 3U

/* ========== Current soft-switch status ========== */
typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
} foc_current_soft_switch_status_t;

/* ========== Cogging compensation status ========== */
typedef struct {
    uint8_t enabled;
    uint8_t available;
    uint8_t source;
    uint16_t point_count;
    float iq_lsb_a;
    float speed_gate_rad_s;
    float iq_limit_a;
    float calib_gain_k;
} foc_cogging_comp_status_t;

/* Cogging calibration runtime state */
typedef struct {
    uint8_t in_progress;
    uint8_t progress_percent;
    uint16_t point_index;
    uint8_t completed_pass_count;
    float pred_mech_angle;
    float travel_accum_rad;
    float angle_prev_rad;
    uint16_t last_lut_index;
    uint16_t bins_collected;
    uint8_t pass_num;
    uint8_t rev_count;
    uint8_t last_reported_progress;
    uint8_t saved_softswitch_enabled;
    uint8_t saved_softswitch_mode;
    uint8_t request_start;
    uint8_t request_dump;
    uint8_t request_export;
} foc_cogging_calib_state_t;

/* ========== SVPWM 插值引擎状态 ========== */
typedef struct {
    svpwm_output_t output;
    float duty_a_current;
    float duty_b_current;
    float duty_c_current;
    float duty_a_target;
    float duty_b_target;
    float duty_c_target;
    float duty_a_step;
    float duty_b_step;
    float duty_c_step;
    uint16_t interp_steps_total;
    uint16_t interp_step_index;
} svpwm_interp_state_t;

/* ========== Sensor data snapshot ========== */
typedef struct {
    kalman_filter_t current_a;
    kalman_filter_t current_b;
    kalman_filter_t current_c;
    kalman_filter_t mech_angle_rad;
    uint8_t adc_valid;
    uint8_t encoder_valid;
    float vbus_voltage_raw;
    float vbus_voltage_filtered;
    uint8_t vbus_valid;
} sensor_data_t;

/* ========== Torque mode ========== */
typedef enum {
    FOC_TORQUE_MODE_OPEN_LOOP = 0,
    FOC_TORQUE_MODE_CURRENT_PID = 1
} foc_torque_mode_t;

/* ========== Motor aggregate state ========== */
/*
 * 所有配置字段直接作为 foc_motor_t 顶层字段。
 * 运行时字段按功能块聚合为子结构体。
 */
typedef struct {
    /* === 运行时状态 === */
    foc_motor_state_t state;

    /* === PID 控制器对象 === */
    foc_pid_t torque_current_pid;
    foc_pid_t speed_pid;
    foc_pid_t angle_pid;

    /* Motor physical parameters and calibration outputs. */
    float phase_resistance;
    uint8_t pole_pairs;
    float mech_angle_at_elec_zero_rad;
    int8_t direction;
    float vbus_voltage;

    /* DQ and electrical-angle runtime states. */
    float electrical_phase_angle;
    float ud;
    float uq;
    float set_voltage;

    /* Current-loop runtime states. */
    float iq_target;
    float iq_measured;
    float cogging_speed_ref_rad_s;

    /* Mechanical angle accumulation states. */
    float mech_angle_accum_rad;
    float mech_angle_prev_rad;
    uint8_t mech_angle_prev_valid;
    int32_t mech_turn_count;

    /* Alpha-beta, three-phase voltages (子结构体). */
    foc_alpha_beta_phase_t alpha_beta;

    /* SVPWM 插值引擎状态 */
    svpwm_interp_state_t svpwm;

    /* ====== 配置字段（原 cfg + runtime_cfg，直接写入） ====== */

    /* 控制目标参数 */
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;

    /* Fine-tuning 参数 */
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;

    /* ====== 运行时状态（子结构体） ====== */

    foc_current_soft_switch_status_t current_soft_switch_status;
    uint8_t current_soft_switch_blend_initialized;

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    float    ecycle_offset_dyn_a;
    float    ecycle_offset_dyn_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float    ecycle_offset_dyn_c;
#endif
    float    ecycle_prev_mech_angle;
    float    ecycle_accu_mech_delta;
    uint16_t ecycle_sample_count;
    float    ecycle_accum_a;
    float    ecycle_accum_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float    ecycle_accum_c;
#endif
    uint8_t  ecycle_offset_valid;
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_comp_status_t cogging_comp_status;
    int16_t cogging_comp_table_q15[FOC_COGGING_LUT_POINT_COUNT];
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_calib_state_t cogging_calib_state;
#endif
#else
    uint8_t _cogging_padding;
#endif

    /* Sensor snapshots */
    sensor_data_t sensor;
    sensor_data_t sensor_fast;

    /* Zero offsets */
    float sensor_zero_offset_a;
    float sensor_zero_offset_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float sensor_zero_offset_c;
#endif

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
    uint8_t  sensor_angle_lpf_valid;
    float    sensor_angle_lpf_state;
#endif

    uint8_t  fast_current_div_counter;

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_LOOP_IQ_LPF_ENABLE == FOC_CFG_ENABLE))
    foc_iq_lpf_state_t iq_lpf;
#endif

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    uint8_t  prev_softswitch_active_mode;
#endif

    /* 按功能块聚合的子结构体 */
    foc_outer_loop_state_t outer_loop_state;
    foc_mode_transition_t  mode_transition;
    foc_svpwm_lpf_state_t  svpwm_lpf;

} foc_motor_t;

#endif /* FOC_CTRL_TYPES_H */
