#ifndef FOC_MOTOR_TYPES_H
#define FOC_MOTOR_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "LS_Config/foc_math_types.h"

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
    uint8_t system_running;              /* 1=正常运行 */
    uint8_t system_fault;                /* 1=故障状态 */
    uint8_t reinit_pending;              /* 1=重初始化请求 */
    uint8_t last_fault_code;             /* foc_fault_code_t */
    uint8_t cfg_dirty;                   /* 1=配置已变更，需要L1应用 */
    uint8_t motor_enabled;               /* 1=电机使能 */
    uint8_t control_mode;                /* 控制模式 */
    uint8_t pending_system_action;       /* 待处理系统命令，L1消费 */
    uint16_t init_check_mask;            /* 已执行初始化检查掩码 */
    uint16_t init_fail_mask;             /* 失败初始化检查掩码 */
    uint16_t sensor_invalid_consecutive; /* 连续无效传感器计数 */
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
} foc_motor_state_t;

/* ========== 控制配置参数 ========== */
typedef struct {
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;
    uint8_t control_mode;

    /* PID 参数 */
    float pid_current_kp;
    float pid_current_ki;
    float pid_current_kd;
    float pid_angle_kp;
    float pid_angle_ki;
    float pid_angle_kd;
    float pid_speed_kp;
    float pid_speed_ki;
    float pid_speed_kd;

    /* Fine-tuning 参数 */
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;

    /* 电流软切换 */
    uint8_t current_soft_switch_enable;
    uint8_t current_soft_switch_mode;
    float current_soft_switch_auto_open_iq_a;
    float current_soft_switch_auto_closed_iq_a;

    /* 齿槽补偿 */
    uint8_t cogging_comp_enable;
    float cogging_comp_iq_limit_a;
    float cogging_comp_speed_gate_rad_s;
    float cogging_calib_gain_k;
} foc_motor_cfg_t;

/* ========== 系统动作枚举（pending_system_action） ========== */
#define FOC_SYSACTION_NONE           0U
#define FOC_SYSACTION_COGGING_START  1U
#define FOC_SYSACTION_COGGING_DUMP   2U
#define FOC_SYSACTION_COGGING_EXPORT 3U

/* ========== Runtime control configuration ========== */
typedef struct {
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_runtime_config_t;

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

    /* Runtime calibration gain K (Δθ→iq conversion factor). */
    float calib_gain_k;
} foc_cogging_comp_status_t;

/* Cogging calibration runtime state */
typedef struct {
    /* === Calibration state machine === */
    uint8_t in_progress;           /* 1 when calibration is active */
    uint8_t progress_percent;      /* 0–100 progress indicator */
    uint16_t point_index;          /* CALIB_PHASE_* or scan-phase index */

    /* Running-average pass count.  Replaces per-bin float[512] and int16_t[512]. */
    uint8_t completed_pass_count;

    /* === Predicted angle state (pure integrator, never re-seeded) === */
    float pred_mech_angle;
    float travel_accum_rad;
    float angle_prev_rad;

    /* === Per-pass tracking === */
    uint16_t last_lut_index;
    uint16_t bins_collected;
    uint8_t pass_num;
    uint8_t rev_count;
    uint8_t last_reported_progress;

    /* === Saved soft-switch state (restored on calibration finish) === */
    uint8_t saved_softswitch_enabled;
    uint8_t saved_softswitch_mode;

    /* === Calibration request flags (set by L1, consumed by SampleStep) === */
    uint8_t request_start;
    uint8_t request_dump;
    uint8_t request_export;
} foc_cogging_calib_state_t;

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
typedef struct {
    /* === 运行时状态 === */
    foc_motor_state_t state;
    /* === 控制配置 === */
    foc_motor_cfg_t cfg;
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

    /* Alpha-beta, three-phase voltages, and SVPWM outputs. */
    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;

    /* Control runtime context migrated from shared C25 singletons. */
    foc_control_runtime_config_t control_runtime_cfg;
    foc_current_soft_switch_status_t current_soft_switch_status;
    uint8_t current_soft_switch_blend_initialized;

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    float    ecycle_offset_dyn_a;
    float    ecycle_offset_dyn_b;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    float    ecycle_offset_dyn_c;
#endif
    float    ecycle_prev_mech_angle;
    float    ecycle_accu_mech_delta;
    uint16_t ecycle_sample_count;
    float    ecycle_accum_a;
    float    ecycle_accum_b;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
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
    /* Placeholder to keep struct size stable when COMP is disabled. */
    uint8_t _cogging_padding;
#endif
} foc_motor_t;

#endif /* FOC_MOTOR_TYPES_H */