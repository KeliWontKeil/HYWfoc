#ifndef FOC_CTRL_TYPES_H
#define FOC_CTRL_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_math_types.h"
#include "L3_Hal/foc_filter_types.h"

/* ========== SVPWM LPF filter state (three-phase) ========== */
typedef struct {
    uint8_t  valid;
    float    phase_a;
    float    phase_b;
    float    phase_c;
} foc_svpwm_lpf_state_t;

/* ========== Alpha-beta / three-phase voltage state ========== */
typedef struct {
    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
} foc_alpha_beta_phase_t;

/* ========== Source identifiers and published source view ========== */
typedef enum {
    FOC_SOURCE_TYPE_NONE     = FOC_CONTROL_SRC_NONE,
    FOC_SOURCE_TYPE_ENCODER  = FOC_CONTROL_SRC_ENCODER,
    FOC_SOURCE_TYPE_SMO      = FOC_CONTROL_SRC_SMO,
    FOC_SOURCE_TYPE_HFI      = FOC_CONTROL_SRC_HFI,
    FOC_SOURCE_TYPE_OPENLOOP = FOC_CONTROL_SRC_OPENLOOP,
    FOC_SOURCE_TYPE_FLUX     = FOC_CONTROL_SRC_FLUX
} foc_source_type_t;

typedef enum {
    FOC_SOURCE_STATE_INIT = 0U,
    FOC_SOURCE_STATE_CONVERGING = 1U,
    FOC_SOURCE_STATE_LOCKED = 2U,
    FOC_SOURCE_STATE_DIVERGED = 3U
} foc_source_state_t;

typedef struct {
    uint8_t  source;
    uint8_t  state;
    uint8_t  valid;
    float    confidence;

    float    elec_angle_rad;
    float    mech_angle_rad;
} foc_active_source_state_t;

typedef enum {
    FOC_CONTROL_REGION_LOW = 0U,
    FOC_CONTROL_REGION_HIGH = 1U,
    FOC_CONTROL_REGION_FULL = 2U
} foc_control_region_t;

typedef enum {
    FOC_REGION_STATE_FULL_ACTIVE = 0U,
    FOC_REGION_STATE_LOW_ACTIVE,
    FOC_REGION_STATE_HIGH_ACQUIRE,
    FOC_REGION_STATE_HIGH_READY,
    FOC_REGION_STATE_HIGH_ACTIVE,
    FOC_REGION_STATE_HIGH_SUSPECT,
    FOC_REGION_STATE_LOW_RECOVERY
} foc_region_state_t;

typedef struct {
    uint8_t comp_available;
    uint8_t comp_active;
    uint8_t calib_available;
    uint8_t reinit_available;
} foc_encoder_services_state_t;

typedef enum {
    FOC_PHASE_OUTPUT_IDLE = 0U,
    FOC_PHASE_OUTPUT_ZERO,
    FOC_PHASE_OUTPUT_DQ_VOLTAGE_ANGLE,
    FOC_PHASE_OUTPUT_DIRECT_DUTY
} foc_phase_output_type_t;

typedef struct {
    uint8_t phase;
    uint8_t type;
    uint8_t valid;
    uint8_t state_id;

    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} foc_phase_output_state_t;

typedef struct {
    uint8_t valid;
    float ud;
    float uq;
} foc_applied_output_state_t;

typedef struct {
    uint8_t active_source;
    uint8_t standby_source;
    uint8_t control_region;
    uint8_t region_state;
    uint8_t switch_in_progress;
    uint32_t switch_counter;
    uint8_t config_valid;
} foc_source_mgr_state_t;

/* ========== Outer-loop runtime state (private to outer_loop) ========== */
typedef struct {
    float  speed_err_accum_rad;
    float  prev_mech_signed_rad;
    uint8_t speed_state_valid;
    float  accum_rad;
    float  prev_rad;
    uint8_t prev_valid;

    float  ramped_speed_rad_s;      /* 加速器当前限幅后速度 */
} foc_outer_loop_private_t;

/* ========== Control mode transition tracking ========== */
typedef struct {
    uint8_t prev_control_mode;
    uint8_t prev_control_mode_valid;
    uint8_t prev_control_mode_check;
} foc_mode_transition_t;

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
    FOC_FAULT_INIT_FAILED = 6U,
    FOC_FAULT_ESTIMATOR_INVALID = 7U
} foc_fault_code_t;

/* ========== 控制阶段枚举 ========== */
typedef enum {
    FOC_CONTROL_PHASE_NORMAL        = 0U,
    FOC_CONTROL_PHASE_COGGING_CALIB = 1U,
    FOC_CONTROL_PHASE_REINIT        = 2U
} foc_control_phase_t;

/* ========== 运行时状态（per-motor） ========== */
typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t last_fault_code;
    uint8_t cfg_dirty;
    uint8_t motor_enabled;
    uint8_t control_mode;
    uint8_t control_phase;
    uint8_t current_loop_ready;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
} foc_motor_state_t;

/* ========== Current soft-switch status ========== */
typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
    uint8_t blend_initialized;
    uint8_t prev_active_mode;
} foc_current_soft_switch_status_t;

/* ========== Cogging compensation status ========== */
typedef struct {
    uint8_t enabled;
    uint8_t available;
    uint8_t source;
    uint16_t point_count;
    float iq_lsb_a;
    float speed_gate_rad_s;
    float speed_ref_rad_s;
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
    uint16_t settle_counter;
    uint16_t last_lut_index;
    uint16_t bins_collected;
    uint8_t pass_num;
    uint8_t last_reported_progress;
    uint8_t saved_softswitch_enabled;
    uint8_t saved_softswitch_mode;
    uint8_t request_start;
    uint8_t request_dump;
    uint8_t request_export;
} foc_cogging_calib_state_t;

/* ========== SMO 估计器私有状态 ========== */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    float    ialpha_est;
    float    ibeta_est;
    float    bemf_alpha;
    float    bemf_beta;
    float    z_alpha;
    float    z_beta;
    float    pll_angle_rad;
    float    pll_speed_rad_s;
    float    pll_integral;
    float    k_slide;
    float    phase_comp_rad;
    float    prev_z_alpha;
    float    prev_z_beta;
    float    mech_speed_rad_s;
    uint16_t converge_counter;
    uint16_t lock_counter;
    uint16_t rot_dir_counter;
    uint8_t  initialized;
    uint8_t  rot_dir_last;
} foc_estim_smo_state_t;
#endif

/* ========== HFI 估计器私有状态 ========== */
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
typedef struct {
    float    hf_sin_demod;
    float    hf_cos_demod;
} foc_estim_hfi_state_t;
#endif

/* ========== OpenLoop angle source private state ========== */
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
#define FOC_OPENLOOP_STATE_IDLE      0U
#define FOC_OPENLOOP_STATE_RUNNING   1U
#define FOC_OPENLOOP_STATE_DONE      2U
#define FOC_OPENLOOP_STATE_FAILED    3U

typedef struct {
    uint8_t  phase;
    float    virtual_angle_rad;
    float    virtual_speed_rad_s;
    float    ramp_rate_rad_s2;
    float    target_speed_rad_s;
    float    mech_speed_rad_s;
} foc_openloop_state_t;
#endif

/* ========== Source Manager low/high source switch private state ========== */
typedef struct {
    uint8_t  low_source;
    uint8_t  high_source;
    float    speed_threshold_high_rad_s;
    float    speed_threshold_low_rad_s;
} foc_source_switch_state_t;

/* ========== 非阻塞重初始化状态 ========== */
#define FOC_REINIT_PHASE_IDLE          0U
#define FOC_REINIT_PHASE_STOP          1U
#define FOC_REINIT_PHASE_ZERO_SAMPLE   2U
#define FOC_REINIT_PHASE_ZERO_CALC     3U
#define FOC_REINIT_PHASE_ALIGN_SETTLE  4U
#define FOC_REINIT_PHASE_ALIGN_SAMPLE  5U
#define FOC_REINIT_PHASE_ALIGN_CALC    6U
#define FOC_REINIT_PHASE_DIR_STEP      7U
#define FOC_REINIT_PHASE_DIR_SAMPLE    8U
#define FOC_REINIT_PHASE_DIR_CALC      9U
#define FOC_REINIT_PHASE_DIR_REV_STEP  10U
#define FOC_REINIT_PHASE_DIR_REV_SAMPLE 11U
#define FOC_REINIT_PHASE_FINALIZE      12U
#define FOC_REINIT_PHASE_DONE          13U

typedef struct {
    uint16_t phase;
    uint16_t settle_cycles;
    uint16_t sample_count;
    uint16_t sample_target;
    float    sin_sum;
    float    cos_sum;
    float    elec_angle_rad;
    float    calib_uq;
    float    prev_mech_rad;
    float    prev_elec_rad;
    float    sum_d_mech;
    float    sum_d_elec;
    uint8_t  has_prev;
    uint8_t  step_index;
    uint8_t  step_count;
    uint8_t  reverse_pass;
} foc_reinit_state_t;

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
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_A) current_a;
    float current_a_zero_offset;
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_B) current_b;
    float current_b_zero_offset;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_C) current_c;
    float current_c_zero_offset;
#endif
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_ANGLE)     mech_angle_rad;
    float prev_mech_angle_rad;
    float mech_speed_rad_s;
    uint8_t mech_speed_valid;
    struct {
        float raw;
        float filtered;
    } vbus;
    uint8_t adc_valid;
    uint8_t encoder_valid;
    uint8_t vbus_valid;
} sensor_data_t;

/* ========== D/Q 控制运行时 ========== */
typedef struct {
    float electrical_angle_rad;
    float ud;
    float uq;
    float max_phase_voltage;
    float iq_target;
    float iq_measured;
} foc_control_runtime_t;

/* ========== 电机物理参数 ========== */
typedef struct {
    float phase_resistance;
    float stator_inductance;
    uint8_t pole_pairs;
    float mech_angle_at_elec_zero_rad;
    int8_t direction;
    float vbus_voltage;
} foc_motor_params_t;

/* ========== 控制目标与调优配置 ========== */
typedef struct {
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_cfg_t;

/* ========== ISR 测速 ========== */
typedef struct {
    uint8_t  fast_current_div_counter;
    uint32_t current_loop_cycles;
} foc_isr_timing_t;

/* ========== Torque mode ========== */
typedef enum {
    FOC_TORQUE_MODE_OPEN_LOOP = 0,
    FOC_TORQUE_MODE_CURRENT_PID = 1
} foc_torque_mode_t;

/* ========== Motor aggregate state ========== */
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
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    foc_cogging_calib_state_t cogging_calib_state;
#endif
#endif
#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)
    foc_reinit_state_t reinit_state;
#endif
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
    foc_svpwm_lpf_state_t svpwm_lpf;
#endif

} foc_motor_t;

#endif /* FOC_CTRL_TYPES_H */
