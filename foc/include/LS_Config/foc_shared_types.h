#ifndef FOC_SHARED_TYPES_H
#define FOC_SHARED_TYPES_H

#include <stdint.h>

/* Unified mathematical constants: single source of truth for the project. */
#define FOC_MATH_PI 3.1415926f
#define FOC_MATH_TWO_PI 6.2831852f
#define FOC_MATH_SQRT3 1.7320508f
#define FOC_MATH_SQRT3_BY_2 0.8660254f
#define FOC_MATH_PI_BY_3 1.0471975f
#define FOC_MATH_EPSILON 1e-6f

#define FOC_DIR_UNDEFINED 0
#define FOC_DIR_NORMAL 1
#define FOC_DIR_REVERSED -1

#define FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED (-1.0f)

#define FOC_POLE_PAIRS_UNDEFINED 0U

typedef enum {
    FOC_TASK_RATE_FAST_CONTROL = 0,
    FOC_TASK_RATE_SERVICE,
    FOC_TASK_RATE_MONITOR,
    FOC_TASK_RATE_HEARTBEAT,
    FOC_TASK_RATE_COUNT
} FOC_TaskRate_t;

typedef struct {
    float raw_value;
    float filtered_value;
    float kalman_gain;
    float estimate_error;
    float measurement_error;
    float process_noise;
    float zero_offset;
    float output_value;
} kalman_filter_t;

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

/* Motor-side cogging LUT capacity used by runtime context storage. */
#ifndef FOC_MOTOR_COGGING_LUT_CAPACITY
#define FOC_MOTOR_COGGING_LUT_CAPACITY 128U
#endif

typedef struct {
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_runtime_config_t;

typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
} foc_current_soft_switch_status_t;

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

    /* Calibration state machine fields (used when FOC_COGGING_CALIB_ENABLE). */
    uint8_t calib_in_progress;
    uint8_t calib_progress_percent;
    uint16_t calib_point_index;
    uint8_t calib_pass_index;
} foc_cogging_comp_status_t;

typedef struct {
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
    foc_cogging_comp_status_t cogging_comp_status;
    int16_t cogging_comp_table_q15[FOC_MOTOR_COGGING_LUT_CAPACITY];
} foc_motor_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} foc_pid_t;

typedef enum {
    FOC_TORQUE_MODE_OPEN_LOOP = 0,
    FOC_TORQUE_MODE_CURRENT_PID = 1
} foc_torque_mode_t;

#endif /* FOC_SHARED_TYPES_H */
