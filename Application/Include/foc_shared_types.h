#ifndef FOC_SHARED_TYPES_H
#define FOC_SHARED_TYPES_H

#include <stdint.h>

#define FOC_SCHEDULER_CALLBACK_TYPE(name) void (*name)(void)
typedef FOC_SCHEDULER_CALLBACK_TYPE(FOC_SchedulerCallback_t);

typedef enum {
    FOC_SCHEDULER_RATE_CONTROL_1KHZ = 0,
    FOC_SCHEDULER_RATE_SERVICE_100HZ,
    FOC_SCHEDULER_RATE_MONITOR_10HZ,
    FOC_SCHEDULER_RATE_HEARTBEAT_1HZ,
    FOC_SCHEDULER_RATE_COUNT
} FOC_SchedulerRate_t;

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
} sensor_data_t;

typedef struct {
    float phase_resistance;
    uint8_t pole_pairs;
    float mech_angle_at_elec_zero_rad;
    int8_t direction;
    float vbus_voltage;

    float electrical_phase_angle;
    float ud;
    float uq;
    float set_voltage;

    float iq_target;
    float iq_measured;

    float mech_angle_accum_rad;
    float mech_angle_prev_rad;
    uint8_t mech_angle_prev_valid;
    int32_t mech_turn_count;

    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
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
