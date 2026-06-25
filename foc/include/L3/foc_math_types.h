#ifndef FOC_MATH_TYPES_H
#define FOC_MATH_TYPES_H

#include <stdint.h>

/* ========== Kalman filter state ========== */
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

/* ========== PID controller ========== */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} foc_pid_t;

#endif /* FOC_MATH_TYPES_H */
