#ifndef FOC_FILTER_TYPES_H
#define FOC_FILTER_TYPES_H

#include <stdint.h>

/* 一阶低通滤波器状态 */
typedef struct {
    float raw_value;
    float state;
    float output_value;
    uint8_t valid;
} foc_filter_lpf1_t;

/* 双二阶 IIR 滤波器状态（骨架，算法未实现） */
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    float output_value;
    uint8_t valid;
} foc_filter_biquad_t;

/* NONE 配置下的占位类型，仅保留 output_value 供外部访问 */
typedef struct {
    float output_value;
} foc_filter_none_t;

/* 卡尔曼滤波器状态 */
typedef struct {
    float raw_value;
    float filtered_value;
    float kalman_gain;
    float estimate_error;
    float measurement_error;
    float process_noise;
    float zero_offset;
    float output_value;
} foc_filter_kalman_t;

#endif /* FOC_FILTER_TYPES_H */
