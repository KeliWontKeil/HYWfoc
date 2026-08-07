#ifndef FOC_FILTER_GATE_H
#define FOC_FILTER_GATE_H

#include "L3_Hal/foc_filter_math.h"
#include "LS_Config/foc_config.h"

static inline float FOC_FilterGate_CurrentA(
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_A) *f, float input)
{
#if (FOC_FILTER_SENSOR_CURRENT_A == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanStep(f, input);
#elif (FOC_FILTER_SENSOR_CURRENT_A == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1Step(f, input, FOC_FILTER_SENSOR_CURRENT_A_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

static inline float FOC_FilterGate_CurrentB(
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_B) *f, float input)
{
#if (FOC_FILTER_SENSOR_CURRENT_B == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanStep(f, input);
#elif (FOC_FILTER_SENSOR_CURRENT_B == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1Step(f, input, FOC_FILTER_SENSOR_CURRENT_B_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

static inline float FOC_FilterGate_CurrentC(
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_CURRENT_C) *f, float input)
{
#if (FOC_FILTER_SENSOR_CURRENT_C == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanStep(f, input);
#elif (FOC_FILTER_SENSOR_CURRENT_C == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1Step(f, input, FOC_FILTER_SENSOR_CURRENT_C_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

static inline float FOC_FilterGate_Angle(
    FOC_FILTER_TYPEDEF(FOC_FILTER_SENSOR_ANGLE) *f, float input)
{
#if (FOC_FILTER_SENSOR_ANGLE == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanAngleStep(f, input);
#elif (FOC_FILTER_SENSOR_ANGLE == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1AngleStep(f, input, FOC_FILTER_SENSOR_ANGLE_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

static inline float FOC_FilterGate_EncoderSpeed(
    FOC_FILTER_TYPEDEF(FOC_FILTER_ENCODER_SPEED) *f, float input)
{
#if (FOC_FILTER_ENCODER_SPEED == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanStep(f, input);
#elif (FOC_FILTER_ENCODER_SPEED == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1Step(f, input, FOC_FILTER_ENCODER_SPEED_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

static inline float FOC_FilterGate_SMOSpeed(
    FOC_FILTER_TYPEDEF(FOC_FILTER_SMO_SPEED) *f, float input)
{
#if (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_KALMAN)
    return FOC_FilterMath_KalmanStep(f, input);
#elif (FOC_FILTER_SMO_SPEED == FOC_FILTER_TYPE_LPF1)
    return FOC_FilterMath_Lpf1Step(f, input, FOC_FILTER_SMO_SPEED_LPF_ALPHA);
#else
    (void)f;
    return input;
#endif
}

#endif /* FOC_FILTER_GATE_H */
