#ifndef FOC_FILTER_MATH_H
#define FOC_FILTER_MATH_H

#include "L3_Hal/foc_filter_types.h"

void  FOC_FilterMath_KalmanInit(foc_filter_kalman_t *f, float meas_err,
                                float est_err, float proc_noise, float init_val);
float FOC_FilterMath_KalmanStep(foc_filter_kalman_t *f, float input);

void  FOC_FilterMath_KalmanAngleInit(foc_filter_kalman_t *f, float meas_err,
                                      float est_err, float proc_noise, float init_val);
float FOC_FilterMath_KalmanAngleStep(foc_filter_kalman_t *f, float input);
void  FOC_FilterMath_KalmanAngleReset(foc_filter_kalman_t *f, float init_val);

void  FOC_FilterMath_Lpf1Init(foc_filter_lpf1_t *f, float init_val);
float FOC_FilterMath_Lpf1Step(foc_filter_lpf1_t *f, float input, float alpha);
float FOC_FilterMath_Lpf1AngleStep(foc_filter_lpf1_t *f, float input, float alpha);
void  FOC_FilterMath_Lpf1Reset(foc_filter_lpf1_t *f, float init_val);

void  FOC_FilterMath_BiquadInit(foc_filter_biquad_t *f, const float *coeff);
float FOC_FilterMath_BiquadStep(foc_filter_biquad_t *f, float input);

#endif /* FOC_FILTER_MATH_H */
