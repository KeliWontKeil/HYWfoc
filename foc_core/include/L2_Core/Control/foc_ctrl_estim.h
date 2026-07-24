#ifndef FOC_CTRL_ESTIM_H
#define FOC_CTRL_ESTIM_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

/* 估计器类型 */
#define FOC_ESTIMATOR_TYPE_NONE     0U
#define FOC_ESTIMATOR_TYPE_ENCODER  1U
#define FOC_ESTIMATOR_TYPE_SMO      2U
#define FOC_ESTIMATOR_TYPE_HFI      3U
#define FOC_ESTIMATOR_TYPE_FLUX     4U
#define FOC_ESTIMATOR_TYPE_BLEND    5U

/* 估计器状态 */
#define FOC_ESTIMATOR_STATE_INIT        0U
#define FOC_ESTIMATOR_STATE_CONVERGING  1U
#define FOC_ESTIMATOR_STATE_LOCKED      2U
#define FOC_ESTIMATOR_STATE_DIVERGED    3U

void FOC_Estimator_Select(foc_motor_t *motor, uint8_t estimator_type);

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimEncoder_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimEncoder_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimSMO_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimSMO_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimHFI_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimHFI_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_FLUX_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimFlux_Step(foc_motor_t *motor, foc_est_state_t *out, float dt_sec);
void FOC_EstimFlux_Init(foc_motor_t *motor);
#endif

#endif /* FOC_CTRL_ESTIM_H */
