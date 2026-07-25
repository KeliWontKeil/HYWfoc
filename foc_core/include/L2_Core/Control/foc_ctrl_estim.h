#ifndef FOC_CTRL_ESTIM_H
#define FOC_CTRL_ESTIM_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

/* Estimator states share the Source state value space. */
#define FOC_ESTIMATOR_STATE_INIT        FOC_SOURCE_STATE_INIT
#define FOC_ESTIMATOR_STATE_CONVERGING  FOC_SOURCE_STATE_CONVERGING
#define FOC_ESTIMATOR_STATE_LOCKED      FOC_SOURCE_STATE_LOCKED
#define FOC_ESTIMATOR_STATE_DIVERGED    FOC_SOURCE_STATE_DIVERGED

#if (FOC_ESTIMATOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimEncoder_Step(foc_motor_t *motor, float dt_sec);
void FOC_EstimEncoder_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimSMO_Step(foc_motor_t *motor, float dt_sec);
void FOC_EstimSMO_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimHFI_Step(foc_motor_t *motor, float dt_sec);
void FOC_EstimHFI_Init(foc_motor_t *motor);
#endif

#if (FOC_ESTIMATOR_FLUX_ENABLE == FOC_CFG_ENABLE)
void FOC_EstimFlux_Step(foc_motor_t *motor, float dt_sec);
void FOC_EstimFlux_Init(foc_motor_t *motor);
#endif

#endif /* FOC_CTRL_ESTIM_H */
