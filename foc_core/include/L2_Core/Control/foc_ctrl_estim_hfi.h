#ifndef FOC_CTRL_ESTIM_HFI_H
#define FOC_CTRL_ESTIM_HFI_H

#include <stdint.h>

#include "L2_Core/Control/foc_ctrl_estim.h"

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
/* ========== HFI 估计器私有状态 ========== */
typedef struct {
    float    hf_sin_demod;
    float    hf_cos_demod;
} foc_estim_hfi_state_t;

void FOC_EstimHFI_Step(foc_estim_hfi_state_t *state, float dt_sec);
void FOC_EstimHFI_Init(foc_estim_hfi_state_t *state);
#endif

#endif /* FOC_CTRL_ESTIM_HFI_H */
