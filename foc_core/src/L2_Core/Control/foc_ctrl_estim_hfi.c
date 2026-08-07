#include "L2_Core/Control/foc_ctrl_estim_hfi.h"

#include "LS_Config/foc_config.h"

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)

void FOC_EstimHFI_Init(foc_estim_hfi_state_t *state)
{
    state->hf_sin_demod = 0.0f;
    state->hf_cos_demod = 0.0f;
}

void FOC_EstimHFI_Step(foc_estim_hfi_state_t *state, float dt_sec)
{
    (void)dt_sec;

    /*
     * TODO: 预收敛速度门限检查（参考 SMO 实现），在支持收敛状态机后引入。
     *   FOC_HFI_ACCEL_PRECONV_SPEED_THRESHOLD_RAD_S
     */
}

#endif /* FOC_ESTIMATOR_HFI_ENABLE */
