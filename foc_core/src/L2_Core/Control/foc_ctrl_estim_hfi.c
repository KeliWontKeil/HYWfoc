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
     * 预留（HFI 状态机未接入）：实现时在此读取注入解调结果
     * （motor 侧 foc_injection_state_t.demod）并做预收敛速度门限检查（参考 SMO 实现）。
     * 载波参数取本域默认值（FOC_HFI_INJECT_AXIS / FOC_HFI_DEFAULT_*），
     * 启用前调用 FOC_Injection_Configure 下发；门限值届时一并在 foc_cfg_init_values.h 补齐。
     */
}

#endif /* FOC_ESTIMATOR_HFI_ENABLE */
