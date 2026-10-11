#ifndef FOC_CTRL_ESTIM_HFI_H
#define FOC_CTRL_ESTIM_HFI_H

#include <stdint.h>

#include "L2_Core/Control/foc_ctrl_estim.h"

#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
/* ========== HFI 估计器私有状态 ==========
 * 预留：HFI 状态机尚未实现（当前源配置下本模块不编译）。
 * 解调结果唯一来源为注入模块 `foc_injection_state_t.demod`——估计器接入时消费该结果，
 * 不在本结构内重复实现解调（避免双份解调状态与相位基准分叉）。
 * 载波参数（轴/频率/幅值）由本域决定：见 `FOC_HFI_INJECT_AXIS` / `FOC_HFI_DEFAULT_FREQ_HZ` /
 * `FOC_HFI_DEFAULT_AMPLITUDE_V`（foc_cfg_init_values.h），启用前调用 `FOC_Injection_Configure`。
 * **参考角亦由本域给出**（估计器自己的坐标帧，且须与解调帧一致）；接入时不要依赖声学路径的取角策略。 */
typedef struct {
    float    hf_sin_demod;
    float    hf_cos_demod;
} foc_estim_hfi_state_t;

void FOC_EstimHFI_Step(foc_estim_hfi_state_t *state, float dt_sec);
void FOC_EstimHFI_Init(foc_estim_hfi_state_t *state);
#endif

#endif /* FOC_CTRL_ESTIM_HFI_H */
