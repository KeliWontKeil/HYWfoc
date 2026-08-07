#ifndef FOC_CTRL_PARAM_LEARN_H

#define FOC_CTRL_PARAM_LEARN_H


#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L3_Hal/foc_svpwm.h"

uint8_t FOC_EstimateDirectionAndPolePairs(foc_control_runtime_t *ctrl,
                                          const foc_motor_params_t *params,
                                          svpwm_interp_state_t *svpwm,
                                          foc_applied_output_state_t *applied,
                                          foc_alpha_beta_phase_t *alpha_beta,
                                          int8_t *direction_est,
                                          uint8_t *pole_pairs_est);


#endif /* FOC_CONTROL_C23_MOTOR_PARAM_LEARN_H */
