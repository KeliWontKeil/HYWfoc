#ifndef FOC_CTRL_ESTIM_SMO_H
#define FOC_CTRL_ESTIM_SMO_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "L2_Core/Control/foc_ctrl_estim.h"
#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_filter_types.h"
#include "L3_Hal/foc_sensor.h"

#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
/* ========== SMO 估计器私有状态 ========== */
typedef struct {
    /* 观测器核心（两路径共用） */
    float    ialpha_est;
    float    ibeta_est;
    float    bemf_alpha;
    float    bemf_beta;
    float    z_alpha;
    float    z_beta;
    float    k_slide;

    /* 角度/速度输出（两路径共用） */
    float    pll_angle_rad;
    float    pll_speed_rad_s;
    float    mech_speed_rad_s;

#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_PLL)
    float    pll_integral;
#endif
#if (FOC_ESTIM_SMO_ANGLE_METHOD == FOC_ESTIM_SMO_ANGLE_METHOD_LPF_ATAN2)
    float    phase_comp_rad;
#endif

    /* 收敛/失锁判定（两路径共用） */
    uint8_t  initialized;
    uint8_t  converged;
    uint16_t lost_count;

    /* 测速链（两路径共用）：速度直接取自 PLL/ATAN2 自带电速，经最终 LPF 平滑 */
    FOC_FILTER_TYPEDEF(FOC_FILTER_SMO_SPEED) smo_speed_filter;
} foc_estim_smo_state_t;

void FOC_EstimSMO_Step(foc_estim_smo_state_t *state,
                       const foc_motor_params_t *params,
                       const sensor_data_t *sensor,
                       const foc_applied_output_state_t *applied,
                       const foc_alpha_beta_phase_t *alpha_beta,
                       const foc_control_runtime_t *ctrl,
                       uint8_t active_source,
                       uint8_t control_region,
                       float speed_cmd_rad_s,
                       float dt_sec);
void FOC_EstimSMO_Init(foc_estim_smo_state_t *state, const foc_motor_params_t *params);
#endif

#endif /* FOC_CTRL_ESTIM_SMO_H */
