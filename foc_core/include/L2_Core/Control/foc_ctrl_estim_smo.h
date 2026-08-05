#ifndef FOC_CTRL_ESTIM_SMO_H
#define FOC_CTRL_ESTIM_SMO_H

#include <stdint.h>

#include "L2_Core/Control/foc_ctrl_estim.h"
#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_filter_types.h"

typedef struct foc_motor_t foc_motor_t;

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

    /* 收敛/方向判定（两路径共用） */
    uint16_t converge_counter;
    uint16_t lock_counter;
    uint16_t rot_dir_counter;
    uint8_t  initialized;
    uint8_t  rot_dir_last;
    uint8_t  converged_once;

    /* 测速链（两路径共用） */
    FOC_FILTER_TYPEDEF(FOC_FILTER_SMO_SPEED) smo_speed_filter;
    float    pll_angle_history[FOC_SMO_ANGLE_HISTORY_SIZE];
    uint8_t  angle_history_idx;
    float    speed_window[FOC_SMO_SPEED_WINDOW_SIZE];
    uint8_t  speed_window_pos;
    uint8_t  speed_window_count;
} foc_estim_smo_state_t;

void FOC_EstimSMO_Step(foc_motor_t *motor, float dt_sec);
void FOC_EstimSMO_Init(foc_motor_t *motor);
#endif

#endif /* FOC_CTRL_ESTIM_SMO_H */