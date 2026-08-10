#ifndef FOC_CTRL_OPENLOOP_H
#define FOC_CTRL_OPENLOOP_H

#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"

#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
/* ========== OpenLoop angle source private state ========== */
#define FOC_OPENLOOP_STATE_IDLE      0U
#define FOC_OPENLOOP_STATE_RUNNING   1U
#define FOC_OPENLOOP_STATE_DONE      2U
#define FOC_OPENLOOP_STATE_FAILED    3U

typedef struct {
    uint8_t  phase;
    float    virtual_angle_rad;
    float    virtual_speed_rad_s;
    float    ramp_rate_rad_s2;
    float    target_speed_rad_s;
    float    mech_speed_rad_s;
} foc_openloop_state_t;

void FOC_OpenLoop_Init(foc_openloop_state_t *state,
                       const foc_motor_params_t *params,
                       const foc_control_cfg_t *cfg);

uint8_t FOC_OpenLoop_RunStep(foc_openloop_state_t *state,
                             foc_control_runtime_t *ctrl,
                             const foc_motor_params_t *params,
                             const foc_control_cfg_t *cfg,
                             float dt_sec,
                             float *mech_speed_rad_s);
#endif

#endif /* FOC_CTRL_OPENLOOP_H */
