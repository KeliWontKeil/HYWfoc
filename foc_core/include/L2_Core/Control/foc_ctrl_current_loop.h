#ifndef FOC_CTRL_CURRENT_LOOP_H

#define FOC_CTRL_CURRENT_LOOP_H


#include <stdint.h>

#include "L3_Hal/foc_math_types.h"
#include "L2_Core/foc_ctrl_types.h"

typedef struct foc_motor_t foc_motor_t;

/* ========== Current soft-switch status（电流环域私有） ========== */
typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
    uint8_t blend_initialized;
    uint8_t prev_active_mode;
} foc_current_soft_switch_status_t;

void FOC_CurrentControlStep(foc_control_runtime_t *ctrl,
                            foc_pid_t *torque_pid,
                            foc_current_soft_switch_status_t *soft_switch,
                            const sensor_data_t *sensor,
                            const foc_motor_params_t *params,
                            float dt_sec);

uint8_t FOC_ControlRequiresCurrentSample(void);

#endif /* FOC_CTRL_CURRENT_LOOP_H */