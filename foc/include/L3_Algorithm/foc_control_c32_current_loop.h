#ifndef FOC_CONTROL_C32_CURRENT_LOOP_H
#define FOC_CONTROL_C32_CURRENT_LOOP_H

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

void FOC_CurrentControlStep(foc_motor_t *motor,
                            foc_pid_t *current_pid,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec);

uint8_t FOC_ControlRequiresCurrentSample(void);

#endif /* FOC_CONTROL_C32_CURRENT_LOOP_H */
