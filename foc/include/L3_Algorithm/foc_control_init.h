#ifndef _FOC_CONTROL_INIT_H_
#define _FOC_CONTROL_INIT_H_

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);

#endif /* _FOC_CONTROL_INIT_H_ */
