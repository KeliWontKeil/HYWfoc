#ifndef FOC_CONTROL_C12_INIT_H
#define FOC_CONTROL_C12_INIT_H

#include <stdint.h>

#include "L2/foc_ctrl_types.h"

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);

/* Initialise sensor, SVPWM, and fast control-executor hardware.
 * L1 calls this once during boot (or after reinit) instead of calling
 * Sensor_*, SVPWM_*, FOC_ControlExecutor_Init directly. */
void FOC_ControlPlatform_InitHardware(foc_motor_t *motor);

#endif /* FOC_CONTROL_C12_INIT_H */
