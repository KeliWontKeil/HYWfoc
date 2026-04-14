#ifndef _FOC_CONTROL_INIT_H_
#define _FOC_CONTROL_INIT_H_

#include <stdint.h>

#include "config/foc_shared_types.h"
#include "algorithm/math_transforms.h"

#define FOC_CALIB_SETTLE_MS 4U
#define FOC_CALIB_MIN_MECH_STEP_RAD 0.0015f
#define FOC_CALIB_LOCK_SETTLE_MS 60U
#define FOC_CALIB_LOCK_SAMPLE_COUNT 24U
/* Legacy aliases retained for compatibility with existing call sites. */
#define FOC_CALIB_STEP_SETTLE_MS FOC_CALIB_FINE_STEP_SETTLE_MS
#define FOC_CALIB_STEP_SAMPLE_COUNT FOC_CALIB_FINE_STEP_SAMPLE_COUNT
#define FOC_CALIB_STEP_ELEC_RAD FOC_CALIB_FINE_STEP_ELEC_RAD
#define FOC_CALIB_STEP_COUNT FOC_CALIB_FINE_STEP_COUNT

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);

#endif /* _FOC_CONTROL_INIT_H_ */
