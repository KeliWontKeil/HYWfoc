#ifndef _FOC_CONTROL_INIT_H_
#define _FOC_CONTROL_INIT_H_

#include <stdint.h>

#include "foc_shared_types.h"
#include "math_transforms.h"
#include "foc_control_internal.h"
#include "foc_platform_api.h"

#define FOC_CALIB_SETTLE_MS 4U
#define FOC_CALIB_MIN_MECH_STEP_RAD 0.0015f
#define FOC_CALIB_LOCK_SETTLE_MS 60U
#define FOC_CALIB_LOCK_SAMPLE_COUNT 24U
#define FOC_CALIB_STEP_SETTLE_MS 8U
#define FOC_CALIB_STEP_SAMPLE_COUNT 6U
#define FOC_CALIB_STEP_ELEC_RAD (MATH_TWO_PI / 20.0f)
#define FOC_CALIB_STEP_COUNT 20U

#define FOC_DIR_UNDEFINED 0
#define FOC_DIR_NORMAL 1
#define FOC_DIR_REVERSED -1
#define FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED (-1.0f)
#define FOC_POLE_PAIRS_UNDEFINED 0U

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);

#endif /* _FOC_CONTROL_INIT_H_ */
