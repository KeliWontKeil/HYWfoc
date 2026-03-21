#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include <stdint.h>
#include "math_transforms.h"

#define FOC_TWO_PI 6.2831852f
#define FOC_CALIB_SETTLE_MS 4U
#define FOC_CALIB_MIN_MECH_STEP_RAD 0.0015f
#define FOC_CALIB_LOCK_SETTLE_MS 60U
#define FOC_CALIB_LOCK_SAMPLE_COUNT 24U
#define FOC_CALIB_STEP_SETTLE_MS 8U
#define FOC_CALIB_STEP_SAMPLE_COUNT 6U
#define FOC_CALIB_STEP_ELEC_RAD (FOC_TWO_PI / 10.0f)
#define FOC_CALIB_STEP_COUNT 20U

#define FOC_DIR_UNDEFINED 0U
#define FOC_DIR_NORMAL 1U
#define FOC_DIR_REVERSED 2U
#define FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED (-1.0f)
#define FOC_POLE_PAIRS_UNDEFINED 0U

typedef struct {
    uint8_t pole_pairs;
    float mech_angle_at_elec_zero_rad;
    uint8_t direction; /* 1: normal, 2: reversed, 0: undefined */
    float vbus_voltage;

    /* Open-loop targets */
    float electrical_phase_angle;
    float ud;
    float uq;
    float set_voltage;

    /* Intermediate and output states */
    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} foc_motor_t;

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   uint8_t direction);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);
void FOC_OpenLoopStep(foc_motor_t *motor, float dt_sec);

#endif /* _FOC_CONTROL_H_ */
