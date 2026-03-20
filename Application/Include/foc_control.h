#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include "gd32f30x.h"
#include "math_transforms.h"

typedef struct {
    /* Open-loop targets */
    float electrical_angle;
    float electrical_speed_hz;
    float ud;
    float uq;
    float set_voltage;
    float vbus_voltage;

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
                   float electrical_speed_hz,
                   float ud,
                   float uq);
void FOC_OpenLoopStep(foc_motor_t *motor, float dt_sec);

#endif /* _FOC_CONTROL_H_ */
