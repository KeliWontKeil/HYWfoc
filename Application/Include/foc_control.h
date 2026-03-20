#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include <stdint.h>
#include "math_transforms.h"

typedef struct {
    uint8_t pole_pairs;
    float zero_electrical_angle;
    uint8_t direction;//1 for normal,2 for reversed,0 for undefined
    float vbus_voltage;

    /* Open-loop targets */
    float electrical_angle;
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
                    float zero_electrical_angle,
                    uint8_t direction);
void FOC_OpenLoopStep(foc_motor_t *motor, float dt_sec);

#endif /* _FOC_CONTROL_H_ */
