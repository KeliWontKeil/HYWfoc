#ifndef _SVPWM_H_
#define _SVPWM_H_

#include <stdint.h>

typedef struct {
    float set_voltage;
    float alpha;
    float beta;
} svpwm_input_t;

typedef struct {
    uint8_t sector;
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_output_t;

void SVPWM_Init(float vbus_voltage);
void SVPWM_SetBusVoltage(float vbus_voltage);
void SVPWM_Update(const svpwm_input_t *input);
const svpwm_output_t* SVPWM_GetOutput(void);

#endif /* _SVPWM_H_ */
