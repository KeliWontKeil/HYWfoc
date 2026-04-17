#ifndef _SVPWM_IFACE_H_
#define _SVPWM_IFACE_H_

#include <stdint.h>

typedef struct {
    uint8_t sector;
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_output_t;

void SVPWM_Init(uint16_t freq_kHz, uint8_t deadtime_percent);
void SVPWM_UpdateRuntime(float phase_a,
                         float phase_b,
                         float phase_c,
                         float voltage_command,
                         float vbus_voltage,
                         uint8_t *sector,
                         float *duty_a,
                         float *duty_b,
                         float *duty_c);
void SVPWM_UpdateDirect(float phase_a,
                        float phase_b,
                        float phase_c,
                        float voltage_command,
                        float vbus_voltage,
                        uint8_t *sector,
                        float *duty_a,
                        float *duty_b,
                        float *duty_c);
void SVPWM_SetRuntimeDutyTarget(uint8_t sector,
                                float duty_a,
                                float duty_b,
                                float duty_c);
void SVPWM_ApplyDirectDuty(uint8_t sector,
                           float duty_a,
                           float duty_b,
                           float duty_c);
void SVPWM_InterpolationISR(void);
const svpwm_output_t* SVPWM_GetOutput(void);

#endif /* _SVPWM_IFACE_H_ */
