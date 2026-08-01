#ifndef FOC_SVPWM_H

#define FOC_SVPWM_H


#include <stdint.h>

#include "L2_Core/foc_ctrl_types.h"
#include "LS_Config/foc_config.h"

/*
 * SVPWM API — all functions require a foc_motor_t pointer for per-motor
 * interpolation state, eliminating static globals.
 */
void SVPWM_Init(foc_motor_t *motor, uint16_t freq_kHz, uint8_t deadtime_percent);
void SVPWM_Update(foc_motor_t *motor,
                  float phase_a,
                  float phase_b,
                  float phase_c,
                  float voltage_command,
                  float vbus_voltage,
                  uint8_t direct_output);
void SVPWM_ApplyDirectDuty(foc_motor_t *motor,
                           uint8_t sector,
                           float duty_a,
                           float duty_b,
                           float duty_c);
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
void SVPWM_SetRuntimeDutyTarget(foc_motor_t *motor,
                                uint8_t sector,
                                float duty_a,
                                float duty_b,
                                float duty_c);
void SVPWM_InterpolationISR(foc_motor_t *motor);
#endif
const svpwm_output_t* SVPWM_GetOutput(const foc_motor_t *motor);

#endif /* FOC_SVPWM_H */
