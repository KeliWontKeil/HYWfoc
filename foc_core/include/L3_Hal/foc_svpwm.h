#ifndef FOC_SVPWM_H

#define FOC_SVPWM_H


#include <stdint.h>

#include "LS_Config/foc_config.h"

/* ========== SVPWM output snapshot type ========== */
typedef struct {
    uint8_t sector;
    float duty_a;
    float duty_b;
    float duty_c;
} svpwm_output_t;

/* ========== SVPWM 插值引擎状态 ========== */
typedef struct {
    svpwm_output_t output;
    float duty_a_current;
    float duty_b_current;
    float duty_c_current;
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    float duty_a_target;
    float duty_b_target;
    float duty_c_target;
    float duty_a_step;
    float duty_b_step;
    float duty_c_step;
    uint16_t interp_steps_total;
    uint16_t interp_step_index;
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    /* 三 ISR 模式双写者并发保护：写端(电流环 ISR)写 pending，PWM ISR 入口原子取走 */
    volatile uint8_t target_pending;
    float pending_duty_a_target;
    float pending_duty_b_target;
    float pending_duty_c_target;
    float pending_duty_a_step;
    float pending_duty_b_step;
    float pending_duty_c_step;
#endif
#endif
} svpwm_interp_state_t;

/*
 * SVPWM API
 */
void SVPWM_Init(svpwm_interp_state_t *svpwm, uint16_t freq_kHz, uint8_t deadtime_percent);
void SVPWM_Update(svpwm_interp_state_t *svpwm,
                  float phase_a,
                  float phase_b,
                  float phase_c,
                  float voltage_command,
                  float vbus_voltage,
                  uint8_t direct_output);
void SVPWM_ApplyDirectDuty(svpwm_interp_state_t *svpwm,
                           uint8_t sector,
                           float duty_a,
                           float duty_b,
                           float duty_c);
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
void SVPWM_SetRuntimeDutyTarget(svpwm_interp_state_t *svpwm,
                                uint8_t sector,
                                float duty_a,
                                float duty_b,
                                float duty_c);
void SVPWM_InterpolationISR(svpwm_interp_state_t *svpwm);
#endif
const svpwm_output_t* SVPWM_GetOutput(const svpwm_interp_state_t *svpwm);

#endif /* FOC_SVPWM_H */
