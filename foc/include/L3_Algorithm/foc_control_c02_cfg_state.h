#ifndef FOC_CONTROL_C02_CFG_STATE_H
#define FOC_CONTROL_C02_CFG_STATE_H

#include <stdint.h>

#include "L3_Algorithm/foc_control_types.h"
#include "LS_Config/foc_shared_types.h"

void FOC_ControlConfigResetDefault(void);
const foc_control_runtime_config_t *FOC_ControlGetRuntimeConfig(void);
void FOC_ControlSetMinMechAngleAccumDeltaRad(float value);
void FOC_ControlSetAngleHoldIntegralLimit(float value);
void FOC_ControlSetAngleHoldPidDeadbandRad(float value);
void FOC_ControlSetSpeedAngleTransitionStartRad(float value);
void FOC_ControlSetSpeedAngleTransitionEndRad(float value);

void FOC_ControlSetCurrentSoftSwitchEnable(uint8_t enable);
void FOC_ControlSetCurrentSoftSwitchMode(uint8_t mode);
void FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(float value);
void FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(float value);
const foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatus(void);
void FOC_ControlResetCurrentSoftSwitchState(void);

void FOC_ControlSetCoggingCompEnable(uint8_t enable);
uint8_t FOC_ControlLoadCoggingCompTableQ15(const int16_t *table_q15,
                                           uint16_t point_count,
                                           float iq_lsb_a,
                                           uint8_t source);
void FOC_ControlSetCoggingCompUnavailable(uint8_t source);
const foc_cogging_comp_status_t *FOC_ControlGetCoggingCompStatus(void);
uint8_t FOC_ControlReadCoggingCompTableQ15(const int16_t **table_q15,
                                           uint16_t *point_count,
                                           float *iq_lsb_a);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

#endif /* FOC_CONTROL_C02_CFG_STATE_H */
