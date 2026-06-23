#ifndef FOC_CONTROL_C13_CFG_STATE_H
#define FOC_CONTROL_C13_CFG_STATE_H

#include <stdint.h>

#include "LS_Config/foc_math_types.h"
#include "LS_Config/foc_motor_types.h"

void FOC_ControlConfigResetDefault(foc_motor_t *motor);
const foc_control_runtime_config_t *FOC_ControlGetRuntimeConfig(const foc_motor_t *motor);
void FOC_ControlSetMinMechAngleAccumDeltaRad(foc_motor_t *motor, float value);
void FOC_ControlSetAngleHoldIntegralLimit(foc_motor_t *motor, float value);
void FOC_ControlSetAngleHoldPidDeadbandRad(foc_motor_t *motor, float value);
void FOC_ControlSetSpeedAngleTransitionStartRad(foc_motor_t *motor, float value);
void FOC_ControlSetSpeedAngleTransitionEndRad(foc_motor_t *motor, float value);

void FOC_ControlSetCurrentSoftSwitchEnable(foc_motor_t *motor, uint8_t enable);
void FOC_ControlSetCurrentSoftSwitchMode(foc_motor_t *motor, uint8_t mode);
void FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(foc_motor_t *motor, float value);
void FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(foc_motor_t *motor, float value);
const foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatus(const foc_motor_t *motor);
void FOC_ControlResetCurrentSoftSwitchState(foc_motor_t *motor);
foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatusMutable(foc_motor_t *motor);

void FOC_ControlSetCoggingCompEnable(foc_motor_t *motor, uint8_t enable);
void FOC_ControlSetCoggingCompIqLimitA(foc_motor_t *motor, float value);
void FOC_ControlSetCoggingCompSpeedGateRadS(foc_motor_t *motor, float value);
void FOC_ControlSetCoggingCalibGainK(foc_motor_t *motor, float value);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

#endif /* FOC_CONTROL_C13_CFG_STATE_H */
