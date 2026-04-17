#ifndef _CONTROL_CONFIG_IFACE_H_
#define _CONTROL_CONFIG_IFACE_H_

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

void FOC_ControlConfigResetDefault(void);
void FOC_ControlSetMinMechAngleAccumDeltaRad(float value);
void FOC_ControlSetAngleHoldIntegralLimit(float value);
void FOC_ControlSetAngleHoldPidDeadbandRad(float value);
void FOC_ControlSetSpeedAngleTransitionStartRad(float value);
void FOC_ControlSetSpeedAngleTransitionEndRad(float value);
void FOC_ControlSetCurrentSoftSwitchEnable(uint8_t enable);
void FOC_ControlSetCurrentSoftSwitchMode(uint8_t mode);
void FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(float value);
void FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(float value);
void FOC_ControlResetCurrentSoftSwitchState(void);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

#endif /* _CONTROL_CONFIG_IFACE_H_ */
