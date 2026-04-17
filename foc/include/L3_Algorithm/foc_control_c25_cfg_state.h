#ifndef FOC_CONTROL_C25_CFG_STATE_H
#define FOC_CONTROL_C25_CFG_STATE_H

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

typedef struct {
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_runtime_config_t;

typedef struct {
    uint8_t enabled;
    uint8_t configured_mode;
    uint8_t active_mode;
    float blend_factor;
    float auto_open_iq_a;
    float auto_closed_iq_a;
} foc_current_soft_switch_status_t;

typedef struct {
    uint8_t enabled;
    uint8_t available;
    uint8_t source;
    uint16_t point_count;
    float iq_lsb_a;
    float speed_gate_rad_s;
    float iq_limit_a;
} foc_cogging_comp_status_t;

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
foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatusMutable(void);
uint8_t *FOC_ControlGetCurrentSoftSwitchBlendInitFlag(void);

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
void FOC_ControlGetCoggingCompContext(const foc_cogging_comp_status_t **status,
                                      const int16_t **table_q15);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

#endif /* FOC_CONTROL_C25_CFG_STATE_H */
