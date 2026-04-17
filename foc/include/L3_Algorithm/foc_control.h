#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

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
void FOC_ControlRebaseMechanicalAngleAccum(foc_motor_t *motor, float mech_angle_rad);
void FOC_ControlResetSpeedLoopState(void);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);

void FOC_SpeedOuterLoopStep(foc_motor_t *motor,
                            foc_pid_t *speed_pid,
                            float speed_ref_rad_s,
                            const sensor_data_t *sensor,
                            float dt_sec);
void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 const sensor_data_t *sensor,
                                 float dt_sec);
uint8_t FOC_ControlOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *current_pid,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 const sensor_data_t *sensor,
                                 uint8_t control_mode,
                                 float speed_only_rad_s,
                                 float target_angle_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec);
uint8_t FOC_ControlRequiresCurrentSample(void);
void FOC_CurrentControlStep(foc_motor_t *motor,
                            foc_pid_t *current_pid,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec);
void FOC_OpenLoopStep(foc_motor_t *motor,float voltage, float turn_speed);

#endif /* _FOC_CONTROL_H_ */
