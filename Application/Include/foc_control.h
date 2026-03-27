#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include <stdint.h>
#include <math.h>

#include "foc_control_init.h"
#include "foc_control_internal.h"
#include "foc_shared_types.h"
#include "math_transforms.h"
#include "foc_platform_api.h"
#include "svpwm.h"

typedef struct {
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_runtime_config_t;

void FOC_ControlConfigResetDefault(void);
const foc_control_runtime_config_t *FOC_ControlGetRuntimeConfig(void);
void FOC_ControlSetMinMechAngleAccumDeltaRad(float value);
void FOC_ControlSetAngleHoldIntegralLimit(float value);
void FOC_ControlSetAngleHoldPidDeadbandRad(float value);
void FOC_ControlSetSpeedAngleTransitionStartRad(float value);
void FOC_ControlSetSpeedAngleTransitionEndRad(float value);

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);
float FOC_PIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec);
float FOC_CurrentLoopPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec);

void FOC_CurrentLoopStep(foc_motor_t *motor,
                                                 foc_pid_t *current_pid,
                                                 float iq_ref,
                                                 const sensor_data_t *sensor,
                                                 float electrical_angle,
                         float dt_sec);
void FOC_TorqueControlStep(foc_motor_t *motor,
                                                     foc_pid_t *current_pid,
                                                     float torque_ref_current,
                                                     const sensor_data_t *sensor,
                           float dt_sec,
                           foc_torque_mode_t mode);
void FOC_SpeedControlStep(foc_motor_t *motor,
                          foc_pid_t *speed_pid,
                          foc_pid_t *current_pid,
                          float speed_ref_rad_s,
                          const sensor_data_t *sensor,
                          float dt_sec,
                          foc_torque_mode_t torque_mode);
void FOC_SpeedAngleControlStep(foc_motor_t *motor,
                                                    foc_pid_t *speed_pid,
                                                    foc_pid_t *angle_hold_pid,
                                                    foc_pid_t *current_pid,
                               float angle_ref_rad,
                               float angle_position_speed_rad_s,
                                                    const sensor_data_t *sensor,
                               float dt_sec,
                               foc_torque_mode_t torque_mode);
void FOC_OpenLoopStep(foc_motor_t *motor,float voltage, float turn_speed);

#endif /* _FOC_CONTROL_H_ */
