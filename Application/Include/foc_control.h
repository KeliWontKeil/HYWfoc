#ifndef _FOC_CONTROL_H_
#define _FOC_CONTROL_H_

#include <stdint.h>
#include <math.h>

#include "foc_shared_types.h"
#include "math_transforms.h"
#include "foc_platform_api.h"
#include "svpwm.h"

#define FOC_TWO_PI 6.2831852f
#define FOC_CALIB_SETTLE_MS 4U
#define FOC_CALIB_MIN_MECH_STEP_RAD 0.0015f
#define FOC_CALIB_LOCK_SETTLE_MS 60U
#define FOC_CALIB_LOCK_SAMPLE_COUNT 24U
#define FOC_CALIB_STEP_SETTLE_MS 8U
#define FOC_CALIB_STEP_SAMPLE_COUNT 6U
#define FOC_CALIB_STEP_ELEC_RAD (FOC_TWO_PI / 20.0f)
#define FOC_CALIB_STEP_COUNT 20U

#define FOC_DIR_UNDEFINED 0
#define FOC_DIR_NORMAL 1
#define FOC_DIR_REVERSED -1
#define FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED (-1.0f)
#define FOC_POLE_PAIRS_UNDEFINED 0U

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction);
void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max);
float FOC_PIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec);

void FOC_CurrentLoopStep(foc_motor_t *motor,
                                                 foc_pid_t *current_pid,
                                                 float iq_ref,
                                                 float phase_a_current,
                                                 float phase_b_current,
                                                 float phase_c_current,
                                                 float electrical_angle,
                         float dt_sec);
void FOC_TorqueControlStep(foc_motor_t *motor,
                                                     foc_pid_t *current_pid,
                                                     float torque_ref_current,
                                                     float phase_a_current,
                                                     float phase_b_current,
                                                     float phase_c_current,
                                                     float mech_angle_rad,
                           float dt_sec,
                           foc_torque_mode_t mode);
void FOC_AngleControlStep(foc_motor_t *motor,
                                                    foc_pid_t *angle_pid,
                                                    foc_pid_t *current_pid,
                          float angle_ref_rad,
                          float phase_a_current,
                          float phase_b_current,
                          float phase_c_current,
                          float mech_angle_rad,
                          float dt_sec,
                          foc_torque_mode_t torque_mode);
void FOC_SpeedControlStep(foc_motor_t *motor,
                                                    foc_pid_t *speed_pid,
                                                    foc_pid_t *current_pid,
                          float speed_ref_rad_s,
                          float phase_a_current,
                          float phase_b_current,
                          float phase_c_current,
                          float mech_angle_rad,
                          float dt_sec,
                          foc_torque_mode_t torque_mode);
void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor);
void FOC_OpenLoopStep(foc_motor_t *motor,float voltage, float turn_speed);

#endif /* _FOC_CONTROL_H_ */
