#ifndef MOTOR_CONTROL_SERVICE_H
#define MOTOR_CONTROL_SERVICE_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"
#include "LS_Config/foc_shared_types.h"

void MotorControlService_ResetControlConfigDefault(foc_motor_t *motor);

void MotorControlService_InitMotor(foc_motor_t *motor,
                                   float vbus_voltage,
                                   float set_voltage,
                                   float phase_resistance,
                                   uint8_t pole_pairs,
                                   float mech_angle_at_elec_zero_rad,
                                   int8_t direction);

void MotorControlService_InitPwmOutput(uint16_t pwm_freq_khz, uint8_t deadtime_percent);
void MotorControlService_RunPwmInterpolationIsr(void);

void MotorControlService_InitSensorInput(uint8_t pwm_freq_khz, float sample_offset_percent);
void MotorControlService_SetSensorSampleOffsetPercent(float sample_offset_percent);
void MotorControlService_ReadAllSensorSnapshot(sensor_data_t *snapshot);
void MotorControlService_ReadCurrentSensorSnapshot(sensor_data_t *snapshot);

uint8_t MotorControlService_RequiresCurrentSample(void);
void MotorControlService_ResetCurrentSoftSwitchState(foc_motor_t *motor);

void MotorControlService_RunOuterLoopControlTask(foc_motor_t *motor,
                                                foc_pid_t *current_pid,
                                                foc_pid_t *speed_pid,
                                                foc_pid_t *angle_hold_pid,
                                                const sensor_data_t *sensor,
                                                uint8_t control_mode,
                                                float speed_only_rad_s,
                                                float target_angle_rad,
                                                float angle_position_speed_rad_s,
                                                float dt_sec);

void MotorControlService_RunCurrentLoopControlTask(foc_motor_t *motor,
                                                    foc_pid_t *current_pid,
                                                    const sensor_data_t *sensor,
                                                    float electrical_angle,
                                                    float dt_sec);

void MotorControlService_RunOpenLoopControlTask(foc_motor_t *motor,
                                                float open_loop_voltage,
                                                float open_loop_turn_speed);

void MotorControlService_InitPidControllers(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid,
                                            const control_config_snapshot_t *control_cfg);

void MotorControlService_ApplyConfigSnapshot(foc_motor_t *motor,
                                             foc_pid_t *current_pid,
                                             foc_pid_t *speed_pid,
                                             foc_pid_t *angle_hold_pid,
                                             const control_config_snapshot_t *control_cfg);

#endif /* MOTOR_CONTROL_SERVICE_H */
