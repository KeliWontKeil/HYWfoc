#ifndef MOTOR_CONTROL_SERVICE_H
#define MOTOR_CONTROL_SERVICE_H

#include <stdint.h>

#include "LS_Config/foc_shared_types.h"

typedef enum {
    MOTOR_CONTROL_SERVICE_TASK_OUTER_LOOP = 0U,
    MOTOR_CONTROL_SERVICE_TASK_CURRENT_LOOP = 1U,
    MOTOR_CONTROL_SERVICE_TASK_OPEN_LOOP = 2U
} motor_control_service_task_t;

typedef struct {
    const sensor_data_t *sensor;
    uint8_t control_mode;
    float speed_only_rad_s;
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float electrical_angle;
    float open_loop_voltage;
    float open_loop_turn_speed;
    float dt_sec;
} motor_control_service_task_args_t;

void MotorControlService_ResetControlConfigDefault(void);

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
uint8_t MotorControlService_ReadAllSensorSnapshot(sensor_data_t *snapshot);
uint8_t MotorControlService_ReadCurrentSensorSnapshot(sensor_data_t *snapshot);

uint8_t MotorControlService_RequiresCurrentSample(void);
void MotorControlService_ResetCurrentSoftSwitchState(void);

uint8_t MotorControlService_RunControlTask(motor_control_service_task_t task,
                                           foc_motor_t *motor,
                                           foc_pid_t *current_pid,
                                           foc_pid_t *speed_pid,
                                           foc_pid_t *angle_hold_pid,
                                           const motor_control_service_task_args_t *args);

void MotorControlService_InitPidControllers(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid);

void MotorControlService_ApplyPendingConfig(foc_motor_t *motor,
                                            foc_pid_t *current_pid,
                                            foc_pid_t *speed_pid,
                                            foc_pid_t *angle_hold_pid);

#endif /* MOTOR_CONTROL_SERVICE_H */
