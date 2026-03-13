#ifndef _USART2_APP_H_
#define _USART2_APP_H_

#include "usart2.h"

/* USART2 application layer configuration - motor control interface */
#define USART2_APP_MOTOR_PROTOCOL_ENABLED    0   /* Currently disabled, reserved for future motor control protocol */

/* Motor control parameter types */
typedef enum {
    MOTOR_PARAM_SPEED = 0,
    MOTOR_PARAM_TORQUE,
    MOTOR_PARAM_POSITION,
    MOTOR_PARAM_CURRENT_LIMIT,
    MOTOR_PARAM_VOLTAGE_LIMIT,
    MOTOR_PARAM_TEMPERATURE_LIMIT,
    MOTOR_PARAM_MODE
} motor_parameter_t;

/* Motor control modes */
typedef enum {
    MOTOR_MODE_IDLE = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_TORQUE,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_CALIBRATION
} motor_mode_t;

/* Application layer status */
typedef enum {
    USART2_APP_OK = 0,
    USART2_APP_ERROR,
    USART2_APP_BUSY,
    USART2_APP_TIMEOUT,
    USART2_APP_INVALID_PARAM
} usart2_app_status_t;

/* Function prototypes - Motor control parameter interface framework */
void USART2_App_Init(void);
void USART2_App_Deinit(void);
usart2_app_status_t USART2_App_SendMotorStatus(float speed, float torque, float position, float temperature);
usart2_app_status_t USART2_App_ReceiveMotorCommand(float *speed_setpoint, float *torque_setpoint, motor_mode_t *mode);
usart2_app_status_t USART2_App_SetMotorParameter(motor_parameter_t param, float value);
usart2_app_status_t USART2_App_GetMotorParameter(motor_parameter_t param, float *value);
void USART2_App_ProcessMotorProtocol(void);

/* Callback registration for motor control command processing */
typedef void (*usart2_app_motor_callback_t)(motor_mode_t mode, float speed, float torque);
void USART2_App_SetMotorCallback(usart2_app_motor_callback_t callback);

#endif /* _USART2_APP_H_ */