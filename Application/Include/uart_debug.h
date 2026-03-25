#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

#include "foc_platform_api.h"
#include "control_scheduler.h"
#include "sensor.h"

#include <stdio.h>
#include <stdint.h>

/* Function prototypes */
void UART_Debug_Init(void);
void UART_Debug_OutputCurrent(void);
void UART_Debug_OutputEncoderAngle(void);
void UART_Debug_OutputAll(void);
void UART_Debug_OutputOscilloscope(sensor_data_t *sensor , foc_motor_t *motor);

#endif /* _UART_DEBUG_H_ */

