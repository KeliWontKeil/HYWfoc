/*!
    \file    uart_debug.h
    \brief   UART debug module for outputting motor control debug information

    \version 2026-03-13, V1.0.0, UART debug for GD32F303CC FOC project
*/

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
void UART_Debug_OutputOscilloscope(float iq);

#endif /* _UART_DEBUG_H_ */

