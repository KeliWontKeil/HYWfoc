/*!
    \file    uart_debug.h
    \brief   UART debug module for outputting motor control debug information

    \version 2026-03-13, V1.0.0, UART debug for GD32F303CC FOC project
*/

#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

#include "adc.h"
#include "as5600.h"
#include <stdint.h>
#include <stdio.h>
#include "usart1.h"
#include "timer1_algorithm.h"
#include "sensor.h"

/* Function prototypes */
void UART_Debug_Init(void);
void UART_Debug_OutputCurrent(void);
void UART_Debug_OutputEncoderAngle(void);
void UART_Debug_OutputAll(void);

#endif /* _UART_DEBUG_H_ */

