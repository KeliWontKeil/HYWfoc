#ifndef _USART2_H_
#define _USART2_H_

/*!
    \file    usart2.h
    \brief   USART2 module for serial communication (motor control parameter interface)

    \version 2026-3-11, V1.0.0, USART2 motor control parameter interface
*/

#include "gd32f30x.h"

/* USART2 pin definitions (from Hardware.md) */
#define USART2_PERIPH          USART2
#define USART2_RCU             RCU_USART2
#define USART2_GPIO_RCU        RCU_GPIOB
#define USART2_GPIO            GPIOB
#define USART2_TX_PIN          GPIO_PIN_10
#define USART2_RX_PIN          GPIO_PIN_11
#define USART2_IRQn            USART2_IRQn

/* USART configuration (same as USART1 for consistency) */
#define USART2_BAUDRATE        115200U
#define USART2_WORD_LENGTH     USART_WL_8BIT
#define USART2_STOP_BITS       USART_STB_1BIT
#define USART2_PARITY          USART_PM_NONE
#define USART2_HARDWARE_FLOW   USART_RTS_DISABLE

/* Buffer sizes */
#define USART2_RX_BUFFER_SIZE  128
#define USART2_TX_BUFFER_SIZE  128

/* USART status flags */
typedef enum {
    USART2_STATUS_OK = 0,
    USART2_STATUS_BUSY,
    USART2_STATUS_ERROR,
    USART2_STATUS_BUFFER_FULL
} usart2_status_t;

/* Function prototypes */
void USART2_Init(void);
usart2_status_t USART2_SendByte(uint8_t data);
usart2_status_t USART2_SendString(const char *str);
uint8_t USART2_ReceiveByte(void);
uint8_t USART2_IsDataAvailable(void);
void USART2_ClearBuffers(void);

/* Interrupt callback type */
typedef void (*usart2_rx_callback_t)(uint8_t data);

/* Callback registration */
void USART2_SetRxCallback(usart2_rx_callback_t callback);

/* Interrupt handler (to be called from vector table) */
void USART2_IRQHandler(void);

#endif /* _USART2_H_ */