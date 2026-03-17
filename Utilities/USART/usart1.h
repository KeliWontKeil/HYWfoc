#ifndef _USART_H_
#define _USART_H_

/*!
    \file    usart.h
    \brief   USART module for serial communication

    \version 2026-3-9, V1.0.0, USART1 loopback implementation
*/

#include "gd32f30x.h"
#include "interrupt_priority.h"
#include <string.h>

/* USART1 pin definitions (from Hardware.md) */
#define USART1_PERIPH          USART1
#define USART1_RCU             RCU_USART1
#define USART1_GPIO_RCU        RCU_GPIOA
#define USART1_GPIO            GPIOA
#define USART1_TX_PIN          GPIO_PIN_2
#define USART1_RX_PIN          GPIO_PIN_3
#define USART1_IRQn            USART1_IRQn

/* USART configuration */
#define USART1_BAUDRATE        115200U
#define USART1_WORD_LENGTH     USART_WL_8BIT
#define USART1_STOP_BITS       USART_STB_1BIT
#define USART1_PARITY          USART_PM_NONE
#define USART1_HARDWARE_FLOW   USART_RTS_DISABLE

/* Buffer sizes */
#define USART1_RX_BUFFER_SIZE  128
#define USART1_TX_BUFFER_SIZE  128

/* USART status flags */
typedef enum {
    USART_STATUS_OK = 0,
    USART_STATUS_BUSY,
    USART_STATUS_ERROR,
    USART_STATUS_BUFFER_FULL
} usart_status_t;

/* Function prototypes */
void USART1_Init(void);
void USART1_LoopbackEnable(void);
void USART1_LoopbackDisable(void);
usart_status_t USART1_SendByte(char data);
usart_status_t USART1_SendString(char *str);
usart_status_t USART1_SendData(const uint8_t *data, uint16_t len);
uint8_t USART1_ReceiveByte(void);
uint8_t USART1_IsDataAvailable(void);
void USART1_ClearBuffers(void);

/* Interrupt callback type */
typedef void (*usart_rx_callback_t)(uint8_t data);

/* Callback registration */
void USART1_SetRxCallback(usart_rx_callback_t callback);

/* Interrupt handler (to be called from vector table) */
void USART1_IRQHandler(void);

#endif /* _USART_H_ */
