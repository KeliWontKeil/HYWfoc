#ifndef _USART_H_
#define _USART_H_

/*!
    \file    usart.h
    \brief   USART1 module for basic serial communication

    \version 2026-3-9, V1.0.0
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

/* USART1 TX DMA: mapped to DMA0 Channel6 on GD32F30x */
#define USART1_TX_DMA_PERIPH   DMA0
#define USART1_TX_DMA_RCU      RCU_DMA0
#define USART1_TX_DMA_CHANNEL  DMA_CH6

/* USART1 RX DMA: mapped to DMA0 Channel5 on GD32F30x */
#define USART1_RX_DMA_PERIPH   DMA0
#define USART1_RX_DMA_RCU      RCU_DMA0
#define USART1_RX_DMA_CHANNEL  DMA_CH5

#define USART1_RX_DMA_BUFFER_SIZE 128U

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
usart_status_t USART1_SendByte(uint8_t data);
usart_status_t USART1_SendString(const char *str);
usart_status_t USART1_SendData(const uint8_t *data, uint16_t len);
uint8_t USART1_IsFrameReady(void);
uint16_t USART1_ReadFrame(uint8_t *buffer, uint16_t max_len);
void USART1_ClearBuffers(void);

/* Interrupt callback type */
typedef void (*usart_rx_callback_t)(uint8_t data);
typedef void (*usart1_idle_callback_t)(void);

/* Callback registration */
void USART1_SetRxCallback(usart_rx_callback_t callback);
void USART1_SetIdleCallback(usart1_idle_callback_t callback);

/* Interrupt handler (to be called from vector table) */
void USART1_IRQHandler(void);
void USART1_IRQHandler_Internal(void);

#endif /* _USART_H_ */
