#ifndef _USART0_H_
#define _USART0_H_

/*!
    \file    usart0.h
    \brief   USART0 module for serial communication (main debug/protocol channel)

    \version 2026-8-13, V1.0.0, USART0 driver for GD32F30x
    \note    USART0 remapped to PB6(TX)/PB7(RX), DMA0 CH4(TX)/CH3(RX).
*/

#include "gd32f30x.h"
#include "interrupt_priority.h"
#include <string.h>

/* USART0 pin definitions (from Hardware.md, remap to PB6/PB7) */
#define USART0_PERIPH          USART0
#define USART0_RCU             RCU_USART0
#define USART0_GPIO_RCU        RCU_GPIOB
#define USART0_GPIO            GPIOB
#define USART0_TX_PIN          GPIO_PIN_6
#define USART0_RX_PIN          GPIO_PIN_7
#define USART0_IRQn            USART0_IRQn

/* USART configuration */
#define USART0_BAUDRATE        115200U
#define USART0_WORD_LENGTH     USART_WL_8BIT
#define USART0_STOP_BITS       USART_STB_1BIT
#define USART0_PARITY          USART_PM_NONE
#define USART0_HARDWARE_FLOW   USART_RTS_DISABLE

/* USART0 TX DMA: mapped to DMA0 Channel4 on GD32F30x */
#define USART0_TX_DMA_PERIPH   DMA0
#define USART0_TX_DMA_RCU      RCU_DMA0
#define USART0_TX_DMA_CHANNEL  DMA_CH3

/* USART0 RX DMA: mapped to DMA0 Channel3 on GD32F30x */
#define USART0_RX_DMA_PERIPH   DMA0
#define USART0_RX_DMA_RCU      RCU_DMA0
#define USART0_RX_DMA_CHANNEL  DMA_CH4

#define USART0_RX_DMA_BUFFER_SIZE 128U

/* Buffer sizes */
#define USART0_RX_BUFFER_SIZE  128

/* Fast path ring buffer size (ISR-safe, TXE interrupt driven) */
#define USART0_FAST_RING_SIZE  16U

/* Slow path TX buffer size (DMA driven, main-loop only) */
#define USART0_SLOW_TX_BUF_SIZE  128U

/* USART status flags */
typedef enum {
    USART0_STATUS_OK = 0,
    USART0_STATUS_BUSY,
    USART0_STATUS_ERROR,
    USART0_STATUS_BUFFER_FULL
} usart0_status_t;

/* ===== RX ===== */
void USART0_Init(void);
uint8_t USART0_IsFrameReady(void);
uint16_t USART0_ReadFrame(uint8_t *buffer, uint16_t max_len);
void USART0_ClearBuffers(void);

/* ===== TX: Fast Writer (ISR-safe, non-blocking, TXE interrupt driven) ===== */
void USART0_FastWriter_PutByte(uint8_t byte);
void USART0_FastWriter_PutString(const char *str);
uint8_t USART0_FastWriter_IsEmpty(void);
void USART0_FastWriter_Flush(void);

/* ===== TX: Slow Writer (main-loop only, blocking DMA) ===== */
usart0_status_t USART0_SlowWriter_SendData(const uint8_t *data, uint16_t len);

/* Interrupt callback type */
typedef void (*usart0_rx_callback_t)(uint8_t data);
typedef void (*usart0_idle_callback_t)(void);

/* Callback registration */
void USART0_SetRxCallback(usart0_rx_callback_t callback);
void USART0_SetIdleCallback(usart0_idle_callback_t callback);

/* Interrupt handler (to be called from vector table) */
void USART0_IRQHandler(void);
void USART0_IRQHandler_Internal(void);

#endif /* _USART0_H_ */
