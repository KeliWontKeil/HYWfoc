#ifndef _USART1_APP_H_
#define _USART1_APP_H_

#include "usart.h"

/* USART1 application layer configuration */
#define USART1_APP_DEBUG_PROTOCOL_ENABLED    0   /* Currently disabled, reserved for future debug protocol */

/* Application layer status */
typedef enum {
    USART1_APP_OK = 0,
    USART1_APP_ERROR,
    USART1_APP_BUSY,
    USART1_APP_TIMEOUT
} usart1_app_status_t;

/* Function prototypes - Debug protocol framework */
void USART1_App_Init(void);
void USART1_App_Deinit(void);
usart1_app_status_t USART1_App_SendDebugData(const char *format, ...);
usart1_app_status_t USART1_App_ReceiveCommand(uint8_t *buffer, uint16_t size, uint32_t timeout);
void USART1_App_ProcessDebugProtocol(void);

/* Callback registration for protocol processing */
typedef void (*usart1_app_debug_callback_t)(uint8_t *data, uint16_t length);
void USART1_App_SetDebugCallback(usart1_app_debug_callback_t callback);

#endif /* _USART1_APP_H_ */