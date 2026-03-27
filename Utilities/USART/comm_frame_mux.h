#ifndef COMM_FRAME_MUX_H
#define COMM_FRAME_MUX_H

#include <stdint.h>

#define COMM_FRAME_MUX_SOURCE_USART1 (1U << 0)
#define COMM_FRAME_MUX_SOURCE_USART2 (1U << 1)
#define COMM_FRAME_MUX_SOURCE_ALL    (COMM_FRAME_MUX_SOURCE_USART1 | COMM_FRAME_MUX_SOURCE_USART2)

typedef enum {
    COMM_FRAME_MUX_ARB_ROUND_ROBIN = 0
} comm_frame_mux_arb_policy_t;

typedef struct {
    uint8_t source_mask;
    uint8_t arbitration_policy;
} comm_frame_mux_config_t;

typedef void (*comm_frame_mux_rx_trigger_callback_t)(void);

void CommFrameMux_Init(const comm_frame_mux_config_t *config);
void CommFrameMux_SetRxTriggerCallback(comm_frame_mux_rx_trigger_callback_t callback);
uint8_t CommFrameMux_HasPendingFrame(void);
uint16_t CommFrameMux_TryDequeueFrame(uint8_t *buffer, uint16_t max_len);

#endif /* COMM_FRAME_MUX_H */
