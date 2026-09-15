#include "usart1.h"

/* Private variables — RX (unchanged) */
static uint8_t rx_dma_buffer[2][USART1_RX_DMA_BUFFER_SIZE];
static volatile uint8_t rx_irq_enabled = 0;
static volatile uint8_t rx_dma_active_idx = 0U;
static volatile uint8_t rx_dma_ready_idx = 0U;
static volatile uint16_t rx_dma_ready_len = 0U;
static volatile uint8_t rx_dma_frame_ready = 0U;
static usart1_idle_callback_t idle_callback = NULL;

/* Private variables — Fast Writer (ISR-safe, TXE interrupt driven) */
static volatile uint8_t usart1_fast_rp = 0U;      /* ring buffer read index (TXE ISR consumes) */
static volatile uint8_t usart1_fast_wp = 0U;      /* ring buffer write index (PutByte produces) */
static uint8_t usart1_fast_ring[USART1_FAST_RING_SIZE];

/* Private variables — Slow Writer (main-loop only, DMA driven) */
static uint8_t usart1_slow_tx_buf[USART1_SLOW_TX_BUF_SIZE];

/* Private function prototypes */
static void USART1_DMAConfigFast(void);
static void USART1_DMARxConfig(void);
static void USART1_DMARxRestart(uint8_t buffer_index);
static void USART1_EnableInterrupts(void);
static void USART1_DisableInterrupts(void);

/*!
    \brief      Initialize USART1 basic communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_Init(void)
{
    /* Enable GPIO and USART clocks */
    rcu_periph_clock_enable(USART1_GPIO_RCU);
    rcu_periph_clock_enable(USART1_RCU);
    
    /* Configure GPIO pins for USART1 */
    gpio_init(USART1_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, USART1_TX_PIN);
    gpio_init(USART1_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, USART1_RX_PIN);
    
    /* USART parameter configuration */
    usart_deinit(USART1_PERIPH);
    usart_baudrate_set(USART1_PERIPH, USART1_BAUDRATE);
    usart_word_length_set(USART1_PERIPH, USART1_WORD_LENGTH);
    usart_stop_bit_set(USART1_PERIPH, USART1_STOP_BITS);
    usart_parity_config(USART1_PERIPH, USART1_PARITY);
    usart_hardware_flow_rts_config(USART1_PERIPH, USART1_HARDWARE_FLOW);
    usart_hardware_flow_cts_config(USART1_PERIPH, USART_CTS_DISABLE);

    usart_receive_config(USART1_PERIPH, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1_PERIPH, USART_TRANSMIT_ENABLE);
    
    
    /* Enable USART */
    usart_enable(USART1_PERIPH);
    
    /* Clear buffers */
    USART1_ClearBuffers();

    USART1_DMAConfigFast();
    USART1_DMARxConfig();

    /* RX IDLE interrupt (fast path TXE is enabled on demand by PutByte) */
    usart_interrupt_enable(USART1_PERIPH, USART_INT_IDLE);
    rx_irq_enabled = 1U;
    
    /* Configure NVIC for USART1 */
    NVIC_CONFIG(USART1_IRQn, USART1_PRIORITY_GROUP, USART1_PRIORITY_SUBGROUP);
}

/* =================================================================
 *  Fast Writer — ISR-safe, non-blocking, TXE interrupt driven
 * ================================================================= */

/*!
    \brief      Write one byte via fast path (ISR-safe, non-blocking)
    \param[in]  byte: data byte to send
    \retval     none
    \note       Uses a ring buffer + TXE interrupt. If the ring is full,
                the byte is silently discarded.
*/
void USART1_FastWriter_PutByte(uint8_t byte)
{
    uint8_t next = (uint8_t)((usart1_fast_wp + 1U) % USART1_FAST_RING_SIZE);

    /* Only enqueue if ring is not full */
    if (next != usart1_fast_rp)
    {
        usart1_fast_ring[usart1_fast_wp] = byte;
        usart1_fast_wp = next;
    }

    /* Enable TXE interrupt to trigger transmission */
    usart_interrupt_enable(USART1_PERIPH, USART_INT_TBE);
}

/*!
    \brief      Write a string via fast path (ISR-safe, non-blocking)
    \param[in]  str: null-terminated string to send
    \retval     none
    \note       Each character is enqueued individually via PutByte.
                If the ring is full, remaining characters are discarded.
*/
void USART1_FastWriter_PutString(const char *str)
{
    if (str == NULL) return;

    while (*str != '\0')
    {
        USART1_FastWriter_PutByte((uint8_t)*str);
        str++;
    }
}

/*!
    \brief      Check if fast path ring buffer is empty
    \param[in]  none
    \retval     1 if empty, 0 if data pending
*/
uint8_t USART1_FastWriter_IsEmpty(void)
{
    return (usart1_fast_rp == usart1_fast_wp) ? 1U : 0U;
}

/*!
    \brief      Spin-wait until fast path ring buffer is drained
    \param[in]  none
    \retval     none
    \note       Must only be called from main-loop context.
*/
void USART1_FastWriter_Flush(void)
{
    while (usart1_fast_rp != usart1_fast_wp) {}
}

/*!
    \brief      Fast path TXE interrupt handler (consumes from ring buffer)
    \param[in]  none
    \retval     none
*/
static void USART1_FastWriter_IRQHandler(void)
{
    if (usart1_fast_rp != usart1_fast_wp)
    {
        usart_data_transmit(USART1_PERIPH, usart1_fast_ring[usart1_fast_rp]);
        usart1_fast_rp = (uint8_t)((usart1_fast_rp + 1U) % USART1_FAST_RING_SIZE);
    }
    else
    {
        usart_interrupt_disable(USART1_PERIPH, USART_INT_TBE);
    }
}

/* =================================================================
 *  Slow Writer — main-loop only, blocking DMA
 * ================================================================= */

/*!
    \brief      Send data via slow path (blocking DMA, main-loop only)
    \param[in]  data: buffer pointer
    \param[in]  len: number of bytes
    \retval     USART_STATUS_OK on success
    \note       Waits for FastWriter ring to drain before starting DMA.
                Re-enables TXE interrupt if fast path has queued data afterwards.
*/
usart_status_t USART1_SlowWriter_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0U;

    if (data == NULL)
    {
        return USART_STATUS_ERROR;
    }

    /* Drain any pending fast-path bytes before starting DMA */
    USART1_FastWriter_Flush();
    /* Disable TXE interrupt to prevent DMA/TXE contention on USART DATA register */
    usart_interrupt_disable(USART1_PERIPH, USART_INT_TBE);

    while (offset < len)
    {
        uint16_t chunk = len - offset;

        if (chunk > USART1_SLOW_TX_BUF_SIZE)
        {
            chunk = USART1_SLOW_TX_BUF_SIZE;
        }

        memcpy(usart1_slow_tx_buf, &data[offset], chunk);

        /* Configure and start DMA transfer */
        dma_channel_disable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
        while ((DMA_CHCTL(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL) & DMA_CHXCTL_CHEN) != 0U) {}
        dma_memory_address_config(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, (uint32_t)usart1_slow_tx_buf);
        dma_transfer_number_config(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, chunk);
        dma_flag_clear(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_G);

        usart_dma_transmit_config(USART1_PERIPH, USART_TRANSMIT_DMA_ENABLE);
        dma_channel_enable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);

        /* Wait for DMA transfer complete */
        while (dma_flag_get(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_FTF) == RESET) {}

        dma_flag_clear(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_G);
        dma_channel_disable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
        while ((DMA_CHCTL(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL) & DMA_CHXCTL_CHEN) != 0U) {}
        usart_dma_transmit_config(USART1_PERIPH, USART_TRANSMIT_DMA_DISABLE);

        /* Wait for USART TC flag (last byte physically shifted out) */
        while (usart_flag_get(USART1_PERIPH, USART_FLAG_TC) == RESET) {}

        offset += chunk;
    }

    /* Re-enable TXE interrupt if fast path has queued data during DMA */
    if (usart1_fast_rp != usart1_fast_wp)
    {
        usart_interrupt_enable(USART1_PERIPH, USART_INT_TBE);
    }

    return USART_STATUS_OK;
}

/* =================================================================
 *  RX (unchanged from original)
 * ================================================================= */

/*!
    \brief      Configure DMA for TX (slow path)
    \param[in]  none
    \retval     none
*/
static void USART1_DMAConfigFast(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(USART1_TX_DMA_RCU);
    dma_deinit(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)usart1_slow_tx_buf;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 0U;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART1_PERIPH);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;

    dma_init(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, &dma_init_struct);
    dma_circulation_disable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
}

uint8_t USART1_IsFrameReady(void)
{
    return rx_dma_frame_ready;
}

uint16_t USART1_ReadFrame(uint8_t *buffer, uint16_t max_len)
{
    uint16_t copy_len;

    if ((buffer == NULL) || (max_len == 0U) || (rx_dma_frame_ready == 0U))
    {
        return 0U;
    }

    copy_len = rx_dma_ready_len;
    if (copy_len > max_len)
    {
        copy_len = max_len;
    }

    memcpy(buffer, (const void *)rx_dma_buffer[rx_dma_ready_idx], copy_len);
    rx_dma_frame_ready = 0U;
    rx_dma_ready_len = 0U;

    return copy_len;
}

/*!
    \brief      Clear USART1 buffers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_ClearBuffers(void)
{
    USART1_DisableInterrupts();

    /* Clear RX buffers */
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));

    rx_dma_active_idx = 0U;
    rx_dma_ready_idx = 0U;
    rx_dma_ready_len = 0U;
    rx_dma_frame_ready = 0U;

    /* Reset fast ring buffer */
    usart1_fast_rp = 0U;
    usart1_fast_wp = 0U;
    memset(usart1_fast_ring, 0, sizeof(usart1_fast_ring));
    
    USART1_EnableInterrupts();
}

/*!
    \brief      Set RX callback function
    \param[in]  callback: function pointer to call when data received
    \param[out] none
    \retval     none
*/
void USART1_SetRxCallback(usart_rx_callback_t callback)
{
    (void)callback;
}

void USART1_SetIdleCallback(usart1_idle_callback_t callback)
{
    idle_callback = callback;
}

/*!
    \brief      USART1 interrupt handler implementation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler_Internal(void)
{
    /* IDLE interrupt (RX frame detection) */
    if (usart_interrupt_flag_get(USART1_PERIPH, USART_INT_FLAG_IDLE) != RESET)
    {
        uint16_t received_len;
        uint8_t completed_idx = rx_dma_active_idx;
        uint8_t next_idx = (uint8_t)(1U - rx_dma_active_idx);

        (void)USART_STAT0(USART1_PERIPH);
        (void)USART_DATA(USART1_PERIPH);

        dma_channel_disable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);

        received_len = (uint16_t)(USART1_RX_DMA_BUFFER_SIZE -
                                  dma_transfer_number_get(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL));

        if (received_len > 0U)
        {
            rx_dma_ready_idx = completed_idx;
            rx_dma_ready_len = received_len;
            rx_dma_frame_ready = 1U;

            if (idle_callback != NULL)
            {
                idle_callback();
            }
        }

        USART1_DMARxRestart(next_idx);
    }

    /* TXE interrupt (FastWriter path) */
    if (usart_interrupt_flag_get(USART1_PERIPH, USART_INT_FLAG_TBE) != RESET)
    {
        USART1_FastWriter_IRQHandler();
    }
}

static void USART1_DMARxConfig(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(USART1_RX_DMA_RCU);
    dma_deinit(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)rx_dma_buffer[0];
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = USART1_RX_DMA_BUFFER_SIZE;
    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(USART1_PERIPH);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;

    dma_init(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL, &dma_init_struct);
    dma_circulation_disable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);

    rx_dma_active_idx = 0U;
    rx_dma_frame_ready = 0U;

    usart_dma_receive_config(USART1_PERIPH, USART_RECEIVE_DMA_ENABLE);
    dma_channel_enable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
}

static void USART1_DMARxRestart(uint8_t buffer_index)
{
    rx_dma_active_idx = (uint8_t)(buffer_index & 0x01U);

    dma_memory_address_config(USART1_RX_DMA_PERIPH,
                              USART1_RX_DMA_CHANNEL,
                              (uint32_t)rx_dma_buffer[rx_dma_active_idx]);
    dma_transfer_number_config(USART1_RX_DMA_PERIPH,
                               USART1_RX_DMA_CHANNEL,
                               USART1_RX_DMA_BUFFER_SIZE);
    dma_flag_clear(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL, DMA_FLAG_G);
    dma_channel_enable(USART1_RX_DMA_PERIPH, USART1_RX_DMA_CHANNEL);
}

/*!
    \brief      Disable USART1 interrupts (for critical sections)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART1_DisableInterrupts(void)
{
    if (rx_irq_enabled != 0U)
    {
        usart_interrupt_disable(USART1_PERIPH, USART_INT_IDLE);
    }
    usart_interrupt_disable(USART1_PERIPH, USART_INT_TBE);
}

/*!
    \brief      Enable USART1 interrupts
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART1_EnableInterrupts(void)
{
    if (rx_irq_enabled != 0U)
    {
        usart_interrupt_enable(USART1_PERIPH, USART_INT_IDLE);
    }
    /* TXE interrupt is enabled on demand by FastWriter_PutByte, not here */
}
