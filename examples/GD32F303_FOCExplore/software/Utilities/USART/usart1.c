#include "usart1.h"

/* Private variables */
static uint8_t tx_dma_buffer[USART1_TX_BUFFER_SIZE];
static uint8_t rx_dma_buffer[2][USART1_RX_DMA_BUFFER_SIZE];
static volatile uint8_t rx_irq_enabled = 0;
static volatile uint8_t rx_dma_active_idx = 0U;
static volatile uint8_t rx_dma_ready_idx = 0U;
static volatile uint16_t rx_dma_ready_len = 0U;
static volatile uint8_t rx_dma_frame_ready = 0U;
static usart1_idle_callback_t idle_callback = NULL;

/* Private function prototypes */
static void USART1_DMATxConfig(void);
static void USART1_DMARxConfig(void);
static void USART1_DMARxRestart(uint8_t buffer_index);
static usart_status_t USART1_DMATxTransfer(const uint8_t *data, uint16_t len);
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

    USART1_DMATxConfig();
    USART1_DMARxConfig();

    usart_interrupt_enable(USART1_PERIPH, USART_INT_IDLE);
    rx_irq_enabled = 1U;
    
    /* Configure NVIC for USART1 */
    NVIC_CONFIG(USART1_IRQn, USART1_PRIORITY_GROUP, USART1_PRIORITY_SUBGROUP);
}

/*!
    \brief      Send a byte via USART1
    \param[in]  data: byte to send
    \param[out] none
    \retval     status of operation
*/
usart_status_t USART1_SendByte(uint8_t data)
{
    return USART1_DMATxTransfer(&data, 1U);
}

/*!
    \brief      Send a string via USART1
    \param[in]  str: null-terminated string to send
    \param[out] none
    \retval     status of operation
*/
usart_status_t USART1_SendString(const char *str)
{
    if (str == NULL)
    {
        return USART_STATUS_ERROR;
    }

    return USART1_SendData((const uint8_t *)str, (uint16_t)strlen(str));
}

/*!
    \brief      Send fixed-length binary data via USART1
    \param[in]  data: buffer pointer
    \param[in]  len: number of bytes
    \param[out] none
    \retval     status of operation
*/
usart_status_t USART1_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0U;

    if (data == NULL)
    {
        return USART_STATUS_ERROR;
    }

    while (offset < len)
    {
        uint16_t chunk = len - offset;

        if (chunk > USART1_TX_BUFFER_SIZE)
        {
            chunk = USART1_TX_BUFFER_SIZE;
        }

        memcpy(tx_dma_buffer, &data[offset], chunk);

        if (USART1_DMATxTransfer(tx_dma_buffer, chunk) != USART_STATUS_OK)
        {
            return USART_STATUS_ERROR;
        }

        offset += chunk;
    }

    return USART_STATUS_OK;
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

    memset(tx_dma_buffer, 0, sizeof(tx_dma_buffer));
    memset(rx_dma_buffer, 0, sizeof(rx_dma_buffer));

    rx_dma_active_idx = 0U;
    rx_dma_ready_idx = 0U;
    rx_dma_ready_len = 0U;
    rx_dma_frame_ready = 0U;
    
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
}

static void USART1_DMATxConfig(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(USART1_TX_DMA_RCU);
    dma_deinit(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)tx_dma_buffer;
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

static usart_status_t USART1_DMATxTransfer(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return USART_STATUS_ERROR;
    }

    dma_channel_disable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
    dma_memory_address_config(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, (uint32_t)data);
    dma_transfer_number_config(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, len);
    dma_flag_clear(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_G);

    usart_dma_transmit_config(USART1_PERIPH, USART_TRANSMIT_DMA_ENABLE);
    dma_channel_enable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);

    while (dma_flag_get(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_FTF) == RESET)
    {
    }

    dma_flag_clear(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL, DMA_FLAG_G);
    dma_channel_disable(USART1_TX_DMA_PERIPH, USART1_TX_DMA_CHANNEL);
    usart_dma_transmit_config(USART1_PERIPH, USART_TRANSMIT_DMA_DISABLE);

    while (usart_flag_get(USART1_PERIPH, USART_FLAG_TC) == RESET)
    {
    }

    return USART_STATUS_OK;
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
}
