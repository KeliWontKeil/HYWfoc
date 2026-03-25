#include "usart1.h"

/* Private variables */
static uint8_t rx_buffer[USART1_RX_BUFFER_SIZE];
static uint8_t tx_dma_buffer[USART1_TX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint8_t rx_irq_enabled = 0;
static usart_rx_callback_t rx_callback = NULL;

/* Private function prototypes */
static uint8_t USART1_BufferIsFull(uint16_t head, uint16_t tail, uint16_t size);
static uint8_t USART1_BufferIsEmpty(uint16_t head, uint16_t tail);
static void USART1_DMATxConfig(void);
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

    /* Enable USART receive interrupt */
    usart_interrupt_enable(USART1_PERIPH, USART_INT_RBNE);
    
    /* RX interrupt is intentionally kept disabled by default in current debug stage. */
    rx_irq_enabled = 0U;

    USART1_DMATxConfig();
    
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

/*!
    \brief      Receive a byte from USART1 buffer
    \param[in]  none
    \param[out] none
    \retval     received byte (0 if buffer empty)
*/
uint8_t USART1_ReceiveByte(void)
{
    uint8_t data = 0;
    
    USART1_DisableInterrupts();
    
    if (!USART1_BufferIsEmpty(rx_head, rx_tail))
    {
        data = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % USART1_RX_BUFFER_SIZE;
    }
    
    USART1_EnableInterrupts();
    
    return data;
}

/*!
    \brief      Check if data is available in RX buffer
    \param[in]  none
    \param[out] none
    \retval     1 if data available, 0 otherwise
*/
uint8_t USART1_IsDataAvailable(void)
{
    return !USART1_BufferIsEmpty(rx_head, rx_tail);
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
    
    rx_head = 0;
    rx_tail = 0;
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(tx_dma_buffer, 0, sizeof(tx_dma_buffer));
    
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
    rx_callback = callback;
}

/*!
    \brief      USART1 interrupt handler implementation
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART1_HandlerInternal(void)
{
    /* Receive buffer not empty interrupt */
    if (usart_interrupt_flag_get(USART1_PERIPH, USART_INT_FLAG_RBNE) != RESET)
    {
        uint8_t data = usart_data_receive(USART1_PERIPH);
        
        /* Add to RX buffer */
        if (!USART1_BufferIsFull(rx_head, rx_tail, USART1_RX_BUFFER_SIZE))
        {
            rx_buffer[rx_head] = data;
            rx_head = (rx_head + 1) % USART1_RX_BUFFER_SIZE;
            
            /* Call callback if set */
            if (rx_callback != NULL)
            {
                rx_callback(data);
            }
        }
        
        usart_interrupt_flag_clear(USART1_PERIPH, USART_INT_FLAG_RBNE);
    }
}

/*!
    \brief      USART1 interrupt handler (to be called from gd32f30x_it.c)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler_Internal(void)
{
    USART1_HandlerInternal();
}

/*!
    \brief      Check if buffer is full
    \param[in]  head: buffer head index
    \param[in]  tail: buffer tail index
    \param[in]  size: buffer size
    \param[out] none
    \retval     1 if full, 0 otherwise
*/
static uint8_t USART1_BufferIsFull(uint16_t head, uint16_t tail, uint16_t size)
{
    return ((head + 1) % size) == tail;
}

/*!
    \brief      Check if buffer is empty
    \param[in]  head: buffer head index
    \param[in]  tail: buffer tail index
    \param[out] none
    \retval     1 if empty, 0 otherwise
*/
static uint8_t USART1_BufferIsEmpty(uint16_t head, uint16_t tail)
{
    return head == tail;
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
        usart_interrupt_disable(USART1_PERIPH, USART_INT_RBNE);
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
        usart_interrupt_enable(USART1_PERIPH, USART_INT_RBNE);
    }
}
