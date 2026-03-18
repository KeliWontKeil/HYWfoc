#include "usart2.h"
#include "interrupt_priority.h"
#include <string.h>

/* Private variables */
static uint8_t rx_buffer[USART2_RX_BUFFER_SIZE];
static uint8_t tx_buffer[USART2_TX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;
static usart2_rx_callback_t rx_callback = NULL;

/* Private function prototypes */
static uint8_t USART2_BufferIsFull(uint16_t head, uint16_t tail, uint16_t size);
static uint8_t USART2_BufferIsEmpty(uint16_t head, uint16_t tail);
static void USART2_EnableInterrupts(void);
static void USART2_DisableInterrupts(void);
static void USART2_HandlerInternal(void);
static uint8_t USART2_ProtocolChecksum(const uint8_t *data, uint8_t len);

/*!
    \brief      Initialize USART2 for motor control parameter communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_Init(void)
{
    /* Enable GPIO and USART clocks */
    rcu_periph_clock_enable(USART2_GPIO_RCU);
    rcu_periph_clock_enable(USART2_RCU);
    
    /* Configure GPIO pins for USART2 */
    gpio_init(USART2_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, USART2_TX_PIN);
    gpio_init(USART2_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, USART2_RX_PIN);
    
    /* USART parameter configuration */
    usart_deinit(USART2_PERIPH);
    usart_baudrate_set(USART2_PERIPH, USART2_BAUDRATE);
    usart_word_length_set(USART2_PERIPH, USART2_WORD_LENGTH);
    usart_stop_bit_set(USART2_PERIPH, USART2_STOP_BITS);
    usart_parity_config(USART2_PERIPH, USART2_PARITY);
    usart_hardware_flow_rts_config(USART2_PERIPH, USART2_HARDWARE_FLOW);
    usart_hardware_flow_cts_config(USART2_PERIPH, USART_CTS_DISABLE);

    usart_receive_config(USART2_PERIPH, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2_PERIPH, USART_TRANSMIT_ENABLE);
    
    /* Enable USART */
    usart_enable(USART2_PERIPH);
    
    /* Clear buffers */
    USART2_ClearBuffers();
    
    /* Enable USART receive interrupt */
    usart_interrupt_enable(USART2_PERIPH, USART_INT_RBNE);
    
    /* Configure NVIC for USART2 */
    NVIC_CONFIG(USART2_IRQn, USART2_PRIORITY_GROUP, USART2_PRIORITY_SUBGROUP);
}

/*!
    \brief      Send a byte via USART2
    \param[in]  data: byte to send
    \param[out] none
    \retval     status of operation
*/
usart2_status_t USART2_SendByte(uint8_t data)
{
    USART2_DisableInterrupts();
    
    /* Check if TX buffer is full */
    if (USART2_BufferIsFull(tx_head, tx_tail, USART2_TX_BUFFER_SIZE))
    {
        USART2_EnableInterrupts();
        return USART2_STATUS_BUFFER_FULL;
    }
    
    /* Add data to TX buffer */
    tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) % USART2_TX_BUFFER_SIZE;
    
    /* If transmitter is idle, start transmission */
    if (!tx_busy)
    {
        tx_busy = 1;
        usart_interrupt_enable(USART2_PERIPH, USART_INT_TBE);
    }
    
    USART2_EnableInterrupts();
    return USART2_STATUS_OK;
}

/*!
    \brief      Send a string via USART2
    \param[in]  str: null-terminated string to send
    \param[out] none
    \retval     status of operation
*/
usart2_status_t USART2_SendString(const char *str)
{
    usart2_status_t status;
    
    while (*str)
    {
        status = USART2_SendByte(*str++);
        if (status != USART2_STATUS_OK)
        {
            return status;
        }
    }
    
    return USART2_STATUS_OK;
}

usart2_status_t USART2_SendData(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL)
    {
        return USART2_STATUS_ERROR;
    }

    for (i = 0; i < len; i++)
    {
        usart2_status_t status = USART2_SendByte(data[i]);
        if (status == USART2_STATUS_OK)
        {
            continue;
        }

        if (status == USART2_STATUS_BUFFER_FULL)
        {
            __NOP();
            i--;
            continue;
        }

        return status;
    }

    return USART2_STATUS_OK;
}

/*!
    \brief      Receive a byte from USART2 buffer
    \param[in]  none
    \param[out] none
    \retval     received byte (0 if buffer empty)
*/
uint8_t USART2_ReceiveByte(void)
{
    uint8_t data = 0;
    
    USART2_DisableInterrupts();
    
    if (!USART2_BufferIsEmpty(rx_head, rx_tail))
    {
        data = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % USART2_RX_BUFFER_SIZE;
    }
    
    USART2_EnableInterrupts();
    
    return data;
}

uint16_t USART2_ReadBuffer(uint8_t *buffer, uint16_t max_len)
{
    uint16_t read_len = 0;

    if (buffer == NULL)
    {
        return 0;
    }

    while ((read_len < max_len) && USART2_IsDataAvailable())
    {
        buffer[read_len++] = USART2_ReceiveByte();
    }

    return read_len;
}

/*!
    \brief      Check if data is available in RX buffer
    \param[in]  none
    \param[out] none
    \retval     1 if data available, 0 otherwise
*/
uint8_t USART2_IsDataAvailable(void)
{
    return !USART2_BufferIsEmpty(rx_head, rx_tail);
}

/*!
    \brief      Clear USART2 buffers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_ClearBuffers(void)
{
    USART2_DisableInterrupts();
    
    rx_head = 0;
    rx_tail = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_busy = 0;
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(tx_buffer, 0, sizeof(tx_buffer));
    
    USART2_EnableInterrupts();
}

/*!
    \brief      Set RX callback function
    \param[in]  callback: function pointer to call when data received
    \param[out] none
    \retval     none
*/
void USART2_SetRxCallback(usart2_rx_callback_t callback)
{
    rx_callback = callback;
}

/*!
    \brief      USART2 interrupt handler implementation
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART2_HandlerInternal(void)
{
    /* Receive buffer not empty interrupt */
    if (usart_interrupt_flag_get(USART2_PERIPH, USART_INT_FLAG_RBNE) != RESET)
    {
        uint8_t data = usart_data_receive(USART2_PERIPH);
        
        /* Add to RX buffer */
        if (!USART2_BufferIsFull(rx_head, rx_tail, USART2_RX_BUFFER_SIZE))
        {
            rx_buffer[rx_head] = data;
            rx_head = (rx_head + 1) % USART2_RX_BUFFER_SIZE;
            
            /* Call callback if set */
            if (rx_callback != NULL)
            {
                rx_callback(data);
            }
        }
        
        usart_interrupt_flag_clear(USART2_PERIPH, USART_INT_FLAG_RBNE);
    }
    
    /* Transmit buffer empty interrupt */
    if (usart_interrupt_flag_get(USART2_PERIPH, USART_INT_FLAG_TBE) != RESET)
    {
        if (!USART2_BufferIsEmpty(tx_head, tx_tail))
        {
            /* Send next byte */
            usart_data_transmit(USART2_PERIPH, tx_buffer[tx_tail]);
            tx_tail = (tx_tail + 1) % USART2_TX_BUFFER_SIZE;
        }
        else
        {
            /* No more data to send, disable TBE interrupt */
            usart_interrupt_disable(USART2_PERIPH, USART_INT_TBE);
            tx_busy = 0;
        }
        
        usart_interrupt_flag_clear(USART2_PERIPH, USART_INT_FLAG_TBE);
    }
}

/*!
    \brief      USART2 interrupt handler (to be called from gd32f30x_it.c)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void)
{
    USART2_HandlerInternal();
}

usart2_status_t USART2_ProtocolSendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[2 + 1 + 1 + USART2_PROTOCOL_MAX_PAYLOAD + 1 + 1];
    uint8_t checksum_input[1 + 1 + USART2_PROTOCOL_MAX_PAYLOAD];
    uint16_t index = 0;
    uint8_t i;

    if ((len > USART2_PROTOCOL_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
    {
        return USART2_STATUS_ERROR;
    }

    frame[index++] = USART2_PROTOCOL_SOF0;
    frame[index++] = USART2_PROTOCOL_SOF1;
    frame[index++] = len;
    frame[index++] = cmd;

    checksum_input[0] = len;
    checksum_input[1] = cmd;

    for (i = 0; i < len; i++)
    {
        frame[index++] = payload[i];
        checksum_input[2U + i] = payload[i];
    }

    frame[index++] = USART2_ProtocolChecksum(checksum_input, (uint8_t)(2U + len));
    frame[index++] = USART2_PROTOCOL_EOF;

    return USART2_SendData(frame, index);
}

uint16_t USART2_ProtocolReadRaw(uint8_t *buffer, uint16_t max_len)
{
    return USART2_ReadBuffer(buffer, max_len);
}

uint8_t USART2_ProtocolTryParse(usart2_protocol_frame_t *frame)
{
    (void)frame;
    /* Reserved for future protocol parser implementation. */
    return 0;
}

/*!
    \brief      Check if buffer is full
    \param[in]  head: buffer head index
    \param[in]  tail: buffer tail index
    \param[in]  size: buffer size
    \param[out] none
    \retval     1 if full, 0 otherwise
*/
static uint8_t USART2_BufferIsFull(uint16_t head, uint16_t tail, uint16_t size)
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
static uint8_t USART2_BufferIsEmpty(uint16_t head, uint16_t tail)
{
    return head == tail;
}

/*!
    \brief      Disable USART2 interrupts (for critical sections)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART2_DisableInterrupts(void)
{
    usart_interrupt_disable(USART2_PERIPH, USART_INT_RBNE);
    usart_interrupt_disable(USART2_PERIPH, USART_INT_TBE);
}

/*!
    \brief      Enable USART2 interrupts
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART2_EnableInterrupts(void)
{
    usart_interrupt_enable(USART2_PERIPH, USART_INT_RBNE);
    if (tx_busy)
    {
        usart_interrupt_enable(USART2_PERIPH, USART_INT_TBE);
    }
}

static uint8_t USART2_ProtocolChecksum(const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}
