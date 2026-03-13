#include "usart.h"

/* Private variables */
static uint8_t rx_buffer[USART1_RX_BUFFER_SIZE];
static uint8_t tx_buffer[USART1_TX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_busy = 0;
static volatile uint8_t loopback_enabled = 0;
static usart_rx_callback_t rx_callback = NULL;

/* Private function prototypes */
static uint8_t USART1_BufferIsFull(uint16_t head, uint16_t tail, uint16_t size);
static uint8_t USART1_BufferIsEmpty(uint16_t head, uint16_t tail);
static void USART1_EnableInterrupts(void);
static void USART1_DisableInterrupts(void);

/*!
    \brief      Initialize USART1 for loopback operation
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
    gpio_init(USART1_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, USART1_RX_PIN);
    
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
    
    /* Configure NVIC for USART1 */
    NVIC_CONFIG(USART1_IRQn, USART1_PRIORITY_GROUP, USART1_PRIORITY_SUBGROUP);
    
    /* Initially enable loopback */
    USART1_LoopbackEnable();
}

/*!
    \brief      Enable loopback mode (echo received data)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_LoopbackEnable(void)
{
    loopback_enabled = 1;
}

/*!
    \brief      Disable loopback mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_LoopbackDisable(void)
{
    loopback_enabled = 0;
}

/*!
    \brief      Send a byte via USART1
    \param[in]  data: byte to send
    \param[out] none
    \retval     status of operation
*/
usart_status_t USART1_SendByte(uint8_t data)
{
    USART1_DisableInterrupts();
    
    /* Check if TX buffer is full */
    if (USART1_BufferIsFull(tx_head, tx_tail, USART1_TX_BUFFER_SIZE)) {
        USART1_EnableInterrupts();
        return USART_STATUS_BUFFER_FULL;
    }
    
    /* Add data to TX buffer */
    tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) % USART1_TX_BUFFER_SIZE;
    
    /* If transmitter is idle, start transmission */
    if (!tx_busy) {
        tx_busy = 1;
        usart_interrupt_enable(USART1_PERIPH, USART_INT_TBE);
    }
    
    USART1_EnableInterrupts();
    return USART_STATUS_OK;
}

/*!
    \brief      Send a string via USART1
    \param[in]  str: null-terminated string to send
    \param[out] none
    \retval     status of operation
*/
usart_status_t USART1_SendString(const char *str)
{
    usart_status_t status;
    
    while (*str) {
        status = USART1_SendByte(*str++);
        if (status != USART_STATUS_OK) {
            return status;
        }
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
    
    if (!USART1_BufferIsEmpty(rx_head, rx_tail)) {
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
    tx_head = 0;
    tx_tail = 0;
    tx_busy = 0;
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(tx_buffer, 0, sizeof(tx_buffer));
    
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
    if (usart_interrupt_flag_get(USART1_PERIPH, USART_INT_FLAG_RBNE) != RESET) {
        uint8_t data = usart_data_receive(USART1_PERIPH);
        
        /* Add to RX buffer */
        if (!USART1_BufferIsFull(rx_head, rx_tail, USART1_RX_BUFFER_SIZE)) {
            rx_buffer[rx_head] = data;
            rx_head = (rx_head + 1) % USART1_RX_BUFFER_SIZE;
            
            /* Call callback if set */
            if (rx_callback != NULL) {
                rx_callback(data);
            }
            
            /* Loopback: echo received data */
            if (loopback_enabled) {
                USART1_SendByte(data);
            }
        }
        
        usart_interrupt_flag_clear(USART1_PERIPH, USART_INT_FLAG_RBNE);
    }
    
    /* Transmit buffer empty interrupt */
    if (usart_interrupt_flag_get(USART1_PERIPH, USART_INT_FLAG_TBE) != RESET) {
        if (!USART1_BufferIsEmpty(tx_head, tx_tail)) {
            /* Send next byte */
            usart_data_transmit(USART1_PERIPH, tx_buffer[tx_tail]);
            tx_tail = (tx_tail + 1) % USART1_TX_BUFFER_SIZE;
        } else {
            /* No more data to send, disable TBE interrupt */
            usart_interrupt_disable(USART1_PERIPH, USART_INT_TBE);
            tx_busy = 0;
        }
        
        usart_interrupt_flag_clear(USART1_PERIPH, USART_INT_FLAG_TBE);
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

/*!
    \brief      Disable USART1 interrupts (for critical sections)
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void USART1_DisableInterrupts(void)
{
    usart_interrupt_disable(USART1_PERIPH, USART_INT_RBNE);
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
    usart_interrupt_enable(USART1_PERIPH, USART_INT_RBNE);
    if (tx_busy) {
        usart_interrupt_enable(USART1_PERIPH, USART_INT_TBE);
    }
}
