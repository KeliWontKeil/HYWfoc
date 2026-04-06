#include "i2c0.h"
#include "foc_cfg_init_values.h"

/* Private function prototypes */
static void I2C0_GPIO_Config(void);
static void I2C0_Config(void);
static void I2C0_BusyDelayLoops(uint32_t loops);
static i2c_status_t I2C0_WaitFlag(i2c_flag_enum flag, FlagStatus status, uint32_t timeout);
static i2c_status_t I2C0_WaitStopClear(uint32_t timeout);
static i2c_status_t I2C0_SendStart(void);
static i2c_status_t I2C0_SendStop(void);
static i2c_status_t I2C0_SendAddress(uint8_t address, uint32_t direction);

/* Timeout configuration (loop budget, not milliseconds). */
#define I2C_TIMEOUT_LOOPS FOC_I2C_WAIT_LOOP_MAX

void I2C0_Init(void)
{
    I2C0_GPIO_Config();
    I2C0_Config();
}

static void I2C0_GPIO_Config(void)
{
    /* Enable GPIO clock */
    rcu_periph_clock_enable(I2C0_GPIO_RCU);
    
    /* Configure SCL and SDA as alternate function open-drain */
    gpio_init(I2C0_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C0_SCL_PIN);
	gpio_init(I2C0_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C0_SDA_PIN);
}

static void I2C0_Config(void)
{
    /* Enable I2C clock */
    rcu_periph_clock_enable(I2C0_RCU);

    /* Reset I2C peripheral state before each configuration/recovery cycle. */
    i2c_disable(I2C0_PERIPH);
    i2c_software_reset_config(I2C0_PERIPH, I2C_SRESET_SET);
    i2c_software_reset_config(I2C0_PERIPH, I2C_SRESET_RESET);
    i2c_deinit(I2C0_PERIPH);

    I2C0_Unlock();
    
    /* I2C clock configure */
    i2c_clock_config(I2C0_PERIPH, I2C_SPEED, I2C_DUTYCYCLE);
    
    /* I2C address configure */
    i2c_mode_addr_config(I2C0_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x00);
    
    /* Enable I2C */
    i2c_enable(I2C0_PERIPH);
    
    /* Enable acknowledge */
    i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
}

static void I2C0_BusyDelayLoops(uint32_t loops)
{
    volatile uint32_t countdown = loops;

    while (countdown > 0U)
    {
        countdown--;
    }
}

static i2c_status_t I2C0_WaitFlag(i2c_flag_enum flag, FlagStatus status, uint32_t timeout)
{
    uint32_t loop_count = timeout;
    
    while (i2c_flag_get(I2C0_PERIPH, flag) != status)
    {
        if (loop_count == 0U)
        {
            /* Timeout means bus state may be inconsistent; recover immediately for next transaction. */
            I2C0_Config();
            return I2C_TIMEOUT;
        }

        loop_count--;
    }

    return I2C_OK;
}

static i2c_status_t I2C0_WaitStopClear(uint32_t timeout)
{
    uint32_t loop_count = timeout;

    while ((I2C_CTL0(I2C0_PERIPH) & I2C_CTL0_STOP) != 0U)
    {
        if (loop_count == 0U)
        {
            I2C0_Config();
            return I2C_TIMEOUT;
        }

        loop_count--;
    }

    return I2C_OK;
}

static i2c_status_t I2C0_SendStart(void)
{
    i2c_status_t status;
    
    /* Generate START condition */
    i2c_start_on_bus(I2C0_PERIPH);
    
    /* Wait for SB flag set */
    status = I2C0_WaitFlag(I2C_FLAG_SBSEND, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        return status;
    }
    
    return I2C_OK;
}

static i2c_status_t I2C0_SendStop(void)
{
    /* Generate STOP condition */
    i2c_stop_on_bus(I2C0_PERIPH);

    return I2C0_WaitStopClear(I2C_TIMEOUT_LOOPS);
}

static i2c_status_t I2C0_SendAddress(uint8_t address, uint32_t direction)
{
    i2c_status_t status;
    
    /* Send address */
    i2c_master_addressing(I2C0_PERIPH, address, direction);
    
    /* Wait for ADDSEND flag set */
    status = I2C0_WaitFlag(I2C_FLAG_ADDSEND, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Clear ADDSEND flag */
    i2c_flag_clear(I2C0_PERIPH, I2C_FLAG_ADDSEND);
    
    return I2C_OK;
}

i2c_status_t I2C0_CheckDevice(uint8_t device_addr)
{
    i2c_status_t status;
    
    /* Generate START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with write direction */
    status = I2C0_SendAddress(device_addr, I2C_TRANSMITTER);
    if (status != I2C_OK)
    {
        (void)I2C0_SendStop();
        return (status == I2C_TIMEOUT) ? I2C_TIMEOUT : I2C_NACK;
    }
    
    /* Generate STOP condition */
    status = I2C0_SendStop();
    if (status != I2C_OK)
    {
        return status;
    }
    
    return I2C_OK;
}

i2c_status_t I2C0_WriteByte(uint8_t device_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_status_t status;
    
    /* Generate START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with write direction */
    status = I2C0_SendAddress(device_addr, I2C_TRANSMITTER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send register address */
    i2c_data_transmit(I2C0_PERIPH, reg_addr);
    status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send data */
    i2c_data_transmit(I2C0_PERIPH, data);
    status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Generate STOP condition */
    I2C0_SendStop();
    
    return I2C_OK;
}

i2c_status_t I2C0_ReadByte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    i2c_status_t status;

    if (data == 0)
    {
        return I2C_ERROR;
    }
    
    /* Generate START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with write direction for register address */
    status = I2C0_SendAddress(device_addr, I2C_TRANSMITTER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send register address */
    i2c_data_transmit(I2C0_PERIPH, reg_addr);
    status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Generate repeated START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with read direction */
    status = I2C0_SendAddress(device_addr, I2C_RECEIVER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Disable ACK before receiving last byte */
    i2c_ack_config(I2C0_PERIPH, I2C_ACK_DISABLE);
    
    /* Generate STOP condition before reading last byte */
    i2c_stop_on_bus(I2C0_PERIPH);
    
    /* Wait for RBNE flag set */
    status = I2C0_WaitFlag(I2C_FLAG_RBNE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
        return status;
    }
    
    /* Read data */
    *data = i2c_data_receive(I2C0_PERIPH);
    
    status = I2C0_WaitStopClear(I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
        return status;
    }
    
    /* Re-enable ACK */
    i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
    
    return I2C_OK;
}

i2c_status_t I2C0_WriteBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    i2c_status_t status;
    uint16_t i;
    
    if (len == 0)
    {
        return I2C_OK;
    }
    if (data == 0)
    {
        return I2C_ERROR;
    }
    
    /* Generate START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with write direction */
    status = I2C0_SendAddress(device_addr, I2C_TRANSMITTER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send register address */
    i2c_data_transmit(I2C0_PERIPH, reg_addr);
    status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send data bytes */
    for (i = 0; i < len; i++)
    {
        i2c_data_transmit(I2C0_PERIPH, data[i]);
        status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
        if (status != I2C_OK)
        {
            I2C0_SendStop();
            return status;
        }
    }
    
    /* Generate STOP condition */
    status = I2C0_SendStop();
    if (status != I2C_OK)
    {
        return status;
    }
    
    return I2C_OK;
}

i2c_status_t I2C0_ReadBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    i2c_status_t status;
    uint16_t i;
    
    if (len == 0)
    {
        return I2C_OK;
    }
    if (data == 0)
    {
        return I2C_ERROR;
    }
    
    /* Generate START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with write direction for register address */
    status = I2C0_SendAddress(device_addr, I2C_TRANSMITTER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Send register address */
    i2c_data_transmit(I2C0_PERIPH, reg_addr);
    status = I2C0_WaitFlag(I2C_FLAG_TBE, SET, I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Generate repeated START condition */
    status = I2C0_SendStart();
    if (status != I2C_OK)
    {
        return status;
    }
    
    /* Send device address with read direction */
    status = I2C0_SendAddress(device_addr, I2C_RECEIVER);
    if (status != I2C_OK)
    {
        I2C0_SendStop();
        return status;
    }
    
    /* Read data bytes */
    for (i = 0; i < len; i++)
    {
        if (i == len - 1)
        {
            /* Last byte: disable ACK and generate STOP */
            i2c_ack_config(I2C0_PERIPH, I2C_ACK_DISABLE);
            i2c_stop_on_bus(I2C0_PERIPH);
        }
        
        /* Wait for RBNE flag set */
        status = I2C0_WaitFlag(I2C_FLAG_RBNE, SET, I2C_TIMEOUT_LOOPS);
        if (status != I2C_OK)
        {
            i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
            return status;
        }
        
        /* Read data */
        data[i] = i2c_data_receive(I2C0_PERIPH);
    }
    
    status = I2C0_WaitStopClear(I2C_TIMEOUT_LOOPS);
    if (status != I2C_OK)
    {
        i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
        return status;
    }
    
    /* Re-enable ACK */
    i2c_ack_config(I2C0_PERIPH, I2C_ACK_ENABLE);
    
    return I2C_OK;
}

void I2C0_Unlock(void)
{
    /* Generate clock pulses and a STOP sequence on open-drain GPIO to release a stuck slave. */
    uint8_t i;
    
    /* Configure SCL and SDA as GPIO open-drain outputs. */
    gpio_init(I2C0_GPIO_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, I2C0_SCL_PIN | I2C0_SDA_PIN);
    gpio_bit_set(I2C0_GPIO_PORT, I2C0_SCL_PIN | I2C0_SDA_PIN);
    I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_STOP_LOOP);
    
    /* Generate 9 clock pulses */
    for (i = 0; i < 9; i++)
    {
        gpio_bit_reset(I2C0_GPIO_PORT, I2C0_SCL_PIN);
        I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_HALF_PULSE_LOOP);
        gpio_bit_set(I2C0_GPIO_PORT, I2C0_SCL_PIN);
        I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_HALF_PULSE_LOOP);
    }
    
    /* Generate STOP condition: SDA low -> SCL high -> SDA high. */
    gpio_bit_reset(I2C0_GPIO_PORT, I2C0_SDA_PIN);
    I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_STOP_LOOP);
    gpio_bit_set(I2C0_GPIO_PORT, I2C0_SCL_PIN);
    I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_STOP_LOOP);
    gpio_bit_set(I2C0_GPIO_PORT, I2C0_SDA_PIN);
    I2C0_BusyDelayLoops(FOC_I2C_UNLOCK_STOP_LOOP);

    gpio_init(I2C0_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, I2C0_SCL_PIN | I2C0_SDA_PIN);
}
