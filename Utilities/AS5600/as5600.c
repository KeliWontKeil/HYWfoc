#include "as5600.h"

/* Private function prototypes */
static uint16_t AS5600_CombineBytes(uint8_t high_byte, uint8_t low_byte);
static i2c_status_t AS5600_ReadRegister16(uint8_t reg_high, uint16_t *value);

void AS5600_Init(void)
{
    I2C0_Init();
}

static uint16_t AS5600_CombineBytes(uint8_t high_byte, uint8_t low_byte)
{
    return (uint16_t)((high_byte << 8) | low_byte);
}

static i2c_status_t AS5600_ReadRegister16(uint8_t reg_high, uint16_t *value)
{
    uint8_t buffer[2];
    i2c_status_t status;

    status = I2C0_ReadBytes(AS5600_I2C_ADDRESS, reg_high, buffer, 2);
    if (status != I2C_OK)
    {
        return status;
    }

    *value = AS5600_CombineBytes(buffer[0], buffer[1]) & 0x0FFF;
    return I2C_OK;
}

as5600_magnet_status_t AS5600_CheckMagnet(void)
{
    uint8_t status_reg;
    i2c_status_t status;
    
    status = AS5600_ReadStatus(&status_reg);
    if (status != I2C_OK)
    {
        return AS5600_MAGNET_NOT_DETECTED;
    }
    
    if (!(status_reg & AS5600_STATUS_MD))
    {
        return AS5600_MAGNET_NOT_DETECTED;
    }
    
    if (status_reg & AS5600_STATUS_MH)
    {
        return AS5600_MAGNET_TOO_STRONG;
    }
    
    if (status_reg & AS5600_STATUS_ML)
    {
        return AS5600_MAGNET_TOO_WEAK;
    }
    
    return AS5600_MAGNET_OK;
}

i2c_status_t AS5600_ReadRawAngle(uint16_t *angle)
{
    return AS5600_ReadRegister16(AS5600_REG_RAW_ANGLE_H, angle);
}

i2c_status_t AS5600_ReadAngle(uint16_t *angle)
{
    return AS5600_ReadRegister16(AS5600_REG_ANGLE_H, angle);
}

i2c_status_t AS5600_ReadMagnitude(uint16_t *magnitude)
{
    return AS5600_ReadRegister16(AS5600_REG_MAGNITUDE_H, magnitude);
}

i2c_status_t AS5600_ReadStatus(uint8_t *status)
{
    return I2C0_ReadByte(AS5600_I2C_ADDRESS, AS5600_REG_STATUS, status);
}

i2c_status_t AS5600_ReadAGC(uint8_t *agc)
{
    return I2C0_ReadByte(AS5600_I2C_ADDRESS, AS5600_REG_AGC, agc);
}

i2c_status_t AS5600_ReadAll(as5600_data_t *data)
{
    i2c_status_t status;
    
    /* Keep ReadAll for debug/diagnosis, but reuse independent lightweight readers. */
    status = AS5600_ReadRawAngle(&data->raw_angle);
    if (status != I2C_OK)
    {
        return status;
    }
    
    status = AS5600_ReadAngle(&data->angle);
    if (status != I2C_OK)
    {
        return status;
    }
    
    status = AS5600_ReadMagnitude(&data->magnitude);
    if (status != I2C_OK)
    {
        return status;
    }
    
    status = AS5600_ReadStatus(&data->status);
    if (status != I2C_OK)
    {
        return status;
    }
    
    status = AS5600_ReadAGC(&data->agc);
    if (status != I2C_OK)
    {
        return status;
    }
    
    data->angle_deg = (float)data->angle * AS5600_ANGLE_TO_DEGREE;
    data->angle_rad = (float)data->angle * AS5600_ANGLE_TO_RADIAN;
    
    return I2C_OK;
}

float AS5600_RawAngleToDegrees(uint16_t angle)
{
    return (float)angle * AS5600_ANGLE_TO_DEGREE;
}

i2c_status_t AS5600_SetStartPosition(uint16_t start_pos)
{
    uint8_t buffer[2];
    
    if (start_pos > AS5600_MAX_ANGLE)
    {
        start_pos = AS5600_MAX_ANGLE;
    }
    
    buffer[0] = (uint8_t)((start_pos >> 8) & 0x0F);  /* High 4 bits */
    buffer[1] = (uint8_t)(start_pos & 0xFF);         /* Low 8 bits */
    
    return I2C0_WriteBytes(AS5600_I2C_ADDRESS, AS5600_REG_ZPOS_H, buffer, 2);
}

i2c_status_t AS5600_SetStopPosition(uint16_t stop_pos)
{
    uint8_t buffer[2];
    
    if (stop_pos > AS5600_MAX_ANGLE)
    {
        stop_pos = AS5600_MAX_ANGLE;
    }
    
    buffer[0] = (uint8_t)((stop_pos >> 8) & 0x0F);  /* High 4 bits */
    buffer[1] = (uint8_t)(stop_pos & 0xFF);         /* Low 8 bits */
    
    return I2C0_WriteBytes(AS5600_I2C_ADDRESS, AS5600_REG_MPOS_H, buffer, 2);
}

i2c_status_t AS5600_SetMaxAngle(uint16_t max_angle)
{
    uint8_t buffer[2];
    
    if (max_angle > AS5600_MAX_ANGLE)
    {
        max_angle = AS5600_MAX_ANGLE;
    }
    
    buffer[0] = (uint8_t)((max_angle >> 8) & 0x0F);  /* High 4 bits */
    buffer[1] = (uint8_t)(max_angle & 0xFF);         /* Low 8 bits */
    
    return I2C0_WriteBytes(AS5600_I2C_ADDRESS, AS5600_REG_MANG_H, buffer, 2);
}

i2c_status_t AS5600_Configure(uint8_t power_mode, uint8_t hysteresis, 
                              uint8_t output_stage, uint8_t pwm_freq,
                              uint8_t slow_filter, uint8_t fast_filter)
{
    uint8_t conf_high, conf_low;
    
    /* Configure high byte */
    conf_high = 0;
    conf_high |= (power_mode & 0x03) << 0;   /* PM[1:0] - bits 0-1 */
    conf_high |= (hysteresis & 0x03) << 2;   /* HYST[1:0] - bits 2-3 */
    conf_high |= (output_stage & 0x01) << 4; /* OUTS[0] - bit 4 */
    conf_high |= (pwm_freq & 0x03) << 5;     /* PWMF[1:0] - bits 5-6 */
    
    /* Configure low byte */
    conf_low = 0;
    conf_low |= (slow_filter & 0x03) << 0;   /* SF[1:0] - bits 0-1 */
    conf_low |= (fast_filter & 0x07) << 2;   /* FTH[2:0] - bits 2-4 */
    
    /* Write configuration */
    i2c_status_t status = I2C0_WriteByte(AS5600_I2C_ADDRESS, AS5600_REG_CONF_H, conf_high);
    if (status != I2C_OK)
    {
        return status;
    }
    
    return I2C0_WriteByte(AS5600_I2C_ADDRESS, AS5600_REG_CONF_L, conf_low);
}
