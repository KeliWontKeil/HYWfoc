/*!
    \file    uart_debug.c
    \brief   UART debug module implementation for outputting motor control debug information

    \version 2026-03-13, V1.0.0, UART debug for GD32F303CC FOC project
*/

#include "uart_debug.h"

/* Private function prototypes */
static void UART_Debug_SendFormattedFloat(const char *label, float value, const char *unit);

/*!
    \brief      Initialize UART debug module
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART_Debug_Init(void)
{
    /* UART debug module initialized - USART1 should already be initialized in main */
}

/*!
    \brief      Output current measurements via UART
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART_Debug_OutputCurrent(void)
{
    adc_sample_t sample;

    if (ADC_GetAllSamples(&sample) == ADC_STATUS_OK)
    {
        UART_Debug_SendFormattedFloat("Phase A Current", sample.phase_a_current, "A");
        UART_Debug_SendFormattedFloat("Phase B Current", sample.phase_b_current, "A");
    }
    else
    {
        USART1_SendString("ADC: Error reading current samples\r\n");
    }
}

/*!
    \brief      Output encoder angle via UART
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART_Debug_OutputEncoderAngle(void)
{
    uint16_t angle_raw;

    //magnet_status = AS5600_CheckMagnet();
    //if (magnet_status == AS5600_MAGNET_OK) {
        if (AS5600_ReadAngle(&angle_raw) == I2C_OK)
        {
            float angle_deg = (float)angle_raw * AS5600_ANGLE_TO_DEGREE;
            UART_Debug_SendFormattedFloat("Encoder Angle", angle_deg, "deg");
        }
        else
        {
            USART1_SendString("AS5600: Error reading angle\r\n");
            I2C0_Unlock();
        }
    /*} else {
        USART1_SendString("AS5600: Magnet status error\r\n");
    }*/
}

/*!
    \brief      Output all debug information via UART
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART_Debug_OutputAll(void)
{
    sensor_data_t* sensor = Sensor_GetData();
    
    /* Output filtered sensor data */
    if (sensor->adc_valid)
    {
        UART_Debug_SendFormattedFloat("Current A (filtered)", sensor->current_a.filtered_value, "A");
        UART_Debug_SendFormattedFloat("Current B (filtered)", sensor->current_b.filtered_value, "A");
        UART_Debug_SendFormattedFloat("Current C (filtered)", sensor->current_c.filtered_value, "A");
    }
    else
    {
        USART1_SendString("ADC: Data not valid\r\n");
    }
    
    if (sensor->encoder_valid)
    {
        UART_Debug_SendFormattedFloat("Encoder Angle (filtered)", sensor->angle_degrees.filtered_value, "deg");
    }
    else
    {
        USART1_SendString("Encoder: Data not valid\r\n");
    }
    
    /* Output algorithm execution time */
    uint32_t exec_time = Timer1_GetExecutionTime();
    char buffer[64];
    sprintf(buffer, "Algorithm execution time: %lf us\r\n", exec_time / 120.0f);
    USART1_SendString(buffer);
    USART1_SendString("\r\n");
}

/*!
    \brief      Send formatted float value via UART
    \param[in]  label: label string
    \param[in]  value: float value to send
    \param[in]  unit: unit string
    \param[out] none
    \retval     none
*/
static void UART_Debug_SendFormattedFloat(const char *label, float value, const char *unit)
{
    char buffer[64];
    sprintf(buffer, "%s: %.3f %s\r\n", label, value, unit);
    USART1_SendString(buffer);
}

/*!
    \brief      Output data in oscilloscope format (binary format for visualization tools like Serial Oscilloscope or Serial Plotter)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UART_Debug_OutputOscilloscope(void)
{
    sensor_data_t* sensor = Sensor_GetData();
    uint8_t buffer[20];  /* Buffer for binary data */
    uint8_t index = 0;

    /* Start marker (0xAA) */
    buffer[index++] = 0xAA;

    /* Pack current values as 16-bit integers (scaled by 1000 for precision) */
    int16_t current_a = (int16_t)(sensor->current_a.filtered_value * 1000.0f);
    int16_t current_b = (int16_t)(sensor->current_b.filtered_value * 1000.0f);
    int16_t current_c = (int16_t)(sensor->current_c.filtered_value * 1000.0f);

    buffer[index++] = (uint8_t)(current_a & 0xFF);
    buffer[index++] = (uint8_t)((current_a >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(current_b & 0xFF);
    buffer[index++] = (uint8_t)((current_b >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(current_c & 0xFF);
    buffer[index++] = (uint8_t)((current_c >> 8) & 0xFF);

    /* Pack encoder angle as 16-bit integer (scaled by 100 for precision) */
    int16_t angle = (int16_t)(sensor->angle_degrees.filtered_value * 100.0f);
    buffer[index++] = (uint8_t)(angle & 0xFF);
    buffer[index++] = (uint8_t)((angle >> 8) & 0xFF);

    /* Pack algorithm execution time as 16-bit integer (in microseconds) */
    uint32_t exec_time_us = Timer1_GetExecutionTime() / 120;  /* Convert to microseconds */
    int16_t exec_time = (int16_t)exec_time_us;
    buffer[index++] = (uint8_t)(exec_time & 0xFF);
    buffer[index++] = (uint8_t)((exec_time >> 8) & 0xFF);

    /* End marker (0x55) */
    buffer[index++] = 0x55;

    /* Send binary data */
    USART1_SendString(buffer);
}
