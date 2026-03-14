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
    sprintf(buffer, "Algorithm execution time: %lu cycles\r\n", exec_time);
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