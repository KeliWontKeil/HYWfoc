/*!
    \file    adc.c
    \brief   ADC driver implementation for current sampling in FOC motor control

    \version 2026-03-12, V1.0.0, ADC driver for GD32F30x
*/

#include "adc.h"

/* Private variables */
static uint16_t adc_buffer[ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT]; /* DMA buffer: interleaved samples */
static volatile uint8_t adc_initialized = 0;
static volatile uint8_t dma_complete = 0;
static float zero_offset_a = 0.0f;  /* Zero current calibration offset for phase A */
static float zero_offset_b = 0.0f;  /* Zero current calibration offset for phase B */

/* Private function prototypes */
static void ADC_GPIO_Config(void);
static void ADC_DMA_Config(void);
static void ADC_Config(void);

/*!
    \brief      Initialize ADC for current sampling
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_Init(void)
{
    /* Initialize all peripherals */
    ADC_GPIO_Config();
    ADC_DMA_Config();
    ADC_Config();
    
    /* Enable ADC */
    adc_enable(ADC1_PERIPH);
    
    /* Wait for ADC stability */
    delay_1ms(1);
    
    /* Calibrate ADC */
    adc_calibration_enable(ADC1_PERIPH);
    
    /* Clear DMA complete flag */
    dma_complete = 0;
    
    /* Mark as initialized */
    adc_initialized = 1;
}

/*!
    \brief      Start ADC sampling with DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_Start(void)
{
    if (!adc_initialized) {
        return;
    }
    
    /* Clear DMA interrupt flag */
    dma_interrupt_flag_clear(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_INT_FLAG_FTF);
    
    /* Enable DMA channel */
    dma_channel_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    
    /* Enable ADC DMA */
    adc_dma_mode_enable(ADC1_PERIPH);
    
    /* Enable external trigger */
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    
    /* Start ADC */
    adc_software_trigger_enable(ADC1_PERIPH, ADC_ROUTINE_CHANNEL);
    
    /* Reset DMA complete flag */
    dma_complete = 0;
}

/*!
    \brief      Stop ADC sampling
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_Stop(void)
{
    if (!adc_initialized) {
        return;
    }
    
    /* Disable external trigger */
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, DISABLE);
    
    /* Disable ADC DMA */
    adc_dma_mode_disable(ADC1_PERIPH);
    
    /* Disable DMA channel */
    dma_channel_disable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
}

/*!
    \brief      Get a single sample from ADC buffer
    \param[in]  sample: pointer to sample structure
    \param[out] none
    \retval     ADC status
*/
adc_status_t ADC_GetSample(adc_sample_t *sample)
{
    uint32_t latest_index;
    
    if (sample == NULL) {
        return ADC_STATUS_ERROR;
    }
    
    /* Calculate index of latest sample (assuming circular buffer) */
    latest_index = (ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT - 2) % (ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT);
    
    /* Get raw ADC values */
    sample->phase_a_raw = adc_buffer[latest_index];
    sample->phase_b_raw = adc_buffer[latest_index + 1];
    
    /* Convert to voltage */
    sample->phase_a_voltage = ADC_RawToVoltage(sample->phase_a_raw);
    sample->phase_b_voltage = ADC_RawToVoltage(sample->phase_b_raw);
    
    /* Convert to current with zero offset calibration */
    sample->phase_a_current = ADC_VoltageToCurrent(sample->phase_a_voltage) - zero_offset_a;
    sample->phase_b_current = ADC_VoltageToCurrent(sample->phase_b_voltage) - zero_offset_b;
    
    return ADC_STATUS_OK;
}

/*!
    \brief      Get multiple latest samples from ADC buffer
    \param[in]  samples: pointer to samples array
    \param[in]  count: number of samples to get
    \param[out] none
    \retval     ADC status
*/
adc_status_t ADC_GetLatestSamples(adc_sample_t *samples, uint16_t count)
{
    uint32_t start_index;
    uint16_t i;
    
    if (samples == NULL || count == 0) {
        return ADC_STATUS_ERROR;
    }
    
    /* Limit count to available samples */
    if (count > ADC_BUFFER_SIZE) {
        count = ADC_BUFFER_SIZE;
    }
    
    /* Calculate start index (assuming circular buffer, getting most recent samples) */
    start_index = (ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT - 2 * count) % (ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT);
    
    /* Extract samples */
    for (i = 0; i < count; i++) {
        uint32_t buffer_index = start_index + i * ADC_CHANNEL_COUNT;
        
        samples[i].phase_a_raw = adc_buffer[buffer_index];
        samples[i].phase_b_raw = adc_buffer[buffer_index + 1];
        
        samples[i].phase_a_voltage = ADC_RawToVoltage(samples[i].phase_a_raw);
        samples[i].phase_b_voltage = ADC_RawToVoltage(samples[i].phase_b_raw);
        
        samples[i].phase_a_current = ADC_VoltageToCurrent(samples[i].phase_a_voltage) - zero_offset_a;
        samples[i].phase_b_current = ADC_VoltageToCurrent(samples[i].phase_b_voltage) - zero_offset_b;
    }
    
    return ADC_STATUS_OK;
}

/*!
    \brief      Convert raw ADC value to voltage
    \param[in]  raw_value: raw ADC value (0-4095)
    \param[out] none
    \retval     voltage in volts
*/
float ADC_RawToVoltage(uint16_t raw_value)
{
    return (float)raw_value * ADC_VREF / ADC_MAX_VALUE;
}

/*!
    \brief      Convert voltage to current
    \param[in]  voltage: voltage in volts
    \param[out] none
    \retval     current in amperes
*/
float ADC_VoltageToCurrent(float voltage)
{
    /* Current sensor: 0A = VREF/2, ±20A range */
    return (voltage - ADC_ZERO_CURRENT_VOLTAGE) * CURRENT_SCALE_FACTOR;
}

/*!
    \brief      Convert raw ADC value directly to current
    \param[in]  raw_value: raw ADC value (0-4095)
    \param[out] none
    \retval     current in amperes
*/
float ADC_RawToCurrent(uint16_t raw_value)
{
    float voltage = ADC_RawToVoltage(raw_value);
    return ADC_VoltageToCurrent(voltage);
}

/*!
    \brief      Calibrate zero current offset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_CalibrateZeroOffset(void)
{
    adc_sample_t sample;
    uint16_t i;
    float sum_a = 0.0f, sum_b = 0.0f;
    const uint16_t calibration_samples = 100;
    
    /* Get multiple samples for averaging */
    for (i = 0; i < calibration_samples; i++) {
        if (ADC_GetSample(&sample) == ADC_STATUS_OK) {
            sum_a += sample.phase_a_current;
            sum_b += sample.phase_b_current;
        }
        delay_1ms(1);
    }
    
    /* Calculate average as zero offset */
    zero_offset_a = sum_a / calibration_samples;
    zero_offset_b = sum_b / calibration_samples;
}

/*!
    \brief      Check if DMA transfer is complete
    \param[in]  none
    \param[out] none
    \retval     DMA status
*/
adc_status_t ADC_DMA_IsComplete(void)
{
    if (dma_flag_get(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_FLAG_FTF)) {
        dma_complete = 1;
        return ADC_STATUS_OK;
    }
    return ADC_STATUS_DMA_ERROR;
}

/*!
    \brief      Set ADC trigger frequency
    \param[in]  frequency_hz: trigger frequency in Hz
    \param[out] none
    \retval     none
*/
void ADC_SetTriggerFrequency(uint32_t frequency_hz)
{
    /* Note: Trigger frequency is controlled by TIMER0 CH0
       This function should be coordinated with PWM frequency setup
       Currently, ADC uses TIMER0_CH0 as trigger source */
    (void)frequency_hz; /* Parameter not used in current implementation */
}

/*!
    \brief      Configure GPIO for ADC channels
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void ADC_GPIO_Config(void)
{
    /* Enable GPIO clock */
    rcu_periph_clock_enable(ADC_GPIO_PA6_RCU);
    rcu_periph_clock_enable(ADC_GPIO_PA7_RCU);
    
    /* Configure PA6 and PA7 as analog inputs */
    gpio_init(ADC_GPIO_PA6_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, ADC_GPIO_PA6_PIN);
    gpio_init(ADC_GPIO_PA7_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, ADC_GPIO_PA7_PIN);
}

/*!
    \brief      Configure DMA for ADC data transfer
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void ADC_DMA_Config(void)
{
    dma_parameter_struct dma_init_struct;
    
    /* Enable DMA clock */
    rcu_periph_clock_enable(ADC_DMA_RCU);
    
    /* Initialize DMA channel */
    dma_deinit(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    
    /* Initialize DMA struct with default values */
    dma_struct_para_init(&dma_init_struct);
    
    dma_init_struct.periph_addr  = (uint32_t)&ADC_RDATA(ADC1_PERIPH);
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)adc_buffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.number       = ADC_BUFFER_SIZE * ADC_CHANNEL_COUNT;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    
    dma_init(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, &dma_init_struct);
    
    /* Enable DMA circular mode */
    dma_circulation_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    
    /* Enable DMA transfer complete interrupt */
    dma_interrupt_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_INT_FTF);
}

/*!
    \brief      Configure ADC for synchronous sampling
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void ADC_Config(void)
{
    /* Enable ADC clock */
    rcu_periph_clock_enable(ADC1_RCU);
    
    /* Reset ADC */
    adc_deinit(ADC1_PERIPH);
    
    /* ADC mode config: independent mode (could be changed to sync mode if needed) */
    adc_mode_config(ADC_MODE_FREE);
    
    /* ADC special function: enable scan mode */
    adc_special_function_config(ADC1_PERIPH, ADC_SCAN_MODE, ENABLE);
    
    /* ADC data alignment: right alignment */
    adc_data_alignment_config(ADC1_PERIPH, ADC_DATAALIGN_RIGHT);
    
    /* ADC resolution: 12-bit */
    adc_resolution_config(ADC1_PERIPH, ADC_RESOLUTION);
    
    /* Configure regular channel sequence */
    adc_routine_channel_config(ADC1_PERIPH, 0, ADC_CHANNEL_PA6, ADC_SAMPLE_TIME);
    adc_routine_channel_config(ADC1_PERIPH, 1, ADC_CHANNEL_PA7, ADC_SAMPLE_TIME);
    
    /* Set regular channel sequence length */
    adc_channel_length_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ADC_CHANNEL_COUNT);
    
    /* Configure external trigger */
    adc_external_trigger_source_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ADC_EXTERNAL_TRIGGER);
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    
    /* Enable DMA request for regular channel */
    adc_dma_mode_enable(ADC1_PERIPH);
}

/*!
    \brief      DMA interrupt handler for ADC (internal)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_DMA_IRQHandler_Internal(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);
        dma_complete = 1;
    }
}
