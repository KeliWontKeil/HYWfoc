/*!
    \file    adc.c
    \brief   ADC driver implementation for current sampling in FOC motor control

    \version 2026-03-12, V1.0.0, ADC driver for GD32F30x
*/

#include "adc.h"

/* Private variables */
static uint32_t adc_buffer[ADC_BUFFER_SIZE]; /* DMA buffer: each word packs ADC0(low16) + ADC1(high16) */
static volatile uint8_t adc_initialized = 0;
static volatile uint8_t dma_complete = 0;
static float zero_offset_a = 0.0f;  /* Zero current calibration offset for phase A */
static float zero_offset_b = 0.0f;  /* Zero current calibration offset for phase B */

/* Private function prototypes */
static void ADC_GPIO_Config(void);
static void ADC_DMA_Config(void);
static void ADC_Config(void);
static uint32_t ADC_GetLatestSampleIndex(void);

static uint32_t ADC_GetLatestSampleIndex(void)
{
    uint32_t total_samples = ADC_BUFFER_SIZE;
    uint32_t remaining_samples = dma_transfer_number_get(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    uint32_t write_index = (total_samples - remaining_samples) % total_samples;

    return (write_index + total_samples - 1U) % total_samples;
}

/*!
    \brief      Initialize ADC for current sampling
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_Init(void)
{
    uint32_t i;
    uint32_t mid_raw;
    uint32_t packed_mid;

    /* Initialize all peripherals */
    ADC_GPIO_Config();
    ADC_DMA_Config();
    ADC_Config();

    /* Prefill DMA buffer to mid-scale so early reads are stable before full DMA history is collected. */
    mid_raw = (uint32_t)(ADC_MAX_VALUE / 2.0f);
    packed_mid = (mid_raw & 0xFFFFU) | ((mid_raw & 0xFFFFU) << 16U);

    for (i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        adc_buffer[i] = packed_mid;
    }
    
    /* Enable ADCs */
    adc_enable(ADC0_PERIPH);
    adc_enable(ADC1_PERIPH);
    
    /* Wait for ADC stability */
    delay_1ms(1);
    
    /* Calibrate both ADCs */
    adc_calibration_enable(ADC0_PERIPH);
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
    if (!adc_initialized) 
    {
        return;
    }

    /* Re-arm DMA channel cleanly before enabling trigger-driven sampling. */
    dma_channel_disable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    dma_transfer_number_config(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, ADC_BUFFER_SIZE);
    dma_periph_address_config(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, (uint32_t)&ADC_RDATA(ADC0_PERIPH));
    dma_memory_address_config(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, (uint32_t)adc_buffer);
    dma_flag_clear(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_FLAG_G);
    dma_interrupt_flag_clear(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_INT_FLAG_G);

    /* Clear stale ADC status flags before starting the next trigger sequence. */
    adc_flag_clear(ADC0_PERIPH, ADC_FLAG_EOC | ADC_FLAG_STRC);
    adc_flag_clear(ADC1_PERIPH, ADC_FLAG_EOC | ADC_FLAG_STRC);

    /* Enable ADC DMA request and then DMA channel. */
    adc_dma_mode_enable(ADC0_PERIPH);
    dma_channel_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    
    /* Enable external trigger for both ADCs */
    adc_external_trigger_config(ADC0_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    
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
    if (!adc_initialized) 
    {
        return;
    }
    
    /* Disable external trigger for both ADCs */
    adc_external_trigger_config(ADC0_PERIPH, ADC_ROUTINE_CHANNEL, DISABLE);
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, DISABLE);
    
    /* Disable ADC DMA */
    adc_dma_mode_disable(ADC0_PERIPH);
    
    /* Disable DMA channel */
    dma_channel_disable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
}

/*!
    \brief      Get a single sample from ADC buffer
    \param[in]  sample: pointer to sample structure
    \param[out] none
    \retval     ADC status
*/
adc_status_t ADC_GetAllSamples(adc_sample_t *sample)
{
    uint32_t latest_index;
    uint32_t packed;

    if (sample == NULL)
    {
        return ADC_STATUS_ERROR;
    }

    latest_index = ADC_GetLatestSampleIndex();
    packed = adc_buffer[latest_index];
    
    /* ADC dual routine parallel mode: ADC0 in low16, ADC1 in high16. */
    sample->phase_a_raw = (uint16_t)(packed & 0xFFFFU);
    sample->phase_b_raw = (uint16_t)((packed >> 16U) & 0xFFFFU);
    
    /* Convert to voltage */
    sample->phase_a_voltage = ADC_RawToVoltage(sample->phase_a_raw);
    sample->phase_b_voltage = ADC_RawToVoltage(sample->phase_b_raw);
    
    /* Convert to current with zero offset calibration */
    sample->phase_a_current = ADC_VoltageToCurrent(sample->phase_a_voltage) - zero_offset_a;
    sample->phase_b_current = ADC_VoltageToCurrent(sample->phase_b_voltage) - zero_offset_b;
    
    return ADC_STATUS_OK;
}

adc_status_t ADC_GetSample(float *sample, adc_sampletype_t type)
{
    uint32_t latest_index;
    uint32_t packed;
    uint16_t raw_a;
    uint16_t raw_b;

    if (sample == NULL)
    {
        return ADC_STATUS_ERROR;
    }

    latest_index = ADC_GetLatestSampleIndex();
    packed = adc_buffer[latest_index];
    raw_a = (uint16_t)(packed & 0xFFFFU);
    raw_b = (uint16_t)((packed >> 16U) & 0xFFFFU);
    
    switch (type) 
    {
        case RAW:
            /* Get raw ADC values */
            sample[0] = (float)raw_a;
            sample[1] = (float)raw_b;
            break;
        case VOLTAGE:
            /* Convert to voltage */
            sample[0] = ADC_RawToVoltage(raw_a);
            sample[1] = ADC_RawToVoltage(raw_b);
            break;
        case CURRENT:
             /* Convert to current with zero offset calibration */
            sample[0] = ADC_VoltageToCurrent(ADC_RawToVoltage(raw_a)) - zero_offset_a;
            sample[1] = ADC_VoltageToCurrent(ADC_RawToVoltage(raw_b)) - zero_offset_b;
            break;
        default:
            return ADC_STATUS_ERROR;
    }
    
    return ADC_STATUS_OK;
}

adc_status_t ADC_GetAverageSample(float *sample, adc_sampletype_t type, uint16_t count)
{
    uint32_t total_samples = ADC_BUFFER_SIZE;
    uint32_t remaining_samples;
    uint32_t write_index;
    uint32_t latest_sample_index;
    uint32_t i;
    float sum_a = 0.0f;
    float sum_b = 0.0f;

    if ((sample == NULL) || (count == 0))
    {
        return ADC_STATUS_ERROR;
    }

    if (count > ADC_BUFFER_SIZE)
    {
        count = ADC_BUFFER_SIZE;
    }

    remaining_samples = dma_transfer_number_get(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    write_index = (total_samples - remaining_samples) % total_samples;
    latest_sample_index = (write_index + total_samples - 1U) % total_samples;

    for (i = 0; i < count; i++)
    {
        uint32_t sample_index = (latest_sample_index + total_samples - i) % total_samples;
        uint32_t packed = adc_buffer[sample_index];
        uint16_t raw_a = (uint16_t)(packed & 0xFFFFU);
        uint16_t raw_b = (uint16_t)((packed >> 16U) & 0xFFFFU);

        switch (type)
        {
            case RAW:
                sum_a += (float)raw_a;
                sum_b += (float)raw_b;
                break;
            case VOLTAGE:
                sum_a += ADC_RawToVoltage(raw_a);
                sum_b += ADC_RawToVoltage(raw_b);
                break;
            case CURRENT:
                sum_a += ADC_VoltageToCurrent(ADC_RawToVoltage(raw_a)) - zero_offset_a;
                sum_b += ADC_VoltageToCurrent(ADC_RawToVoltage(raw_b)) - zero_offset_b;
                break;
            default:
                return ADC_STATUS_ERROR;
        }
    }

    sample[0] = sum_a / (float)count;
    sample[1] = sum_b / (float)count;
    return ADC_STATUS_OK;
}

/*!
    \brief      Get multiple latest samples from ADC buffer
    \param[in]  samples: pointer to samples array
    \param[in]  count: number of samples to get
    \param[out] none
    \retval     ADC status
*/
adc_status_t ADC_GetAllLatestSamples(adc_sample_t *samples, uint16_t count)
{
    uint32_t total_samples = ADC_BUFFER_SIZE;
    uint32_t latest_sample_index;
    uint32_t start_index;
    uint16_t i;

    if ((samples == NULL) || (count == 0U))
    {
        return ADC_STATUS_ERROR;
    }
    if (count > ADC_BUFFER_SIZE)
    {
        count = ADC_BUFFER_SIZE;
    }
    
    latest_sample_index = ADC_GetLatestSampleIndex();
    start_index = (latest_sample_index + total_samples - (uint32_t)(count - 1U)) % total_samples;
    
    /* Extract samples */
    for (i = 0; i < count; i++) 
    {
        uint32_t buffer_index = (start_index + i) % total_samples;
        uint32_t packed = adc_buffer[buffer_index];
        
        samples[i].phase_a_raw = (uint16_t)(packed & 0xFFFFU);
        samples[i].phase_b_raw = (uint16_t)((packed >> 16U) & 0xFFFFU);
        
        samples[i].phase_a_voltage = ADC_RawToVoltage(samples[i].phase_a_raw);
        samples[i].phase_b_voltage = ADC_RawToVoltage(samples[i].phase_b_raw);
        
        samples[i].phase_a_current = ADC_VoltageToCurrent(samples[i].phase_a_voltage) - zero_offset_a;
        samples[i].phase_b_current = ADC_VoltageToCurrent(samples[i].phase_b_voltage) - zero_offset_b;
    }
    
    return ADC_STATUS_OK;
}

adc_status_t ADC_GetLatestSample(float *sample, adc_sampletype_t type, uint16_t count)
{
    uint32_t total_samples = ADC_BUFFER_SIZE;
    uint32_t latest_sample_index;
    uint32_t start_index;
    uint32_t packed;
    uint16_t raw_a;
    uint16_t raw_b;

    if ((sample == NULL) || (count == 0U))
    {
        return ADC_STATUS_ERROR;
    }
    if (count > ADC_BUFFER_SIZE)
    {
        count = ADC_BUFFER_SIZE;
    }
    
    latest_sample_index = ADC_GetLatestSampleIndex();
    start_index = (latest_sample_index + total_samples - (uint32_t)(count - 1U)) % total_samples;
    packed = adc_buffer[start_index];
    raw_a = (uint16_t)(packed & 0xFFFFU);
    raw_b = (uint16_t)((packed >> 16U) & 0xFFFFU);

    switch (type) 
    {
        case RAW:
            /* Get raw ADC values */
            sample[0] = (float)raw_a;
            sample[1] = (float)raw_b;
            break;
        case VOLTAGE:
            /* Convert to voltage */
            sample[0] = ADC_RawToVoltage(raw_a);
            sample[1] = ADC_RawToVoltage(raw_b);
            break;
        case CURRENT:
         /* Convert to current with zero offset calibration */
            sample[0] = ADC_VoltageToCurrent(ADC_RawToVoltage(raw_a)) - zero_offset_a;
            sample[1] = ADC_VoltageToCurrent(ADC_RawToVoltage(raw_b)) - zero_offset_b;
             break;
        default:
             return ADC_STATUS_ERROR;
    }
    
    return ADC_STATUS_OK;
}

adc_status_t ADC_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b, uint16_t avg_count)
{
    float sample[2];

    if ((phase_current_a == NULL) || (phase_current_b == NULL))
    {
        return ADC_STATUS_ERROR;
    }

    if (ADC_GetAverageSample(sample, CURRENT, avg_count) != ADC_STATUS_OK)
    {
        return ADC_STATUS_ERROR;
    }

    *phase_current_a = sample[0];
    *phase_current_b = sample[1];
    return ADC_STATUS_OK;
}

uint8_t ADC_ReadPhaseCurrentABOk(float *phase_current_a, float *phase_current_b, uint16_t avg_count)
{
    return (ADC_ReadPhaseCurrentAB(phase_current_a, phase_current_b, avg_count) == ADC_STATUS_OK) ? 1U : 0U;
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
    \brief      Check if DMA transfer is complete
    \param[in]  none
    \param[out] none
    \retval     DMA status
*/
adc_status_t ADC_DMA_IsComplete(void)
{
    if (dma_flag_get(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_FLAG_FTF)) 
    {
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
    
    dma_init_struct.periph_addr  = (uint32_t)&ADC_RDATA(ADC0_PERIPH);
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)adc_buffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.number       = ADC_BUFFER_SIZE;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    
    dma_init(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, &dma_init_struct);
    
    /* Enable DMA circular mode */
    dma_circulation_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL);
    
    /* Enable DMA transfer complete interrupt */
    dma_interrupt_enable(ADC_DMA_PERIPH, ADC_DMA_CHANNEL, DMA_INT_FTF);

    /* Enable DMA channel IRQ routing so DMA completion state is observable. */
    nvic_irq_enable(DMA0_Channel0_IRQn, ADC_DMA_PRIORITY_GROUP, ADC_DMA_PRIORITY_SUBGROUP);
}

/*!
    \brief      Configure ADC for synchronous sampling
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void ADC_Config(void)
{
    /* Enable ADC clocks */
    rcu_periph_clock_enable(ADC0_RCU);
    rcu_periph_clock_enable(ADC1_RCU);
    
    /* Reset ADCs */
    adc_deinit(ADC0_PERIPH);
    adc_deinit(ADC1_PERIPH);
    
    /* ADC mode config: ADC0 master, ADC1 slave for synchronous sampling */
    adc_mode_config(ADC_DAUL_ROUTINE_PARALLEL);
    
    /* ADC special function: enable scan mode for both ADCs */
    adc_special_function_config(ADC0_PERIPH, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1_PERIPH, ADC_SCAN_MODE, ENABLE);
    
    /* ADC data alignment: right alignment for both */
    adc_data_alignment_config(ADC0_PERIPH, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1_PERIPH, ADC_DATAALIGN_RIGHT);
    
    /* ADC resolution: 12-bit for both */
    adc_resolution_config(ADC0_PERIPH, ADC_RESOLUTION);
    adc_resolution_config(ADC1_PERIPH, ADC_RESOLUTION);
    
    /* Configure regular channel sequence for ADC0 */
    adc_routine_channel_config(ADC0_PERIPH, 0, ADC_CHANNEL_PA6, ADC_SAMPLE_TIME);
    
    /* Configure regular channel sequence for ADC1 */
    adc_routine_channel_config(ADC1_PERIPH, 0, ADC_CHANNEL_PA7, ADC_SAMPLE_TIME);
    
    /* Set regular channel sequence length (1 channel each) */
    adc_channel_length_config(ADC0_PERIPH, ADC_ROUTINE_CHANNEL, 1);
    adc_channel_length_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, 1);
    
    /* Configure external trigger source: TIMER3 CH3 compare event for both ADCs. */
    adc_external_trigger_source_config(ADC0_PERIPH, ADC_ROUTINE_CHANNEL, ADC0_1_EXTTRIG_ROUTINE_T3_CH3);
    adc_external_trigger_source_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ADC0_1_EXTTRIG_ROUTINE_T3_CH3);
    
    /* Enable external trigger for both ADCs */
    adc_external_trigger_config(ADC0_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC1_PERIPH, ADC_ROUTINE_CHANNEL, ENABLE);
    
    /* Enable DMA request for ADC0 regular channel */
    adc_dma_mode_enable(ADC0_PERIPH);

    /* Enable ADC0 end-of-conversion interrupt to observe trigger timing on GPIO. */
    adc_interrupt_enable(ADC0_PERIPH, ADC_INT_EOC);
    nvic_irq_enable(ADC0_1_IRQn, ADC0_1_PRIORITY_GROUP, ADC0_1_PRIORITY_SUBGROUP);
}

/*!
    \brief      DMA interrupt handler for ADC (internal)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ADC_DMA_IRQHandler_Internal(void)
{
    if (dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF)) 
    {
        dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);
        dma_complete = 1;
    }
}

void ADC_IRQHandler_Internal(void)
{
    if (adc_interrupt_flag_get(ADC0_PERIPH, ADC_INT_FLAG_EOC) != RESET)
    {
        adc_interrupt_flag_clear(ADC0_PERIPH, ADC_INT_FLAG_EOC);
    }
}
