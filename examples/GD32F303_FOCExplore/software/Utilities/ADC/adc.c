/*!
    \file    adc.c
    \brief   ADC driver implementation for current sampling in FOC motor control

    \version 2026-03-12, V1.0.0, ADC driver for GD32F30x
*/

#include "adc.h"

/* Private variables */
static uint32_t adc_buffer[ADC_BUFFER_SIZE]; /* DMA buffer: each word packs ADC0(low16) + ADC1(high16) */
static volatile uint8_t adc_initialized = 0;
/* Zero-offset correction is at L3 (sensor) via kalman_filter_t.zero_offset. */

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
    uint32_t i;
    uint32_t mid_raw;
    uint32_t packed_mid;

    /* Initialize all peripherals */
    ADC_GPIO_Config();
    ADC_DMA_Config();
    ADC_Config();

#if (FOC_ISR_VIS_ADC_DMA_TOGGLE_ENABLE == 1U)
    rcu_periph_clock_enable(FOC_ISR_VIS_ADC_DMA_GPIO_RCU);
    gpio_init(FOC_ISR_VIS_ADC_DMA_GPIO_PORT,
              GPIO_MODE_OUT_PP,
              GPIO_OSPEED_50MHZ,
              FOC_ISR_VIS_ADC_DMA_GPIO_PIN);
    gpio_bit_reset(FOC_ISR_VIS_ADC_DMA_GPIO_PORT, FOC_ISR_VIS_ADC_DMA_GPIO_PIN);
#endif

    /* Prefill DMA buffer to mid-scale so early reads are stable before full DMA history is collected. */
    mid_raw = (uint32_t)(ADC_15_MAX_VALUE / 2.0f);
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
                sum_a += ADC_CurrentRawToVoltage(raw_a);
                sum_b += ADC_CurrentRawToVoltage(raw_b);
                break;
            case CURRENT:
                sum_a += ADC_VoltageToCurrentPhaseA(ADC_CurrentRawToVoltage(raw_a)) * CURRENT_DIR_A;
                sum_b += ADC_VoltageToCurrentPhaseB(ADC_CurrentRawToVoltage(raw_b)) * CURRENT_DIR_B;
                break;
            default:
                return ADC_STATUS_ERROR;
        }
    }

    sample[0] = sum_a / (float)count;
    sample[1] = sum_b / (float)count;
    return ADC_STATUS_OK;
}

adc_status_t ADC_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b, uint16_t avg_count)
{
    float sample[2];

    if ((phase_current_a == NULL) || (phase_current_b == NULL))
    {
        return ADC_STATUS_ERROR;
    }

    if (avg_count == 0U)
    {
        avg_count = 1U;
    }
    else if (avg_count > ADC_BUFFER_SIZE)
    {
        avg_count = ADC_BUFFER_SIZE;
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
    \brief      Convert raw ADC value to voltage (current sampling, 15-bit)
    \param[in]  raw_value: raw ADC value (0-32760 for 15-bit oversampled)
    \param[out] none
    \retval     voltage in volts
*/
float ADC_CurrentRawToVoltage(uint16_t raw_value)
{
    return (float)raw_value * ADC_VREF / ADC_15_MAX_VALUE;
}

/*!
    \brief      Convert voltage to current for phase A
    \param[in]  voltage: voltage in volts
    \param[out] none
    \retval     current in amperes
*/
float ADC_VoltageToCurrentPhaseA(float voltage)
{
    return (voltage - ADC_ZERO_CURRENT_VOLTAGE_A) * CURRENT_SCALE_FACTOR_A;
}

/*!
    \brief      Convert voltage to current for phase B
    \param[in]  voltage: voltage in volts
    \param[out] none
    \retval     current in amperes
*/
float ADC_VoltageToCurrentPhaseB(float voltage)
{
    return (voltage - ADC_ZERO_CURRENT_VOLTAGE_B) * CURRENT_SCALE_FACTOR_B;
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
    
    /* ADC oversampling: 8x oversampling, no shift, all conversions consecutively.
       8 accumulated 12-bit samples yield 15-bit effective range (0~32760). */
    adc_oversample_mode_config(ADC0_PERIPH, ADC_OVERSAMPLING_ALL_CONVERT,
                               ADC_OVERSAMPLING_SHIFT_NONE, ADC_OVERSAMPLING_RATIO_MUL8);
    adc_oversample_mode_config(ADC1_PERIPH, ADC_OVERSAMPLING_ALL_CONVERT,
                               ADC_OVERSAMPLING_SHIFT_NONE, ADC_OVERSAMPLING_RATIO_MUL8);
    adc_oversample_mode_enable(ADC0_PERIPH);
    adc_oversample_mode_enable(ADC1_PERIPH);
    
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
/*
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
#if (FOC_ISR_VIS_ADC_DMA_TOGGLE_ENABLE == 1U)
        if (gpio_output_bit_get(FOC_ISR_VIS_ADC_DMA_GPIO_PORT, FOC_ISR_VIS_ADC_DMA_GPIO_PIN) != RESET)
        {
            gpio_bit_reset(FOC_ISR_VIS_ADC_DMA_GPIO_PORT, FOC_ISR_VIS_ADC_DMA_GPIO_PIN);
        }
        else
        {
            gpio_bit_set(FOC_ISR_VIS_ADC_DMA_GPIO_PORT, FOC_ISR_VIS_ADC_DMA_GPIO_PIN);
        }
#endif
    }
}

void ADC_IRQHandler_Internal(void)
{
    if (adc_interrupt_flag_get(ADC0_PERIPH, ADC_INT_FLAG_EOC) != RESET)
    {
        adc_interrupt_flag_clear(ADC0_PERIPH, ADC_INT_FLAG_EOC);
    }
}

/*
 * VBUS voltage divider ratio: VBUS = Vadc / 0.1935
 * (derived from: Vadc = VBUS * (R2 / (R1 + R2)), where R1+R2 divider gives ~0.1935)
 */

static uint8_t g_adc2_initialized = 0U;

static void ADC2_GPIO_Config(void)
{
    rcu_periph_clock_enable(ADC_GPIO_PA1_RCU);
    gpio_init(ADC_GPIO_PA1_PORT, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, ADC_GPIO_PA1_PIN);
}

static void ADC2_Config(void)
{
    rcu_periph_clock_enable(ADC2_RCU);
    adc_deinit(ADC2_PERIPH);

    adc_data_alignment_config(ADC2_PERIPH, ADC_DATAALIGN_RIGHT);
    adc_resolution_config(ADC2_PERIPH, ADC_RESOLUTION_12B);

    /* Single channel, software trigger, no scan mode. */
    adc_channel_length_config(ADC2_PERIPH, ADC_ROUTINE_CHANNEL, 1U);
    adc_routine_channel_config(ADC2_PERIPH, 0U, ADC_CHANNEL_PA1, ADC_SAMPLETIME_55POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC2, ADC_ROUTINE_CHANNEL, ADC0_1_2_EXTTRIG_ROUTINE_NONE); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC2, ADC_ROUTINE_CHANNEL, ENABLE);
    adc_software_trigger_enable(ADC2, ADC_ROUTINE_CHANNEL);

    /* Enable ADC2. */
    adc_enable(ADC2_PERIPH);
    delay_1ms(1U);
    adc_calibration_enable(ADC2_PERIPH);
}

uint8_t ADC2_ReadVbus(float *vbus_v)
{
    uint32_t loop_count;
    uint16_t raw_value;
    float vbus;

    if (vbus_v == 0)
    {
        return 0U;
    }

    if (g_adc2_initialized == 0U)
    {
        ADC2_GPIO_Config();
        ADC2_Config();
        g_adc2_initialized = 1U;
    }

    /* Software trigger regular conversion. */
    adc_software_trigger_enable(ADC2_PERIPH, ADC_ROUTINE_CHANNEL);

    /* Wait for EOC with timeout. */
    loop_count = 0U;
    while (adc_flag_get(ADC2_PERIPH, ADC_FLAG_EOC) == RESET)
    {
        loop_count++;
        if (loop_count >= ADC2_VBUS_EOC_TIMEOUT_LOOPS)
        {
            return 0U; /* Timeout */
        }
    }
    adc_flag_clear(ADC2_PERIPH, ADC_FLAG_EOC);

    raw_value = (uint16_t)adc_routine_data_read(ADC2_PERIPH);
    vbus = ((float)raw_value / ADC_12_MAX_VALUE) * ADC_VREF * ADC2_VBUS_CONVERSION_K;

    if (vbus < 0.0f)
    {
        vbus = 0.0f;
    }

    *vbus_v = vbus;
    return 1U;
}
