/*!
    \file    adc.h
    \brief   ADC driver for current sampling in FOC motor control

    \version 2026-03-12, V1.0.0, ADC driver for GD32F30x
*/

#ifndef _ADC_H_
#define _ADC_H_

#include "gd32f30x.h"
#include "interrupt_priority.h"
#include "systick.h"
#include <stddef.h>
#include <string.h>

/* ADC peripheral definitions */
#define ADC0_PERIPH           ADC0
#define ADC0_RCU              RCU_ADC0
#define ADC1_PERIPH           ADC1
#define ADC1_RCU              RCU_ADC1

/* ADC channel definitions */
#define ADC_CHANNEL_PA6       ADC_CHANNEL_6     /* Phase A current */
#define ADC_CHANNEL_PA7       ADC_CHANNEL_7     /* Phase B current */

/* ADC GPIO definitions */
#define ADC_GPIO_PA6_RCU      RCU_GPIOA
#define ADC_GPIO_PA6_PORT     GPIOA
#define ADC_GPIO_PA6_PIN      GPIO_PIN_6

#define ADC_GPIO_PA7_RCU      RCU_GPIOA
#define ADC_GPIO_PA7_PORT     GPIOA
#define ADC_GPIO_PA7_PIN      GPIO_PIN_7

#define FOC_ISR_VIS_ADC_DMA_GPIO_RCU RCU_GPIOB
#define FOC_ISR_VIS_ADC_DMA_GPIO_PORT GPIOB
#define FOC_ISR_VIS_ADC_DMA_GPIO_PIN GPIO_PIN_8

/* ADC configuration */
#define ADC_SAMPLE_TIME       ADC_SAMPLETIME_55POINT5  /* Maximum resolution */
#define ADC_RESOLUTION        ADC_RESOLUTION_12B       /* 12-bit resolution */
#define ADC_EXTERNAL_TRIGGER  ADC0_1_EXTTRIG_ROUTINE_T3_CH3  /* TIMER3 CH3 trigger */
#define ADC_REGULAR_CHANNEL   ADC_ROUTINE_CHANNEL      /* Routine channel (regular channel) */

/* DMA configuration */
#define ADC_DMA_PERIPH        DMA0
#define ADC_DMA_CHANNEL       DMA_CH0
#define ADC_DMA_RCU           RCU_DMA0

/* Buffer configuration */
#define ADC_BUFFER_SIZE       24     /* DMA buffer size (samples per channel) */
#define ADC_CHANNEL_COUNT     2       /* Number of channels: PA6 and PA7 */
#define ADC_AVG_DEFAULT_COUNT 24      /* 24kHz sampling / 1kHz control loop */

/* Current calculation constants */
#define ADC_VREF              3.3f    /* Reference voltage (V) */
#define ADC_MAX_VALUE         4095.0f /* 12-bit ADC max value */
#define ADC_ZERO_CURRENT_VOLTAGE (ADC_VREF / 2.0f) /* Voltage at zero current */
#define CURRENT_RANGE 20.0f /* ±20A current range corresponding to 0 to VREF/2 deviation */
#define CURRENT_SCALE_FACTOR  (CURRENT_RANGE / (ADC_VREF / 2.0f)) /* 20A range, ±VREF/2 */

/* Data structures */
typedef struct {
    uint16_t phase_a_raw;     /* Raw ADC value for phase A */
    uint16_t phase_b_raw;     /* Raw ADC value for phase B */
    float phase_a_current;    /* Calculated current for phase A (A) */
    float phase_b_current;    /* Calculated current for phase B (A) */
    float phase_a_voltage;    /* Calculated voltage for phase A (V) */
    float phase_b_voltage;    /* Calculated voltage for phase B (V) */
} adc_sample_t;

typedef enum {
    ADC_STATUS_OK = 0,
    ADC_STATUS_ERROR,
    ADC_STATUS_NOT_INITIALIZED,
    ADC_STATUS_DMA_ERROR
} adc_status_t;

typedef enum {
    RAW = 0,
    VOLTAGE,
    CURRENT
} adc_sampletype_t;

/* Function prototypes */
void ADC_Init(void);
void ADC_Start(void);
void ADC_Stop(void);
adc_status_t ADC_GetSample(float *sample, adc_sampletype_t type);
adc_status_t ADC_GetAverageSample(float *sample, adc_sampletype_t type, uint16_t count);
adc_status_t ADC_GetAllSamples(adc_sample_t *sample);
adc_status_t ADC_GetLatestSample(float *sample, adc_sampletype_t type, uint16_t count);
adc_status_t ADC_GetAllLatestSamples(adc_sample_t *samples, uint16_t count);
adc_status_t ADC_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b, uint16_t avg_count);
uint8_t ADC_ReadPhaseCurrentABOk(float *phase_current_a, float *phase_current_b, uint16_t avg_count);

float ADC_RawToVoltage(uint16_t raw_value);
float ADC_VoltageToCurrent(float voltage);
float ADC_RawToCurrent(uint16_t raw_value);
adc_status_t ADC_DMA_IsComplete(void);
void ADC_SetTriggerFrequency(uint32_t frequency_hz);

/* DMA interrupt handler (called from ISR) */
void ADC_DMA_IRQHandler_Internal(void);
void ADC_IRQHandler_Internal(void);

#endif /* _ADC_H_ */


