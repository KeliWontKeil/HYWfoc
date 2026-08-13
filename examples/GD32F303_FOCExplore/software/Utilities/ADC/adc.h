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
#define ADC2_PERIPH           ADC2
#define ADC2_RCU              RCU_ADC2

/* ADC channel definitions */
#define ADC_CHANNEL_PHASE_A   ADC_CHANNEL_0     /* Phase A current, PA0 */
#define ADC_CHANNEL_PHASE_B   ADC_CHANNEL_1     /* Phase B current, PA1 */
#define CURRENT_DIR_A         -1.0f             /* Phase A current direction multiplier */
#define CURRENT_DIR_B         -1.0f             /* Phase B current direction multiplier */

#define ADC_CHANNEL_VBUS      ADC_CHANNEL_3     /* VBUS voltage (ADC2), PA3 */

/* ADC GPIO definitions */
#define ADC_GPIO_PHASE_A_RCU  RCU_GPIOA
#define ADC_GPIO_PHASE_A_PORT GPIOA
#define ADC_GPIO_PHASE_A_PIN  GPIO_PIN_0

#define ADC_GPIO_PHASE_B_RCU  RCU_GPIOA
#define ADC_GPIO_PHASE_B_PORT GPIOA
#define ADC_GPIO_PHASE_B_PIN  GPIO_PIN_1

#define ADC_GPIO_VBUS_RCU     RCU_GPIOA
#define ADC_GPIO_VBUS_PORT    GPIOA
#define ADC_GPIO_VBUS_PIN     GPIO_PIN_3


#define FOC_ISR_VIS_ADC_DMA_TOGGLE_ENABLE 1U
#define FOC_ISR_VIS_ADC_DMA_GPIO_RCU RCU_GPIOC
#define FOC_ISR_VIS_ADC_DMA_GPIO_PORT GPIOC
#define FOC_ISR_VIS_ADC_DMA_GPIO_PIN GPIO_PIN_13

#define ADC2_VBUS_DIVIDER_RATIO 0.1935f
#define ADC2_VBUS_CONVERSION_K  (1.0f / ADC2_VBUS_DIVIDER_RATIO)
#define ADC2_VBUS_EOC_TIMEOUT_LOOPS 10000U

/* ADC configuration */
#define ADC_SAMPLE_TIME       ADC_SAMPLETIME_13POINT5  /* Maximum resolution */
#define ADC_RESOLUTION        ADC_RESOLUTION_12B       /* 12-bit resolution */
#define ADC_EXTERNAL_TRIGGER  ADC0_1_EXTTRIG_ROUTINE_T3_CH3  /* TIMER3 CH3 trigger */
#define ADC_REGULAR_CHANNEL   ADC_ROUTINE_CHANNEL      /* Routine channel (regular channel) */

/* DMA configuration */
#define ADC_DMA_PERIPH        DMA0
#define ADC_DMA_CHANNEL       DMA_CH0
#define ADC_DMA_RCU           RCU_DMA0

/*
 * DMA circular buffer size (number of samples per channel).
 *
 * Must be >= FOC_SENSOR_ADC_AVG_COUNT_SLOW (typically 24 = PWM_FREQ_KHZ * 1000 / CONTROL_HZ)
 * to avoid silent clamp in ADC_GetAverageSample().  64 provides ample margin for
 * higher PWM frequencies or slower control rates.
 */
#define ADC_BUFFER_SIZE       64U
#define ADC_CHANNEL_COUNT     2       /* Number of channels: PA0 (A) and PA1 (B) */

/* Current calculation constants */
#define ADC_VREF              3.28f    /* Reference voltage (V) */
#define ADC_15_MAX_VALUE         4095.0f /* 12-bit ADC max value (oversampling disabled) */
#define ADC_12_MAX_VALUE           4095.0f /* 12-bit ADC max value */

/* Phase A current sensor parameters */
#define ADC_ZERO_CURRENT_VOLTAGE_A 1.64f  /* Voltage at zero current, phase A */
#define CURRENT_RANGE_A            16.5f               /* ±16.5A current range, phase A */
#define CURRENT_SCALE_FACTOR_A     (CURRENT_RANGE_A / (ADC_VREF / 2.0f))

/* Phase B current sensor parameters */
#define ADC_ZERO_CURRENT_VOLTAGE_B 1.64f  /* Voltage at zero current, phase B */
#define CURRENT_RANGE_B            16.5f               /* ±16.5A current range, phase B */
#define CURRENT_SCALE_FACTOR_B     (CURRENT_RANGE_B / (ADC_VREF / 2.0f))

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
adc_status_t ADC_GetAverageSample(float *sample, adc_sampletype_t type, uint16_t count);
adc_status_t ADC_ReadPhaseCurrentAB(float *phase_current_a, float *phase_current_b, uint16_t avg_count);
uint8_t ADC_ReadPhaseCurrentABOk(float *phase_current_a, float *phase_current_b, uint16_t avg_count);

float ADC_CurrentRawToVoltage(uint16_t raw_value);
float ADC_VoltageToCurrentPhaseA(float voltage);
float ADC_VoltageToCurrentPhaseB(float voltage);

/* DMA interrupt handler (called from ISR) */
void ADC_DMA_IRQHandler_Internal(void);
void ADC_IRQHandler_Internal(void);

/* ADC2 VBUS voltage sampling (software trigger, EOC polling). */
uint8_t ADC2_ReadVbus(float *vbus_v);


#endif /* _ADC_H_ */
