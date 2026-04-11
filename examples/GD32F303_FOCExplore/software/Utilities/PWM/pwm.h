#ifndef _PWM_H_
#define _PWM_H_

/*!
    \file    pwm.h
    \brief   PWM module for complementary PWM output using TIMER0

    \version 2026-3-9, V1.0.0, Three-channel complementary PWM
*/

#include "gd32f30x.h"

/* TIMER0 pin definitions (from Hardware.md) */
/* Main output channels */
#define PWM_TIMER0_CH0_PIN          GPIO_PIN_8
#define PWM_TIMER0_CH0_GPIO         GPIOA
#define PWM_TIMER0_CH0_RCU          RCU_GPIOA

#define PWM_TIMER0_CH1_PIN          GPIO_PIN_9
#define PWM_TIMER0_CH1_GPIO         GPIOA
#define PWM_TIMER0_CH1_RCU          RCU_GPIOA

#define PWM_TIMER0_CH2_PIN          GPIO_PIN_10
#define PWM_TIMER0_CH2_GPIO         GPIOA
#define PWM_TIMER0_CH2_RCU          RCU_GPIOA

/* Complementary output channels */
#define PWM_TIMER0_CH0N_PIN         GPIO_PIN_13
#define PWM_TIMER0_CH0N_GPIO        GPIOB
#define PWM_TIMER0_CH0N_RCU         RCU_GPIOB

#define PWM_TIMER0_CH1N_PIN         GPIO_PIN_14
#define PWM_TIMER0_CH1N_GPIO        GPIOB
#define PWM_TIMER0_CH1N_RCU         RCU_GPIOB

#define PWM_TIMER0_CH2N_PIN         GPIO_PIN_15
#define PWM_TIMER0_CH2N_GPIO        GPIOB
#define PWM_TIMER0_CH2N_RCU         RCU_GPIOB

/* Timer peripheral */
#define PWM_TIMER0_PERIPH           TIMER0
#define PWM_TIMER0_RCU              RCU_TIMER0

/* PWM Configuration */
#define PWM_TIMER_CLOCK_HZ          120000000U /* System clock: 120MHz */

/* Default duty cycles (50% for all channels) */
#define PWM_DEFAULT_DUTY_CH0        0U  /* percentage */
#define PWM_DEFAULT_DUTY_CH1        0U
#define PWM_DEFAULT_DUTY_CH2        0U

/* Dead time configuration (optional, in timer clock cycles) */
#define PWM_DEAD_TIME               10U  /* Approx 0.5us at 20kHz clock */

/* PWM channel enumeration */
typedef enum {
    PWM_CHANNEL_0 = 0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_COUNT
} pwm_channel_t;

typedef void (*pwm_update_callback_t)(void);

/* Function prototypes */
void PWM_Init(uint8_t freq_kHz,uint8_t deadtime_percent);
void PWM_Start(void);
void PWM_Stop(void);
void PWM_SetDutyCycle(pwm_channel_t channel, uint8_t duty_percent);
void PWM_SetDutyCycleFloat(pwm_channel_t channel, float duty);
void PWM_SetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);
uint8_t PWM_GetDutyCycle(pwm_channel_t channel);
void PWM_SetDeadTime(uint16_t dead_time_cycles);
void PWM_EnableComplementaryOutputs(void);
void PWM_DisableComplementaryOutputs(void);
void PWM_SetUpdateCallback(pwm_update_callback_t callback);
void PWM_Timer0Update_IRQHandler_Internal(void);

#endif /* _PWM_H_ */
