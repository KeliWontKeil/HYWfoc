#ifndef _PWM_H_
#define _PWM_H_

/*!
    \file    pwm.h
    \brief   PWM module for complementary PWM output using TIMER0

    \version 2026-3-9, V1.0.0, Three-channel complementary PWM
*/

#include "gd32f30x.h"

/* =========================================================================
 * 相 -> TIMER0 通道映射（板级功率级接线，唯一真值；换板/改接线只改这一段）
 * -------------------------------------------------------------------------
 * 相标签由电流采样链定义：adc.h 的 ADC_CHANNEL_PHASE_A/B 即 current_a/current_b，
 * 第三相由 ic = -(ia + ib) 重建。PWM 输出腿必须与"采样相"是同一物理相，否则
 * 测量链（Clarke/Park）与施加链（逆Park/SVPWM）的 αβ 帧互为镜像，表现为 iq
 * 以 2 倍电频率正弦波动（幅值 = 相电流幅值）且 SMO 无法收敛。
 * 取值 = TIMER0 硬件通道索引：
 *   0 = CH0(PA8 / PWMA)    1 = CH1(PA9 / PWMB)    2 = CH2(PA10 / PWMC)
 * 三相须取 0/1/2 的一个排列，非法配置由下方编译期检查拦截。
 * ========================================================================= */
#define PWM_PHASE_A_CHANNEL   2     /* 相A（current_a 所在相）驱动腿：CH2 / PA10 / PWMC */
#define PWM_PHASE_B_CHANNEL   1     /* 相B（current_b 所在相）驱动腿：CH1 / PA9  / PWMB */
#define PWM_PHASE_C_CHANNEL   0     /* 相C（重建相）驱动腿：       CH0 / PA8  / PWMA */

#if ((PWM_PHASE_A_CHANNEL > 2) || (PWM_PHASE_B_CHANNEL > 2) || (PWM_PHASE_C_CHANNEL > 2))
#error "PWM phase channel index must be 0 (TIMER_CH_0), 1 (TIMER_CH_1) or 2 (TIMER_CH_2)"
#endif
#if ((PWM_PHASE_A_CHANNEL == PWM_PHASE_B_CHANNEL) || (PWM_PHASE_A_CHANNEL == PWM_PHASE_C_CHANNEL) || (PWM_PHASE_B_CHANNEL == PWM_PHASE_C_CHANNEL))
#error "PWM phase channel mapping must be a permutation of 0/1/2 (one distinct leg per phase)"
#endif

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

/* 默认占空比（按相，0.0 = 全关）：PWM_Init 按相映射写入对应驱动腿 */
#define PWM_DEFAULT_DUTY_PHASE_A    0.0f
#define PWM_DEFAULT_DUTY_PHASE_B    0.0f
#define PWM_DEFAULT_DUTY_PHASE_C    0.0f

/* 死区无独立宏：唯一配置源为 foc_cfg_init_values.h 的 FOC_SVPWM_DEADTIME_PERCENT_DEFAULT
 * （周期百分比），经 FOC_Platform_PWMInit 传入 PWM_Init，模块内只有这一处设置点。 */

/* PWM channel enumeration：TIMER0 硬件通道索引（非 FOC 相号，相->通道映射见 PWM_PHASE_*_CHANNEL） */
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
void PWM_SetUpdateInterruptEnabled(uint8_t enable);
/* 单通道占空比（硬件通道索引，初始化/调试用；FOC 控制路径用 TripleFloat） */
void PWM_SetDutyCycle(pwm_channel_t channel, uint8_t duty_percent);
void PWM_SetDutyCycleFloat(pwm_channel_t channel, float duty);
/* 三相占空比（FOC 相坐标 0.0~1.0）：按 PWM_PHASE_*_CHANNEL 映射到 TIMER0 通道 */
void PWM_SetDutyCycleTripleFloat(float duty_a, float duty_b, float duty_c);
uint8_t PWM_GetDutyCycle(pwm_channel_t channel);
/* 低层死区接口（定时器时钟周期，0~255）；配置入口是 PWM_Init 的周期百分比参数 */
void PWM_SetDeadTime(uint16_t dead_time_cycles);
void PWM_EnableComplementaryOutputs(void);
void PWM_DisableComplementaryOutputs(void);
void PWM_SetUpdateCallback(pwm_update_callback_t callback);
void PWM_Timer0Update_IRQHandler_Internal(void);

#endif /* _PWM_H_ */
