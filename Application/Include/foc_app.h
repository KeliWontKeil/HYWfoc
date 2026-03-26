#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdio.h>

#include "foc_platform_api.h"
#include "control_scheduler.h"
#include "foc_control.h"
#include "foc_control_init.h"
#include "sensor.h"
#include "uart_debug.h"

#define FOC_APP_PWM_FREQ_KHZ                24U
#define FOC_APP_SENSOR_SAMPLE_FREQ_KHZ      FOC_APP_PWM_FREQ_KHZ
#define FOC_APP_CONTROL_LOOP_HZ             1000.0f
#define FOC_APP_CONTROL_DT_SEC              (1.0f / FOC_APP_CONTROL_LOOP_HZ)
#define FOC_APP_TARGET_ANGLE_RAD            3.14f
#define FOC_APP_ANGLE_POSITION_SPEED_RAD_S  16.0f
#define FOC_APP_MAIN_LOOP_DELAY_MS          200U

void FOC_App_Init(void);
void FOC_App_Start(void);
void FOC_App_Loop(void);

#endif /* FOC_APP_H */
