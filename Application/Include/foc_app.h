#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdio.h>

#include "foc_platform_api.h"
#include "control_scheduler.h"
#include "foc_control.h"
#include "sensor.h"
#include "svpwm.h"
#include "uart_debug.h"

void FOC_App_Init(void);
void FOC_App_Start(void);
void FOC_App_Loop(void);

#endif /* FOC_APP_H */
