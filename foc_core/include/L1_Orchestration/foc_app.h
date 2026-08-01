#ifndef FOC_APP_H
#define FOC_APP_H

#include "LS_Config/foc_config.h"

/* 顶层应用入口 */
void FOC_App_Init(void);
void FOC_App_Start(void);
void FOC_App_Loop(void);

/* 调度器回调桥接（注册到 ControlScheduler_SetCallback） */
void FOC_App_ServiceTrigger(void);
void FOC_App_ControlTrigger(void);
void FOC_App_MonitorTrigger(void);

/* PWM ISR 桥接（注册到 FOC_Platform_SetPwmUpdateCallback） */
void FOC_App_OnPwmUpdateISR(void);

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
/* 电流环 ISR 桥接（注册到 FOC_Platform_AuxTimerInit） */
void FOC_App_OnCurrentLoopISR(void);
#endif

/* 特殊控制状态退出（由协议 Y:A 或自动退出守卫调用） */
void FOC_App_AbortSpecialPhase(void);

#endif /* FOC_APP_H */
