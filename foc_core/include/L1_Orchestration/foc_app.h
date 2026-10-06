#ifndef FOC_APP_H
#define FOC_APP_H

#include "LS_Config/foc_config.h"
#include "L2_Core/foc_ctrl_types.h"

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

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
/* 声学回报触发（协议 A 组与 L1 内部事件共用的单一收口；含注入互斥与音域/幅值收敛） */
uint8_t FOC_App_PlayTune(uint8_t tune_id);
void    FOC_App_StopTune(void);
#endif

#endif /* FOC_APP_H */
