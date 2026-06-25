#ifndef FOC_APP_H
#define FOC_APP_H

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

#endif /* FOC_APP_H */
