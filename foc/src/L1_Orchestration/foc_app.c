#include "L1_Orchestration/foc_app.h"

#include <stdio.h>

#include "L1_Orchestration/control_scheduler.h"
#include "L2_Service/debug_stream.h"
#include "L2_Service/runtime_c1_entry.h"
#include "L2_Service/motor_control_service.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define FOC_APP_COMM_FRAMES_PER_STEP 1U  /* 每步处理1帧通讯 */

static void Service_Task_Trigger(void);
static void Motor_Control_Loop(void);
static void Monitor_Task_Trigger(void);
static void FOC_App_OnPwmUpdateISR(void);
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data);
static void FOC_App_StopFastCurrentLoop(void);
static void FOC_App_ReInitMotor(void);
static void FOC_App_InitMotorHardware(void);
static void FOC_App_EnterSafeOutputState(uint8_t report_skip);

static foc_motor_t g_motor;                                          /* 电机状态实例 */
static sensor_data_t g_sensor_snapshot;                              /* 传感器快照（主控周期更新） */
static sensor_data_t g_fast_current_sensor_snapshot;                 /* 电流环传感器快照（ISR内更新） */
static foc_pid_t g_torque_current_pid;                               /* 转矩电流PID实例 */
static foc_pid_t g_angle_pid;                                        /* 角度保持PID实例 */
static foc_pid_t g_speed_pid;                                        /* 速度PID实例 */
static runtime_snapshot_t g_StateSnapshot;                           /* 运行时快照（参数+状态） */

static volatile uint8_t g_service_task_pending = 0U;                 /* 服务任务挂起标志 */
static volatile uint8_t g_monitor_task_pending = 0U;                 /* 监控任务挂起标志 */

static volatile uint8_t g_fast_current_loop_enabled = 0U;            /* 快速电流环启用标志 */
static volatile uint8_t g_fast_current_loop_div_counter = 0U;        /* 电流环分频计数器 */
static volatile float g_fast_current_loop_iq_target = 0.0f;          /* 电流环IQ目标值[A] */
static volatile float g_fast_current_loop_electrical_angle = 0.0f;   /* 电流环电角度[rad] */

static volatile uint8_t g_reinit_in_progress = 0U;                   /* 电机重初始化进行中标志 */
static uint16_t g_led_comm_pulse_counter = 0U;                       /* 通讯LED脉冲计数器 */

/* 更新状态指示灯 */
static void FOC_App_UpdateIndicators(void)
{
    static uint8_t s_led_run_on = 0U;
    static uint16_t s_led_run_blink_counter = 0U;
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (g_StateSnapshot.runtime.system_fault != 0U)
    {
        /* 故障状态：运行灯灭，故障灯亮 */
        led_run_on = 0U;
        led_fault_on = 1U;
        s_led_run_on = 0U;
        s_led_run_blink_counter = 0U;
    }
    else if (g_StateSnapshot.runtime.system_running != 0U)
    {
        /* 运行状态：运行灯闪烁 */
        if (s_led_run_blink_counter >= (FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS - 1U))
        {
            s_led_run_blink_counter = 0U;
            s_led_run_on = (s_led_run_on == 0U) ? 1U : 0U;
        }
        else
        {
            s_led_run_blink_counter++;
        }
        led_run_on = s_led_run_on;
    }
    else
    {
        /* 空闲状态：所有灯灭 */
        led_run_on = 0U;
        s_led_run_on = 0U;
        s_led_run_blink_counter = 0U;
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, led_fault_on);

    /* 通讯脉冲：收到有效帧后短暂点亮通讯灯 */
    if (g_led_comm_pulse_counter > 0U)
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
        g_led_comm_pulse_counter--;
    }
    else
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 0U);
    }
}

/* 停止快速电流环，复位所有相关状态 */
static void FOC_App_StopFastCurrentLoop(void)
{
    g_fast_current_loop_enabled = 0U;
    g_fast_current_loop_iq_target = 0.0f;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_div_counter = 0U;
}

/* 初始化电机硬件参数 */
static void FOC_App_InitMotorHardware(void)
{
    MotorControlService_InitMotor(&g_motor,
                                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);

    /* PID控制器初始化（基于默认配置） */
    MotorControlService_InitPidControllers(&g_motor,
                                           &g_torque_current_pid,
                                           &g_speed_pid,
                                           &g_angle_pid,
                                           &g_StateSnapshot.control_cfg);
}

/* 重新初始化电机（恢复校准），需屏蔽中断路径防止并发访问 */
static void FOC_App_ReInitMotor(void)
{
    FOC_Platform_WriteDebugText("\r\n=== ReInitMotor started ===\r\n");

    /* 关闭快速电流环ISR路径 */
    FOC_App_StopFastCurrentLoop();
    MotorControlService_ResetCurrentSoftSwitchState(&g_motor);
    MotorControlService_RunOpenLoopControlTask(&g_motor, 0.0f, 0.0f);
    MotorControlService_ForceStopPwm();

    /* 门控ISR控制路径，防止重标定期间访问g_motor */
    g_reinit_in_progress = 1U;

    FOC_App_InitMotorHardware();

    /* 解除ISR控制路径封锁 */
    g_reinit_in_progress = 0U;

    char info[120];
    snprintf(info,
             sizeof(info),
             "reinit done: mech_zero=%.4f rad, dir=%d, poles=%d, vbus=%.2fV\r\n",
             (double)g_motor.mech_angle_at_elec_zero_rad,
             (int)g_motor.direction,
             (int)g_motor.pole_pairs,
             (double)g_sensor_snapshot.vbus_voltage_filtered);
    FOC_Platform_WriteDebugText(info);

    Runtime_ClearReinit();
}

/* 进入安全输出状态：停止PWM输出，电机处于高阻态 */
static void FOC_App_EnterSafeOutputState(uint8_t report_skip)
{
    FOC_App_StopFastCurrentLoop();
    MotorControlService_ResetCurrentSoftSwitchState(&g_motor);

    /* 开环输出0 -> 电机不施加电压 */
    MotorControlService_RunOpenLoopControlTask(&g_motor, 0.0f, 0.0f);
    MotorControlService_ForceStopPwm();

    if (report_skip != 0U)
    {
        runtime_step_signal_t step_input = {0};
        step_input.control_loop_skipped = 1U;
        Runtime_UpdateSignals(&step_input);
    }
}

/* 运行控制算法入口（外环+补偿） */
static void FOC_App_RunControlAlgorithm(const sensor_data_t *sensor_data)
{
    static uint8_t s_last_control_mode = 0xFFU;
    uint8_t cur_mode;
    float dt_sec;

    cur_mode = g_StateSnapshot.control_cfg.control_mode;

    /* 控制模式切换时复位电流软开关状态和PID积分 */
    if (cur_mode != s_last_control_mode)
    {
        MotorControlService_ResetCurrentSoftSwitchState(&g_motor);
        s_last_control_mode = cur_mode;
    }

    dt_sec = FOC_CONTROL_DT_SEC;
    /* 运行外环（速度/角度/PID计算），输出iq_target给快速电流环 */
    MotorControlService_RunOuterLoopControlTask(&g_motor,
                                                &g_torque_current_pid,
                                                &g_speed_pid,
                                                &g_angle_pid,
                                                sensor_data,
                                                cur_mode,
                                                g_StateSnapshot.control_cfg.speed_only_rad_s,
                                                g_StateSnapshot.control_cfg.target_angle_rad,
                                                g_StateSnapshot.control_cfg.angle_position_speed_rad_s,
                                                dt_sec);

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    (void)MotorControlService_CoggingCalibSampleStep(&g_motor, sensor_data, dt_sec);
#endif

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    MotorControlService_RunCompensationStep(&g_motor, sensor_data);
#endif

    /* 更新快速电流环的参考值 */
    g_fast_current_loop_iq_target = g_motor.iq_target;
    g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
    g_fast_current_loop_enabled = 1U;
}


void FOC_App_Init(void)
{
    runtime_step_signal_t init_step = {0};

    /* 1. 平台硬件初始化 */
    FOC_Platform_RuntimeInit();

    /* 2. 指示灯初始化（点亮所有灯用于自检） */
    FOC_Platform_IndicatorInit();
    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, 1U);

    /* 3. 控制定时器初始化 */
    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init();

    FOC_Platform_SetControlTickCallback(ControlScheduler_RunTick);

    /* 4. 注册调度回调 */
    ControlScheduler_SetCallback(FOC_TASK_RATE_SERVICE, Service_Task_Trigger);
    ControlScheduler_SetCallback(FOC_TASK_RATE_FAST_CONTROL, Motor_Control_Loop);
    ControlScheduler_SetCallback(FOC_TASK_RATE_MONITOR, Monitor_Task_Trigger);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

    /* 5. 通讯初始化 */
    FOC_Platform_CommInit();

    /* 6. 运行时初始化 */
    Runtime_Init();
    Runtime_GetSnapshot(&g_StateSnapshot);
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask |
                                      RUNTIME_INIT_CHECK_COMMAND |
                                      RUNTIME_INIT_CHECK_COMM);

    /* 7. 电机控制配置默认值 */
    MotorControlService_ResetControlConfigDefault(&g_motor);
    FOC_Platform_WriteDebugText("\r\n=== FOC System Started ===\r\n");
    FOC_Platform_WriteDebugText("Init motor,please wait...\r\n\r\n");

    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_PROTOCOL);

    /* 8. 调试流初始化 */
    DebugStream_Init();

    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_DEBUG);

    /* 9. 传感器初始化 */
    MotorControlService_InitSensorInput(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot);

    if ((g_sensor_snapshot.adc_valid != 0U) && (g_sensor_snapshot.encoder_valid != 0U))
    {
        init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_SENSOR);
    }
    else
    {
        init_step.init_checks_fail_mask = (uint16_t)(init_step.init_checks_fail_mask | RUNTIME_INIT_CHECK_SENSOR);
    }

    /* 10. 欠压检测 */
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)

    if(g_sensor_snapshot.vbus_voltage_filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_VBUS);
    }
    else
    {
        init_step.init_checks_fail_mask = (uint16_t)(init_step.init_checks_fail_mask | RUNTIME_INIT_CHECK_VBUS);
    }

#else
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_VBUS);
#endif

    /* 11. PWM输出初始化 */
    MotorControlService_InitPwmOutput(FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);
    FOC_Platform_SetPwmUpdateCallback(FOC_App_OnPwmUpdateISR);
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_PWM);

    /* 12. 电机初始化 + 标定 */
    FOC_App_InitMotorHardware();
    init_step.init_checks_pass_mask = (uint16_t)(init_step.init_checks_pass_mask | RUNTIME_INIT_CHECK_MOTOR);

    char startup_info[160];
    snprintf(startup_info,
            sizeof(startup_info),
            "mech zero at elec0: %.4f rad, direction: %d, pole pairs: %d, vbus: %.2fV, set_voltage: %.2fV, duty_max: %.2f\r\n true_vbus: %.2fV\r\n",
            (double)g_motor.mech_angle_at_elec_zero_rad,
            (int)g_motor.direction,
            (int)g_motor.pole_pairs,
            (double)g_motor.vbus_voltage,
            (double)g_motor.set_voltage,
            (double)(g_motor.vbus_voltage > 0.0f ? g_motor.set_voltage / g_motor.vbus_voltage : 0.0f),
            (double)g_sensor_snapshot.vbus_voltage_filtered);
    FOC_Platform_WriteDebugText(startup_info);
    init_step.finalize_init = 1U;

    Runtime_UpdateSignals(&init_step);

    FOC_App_UpdateIndicators();
}

/* 启动控制定时器，使能运行时中断 */
void FOC_App_Start(void)
{
    FOC_Platform_StartControlTickSource();
    FOC_Platform_SetControlRuntimeInterrupts(1U);
}

/* 主循环：处理服务任务和监控任务 */
void FOC_App_Loop(void)
{
    if (g_monitor_task_pending != 0U)
    {
        g_monitor_task_pending = 0U;
    #if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
        DebugStream_SetExecutionCycles(ControlScheduler_GetExecutionCycles());
        DebugStream_Process(&g_sensor_snapshot,
                            &g_motor,
                            &g_StateSnapshot.runtime,
                            &g_StateSnapshot.telemetry);
    #endif
    }

    if (g_service_task_pending != 0U)
    {
        g_service_task_pending = 0U;

        /* 处理重初始化请求 */
        if (g_StateSnapshot.runtime.reinit_pending != 0U)
        {
            FOC_App_ReInitMotor();
            Runtime_GetSnapshot(&g_StateSnapshot);
        }

        /* 处理通讯帧 */
        if (Runtime_FrameRunStep(FOC_APP_COMM_FRAMES_PER_STEP) != 0U)
            g_led_comm_pulse_counter = FOC_LED_COMM_PULSE_TICKS;

        /* 参数变更时同步到电机实例 */
        if (g_StateSnapshot.runtime.params_dirty != 0U)
        {
            MotorControlService_ApplyConfigSnapshot(&g_motor,
                                                        &g_torque_current_pid,
                                                        &g_speed_pid,
                                                        &g_angle_pid,
                                                        &g_StateSnapshot.control_cfg);
            MotorControlService_SetSensorSampleOffsetPercent(g_StateSnapshot.control_cfg.sensor_sample_offset_percent);
            Runtime_Commit();
        }

        /* 齿槽补偿标定数据转储/导出 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    if (MotorControlService_CoggingCalibIsDumpPending() != 0U)
    {
        MotorControlService_CoggingCalibClearDumpPending();
        MotorControlService_CoggingCalibDumpTable(&g_motor);
    }
    else if (MotorControlService_CoggingCalibIsExportPending() != 0U)
    {
        MotorControlService_CoggingCalibClearExportPending();
        MotorControlService_CoggingCalibExportTable(&g_motor);
    }
#endif
    }
}

/* PWM更新ISR回调：快速电流环（在PWM更新中断中执行） */
static void FOC_App_OnPwmUpdateISR(void)
{
    uint8_t divider;
    float current_loop_dt_sec;
    const sensor_data_t *current_sensor = 0;

    /* 重初始化进行中时跳过 */
    if (g_reinit_in_progress != 0U)
    {
        return;
    }

    /* 快速电流环未启用时跳过 */
    if (g_fast_current_loop_enabled == 0U)
    {
        return;
    }

    /* PWM插值（更新SVPWM比较值） */
    MotorControlService_RunPwmInterpolationIsr();

    /* 电流环分频：不是每次PWM中断都运行电流环 */
    divider = (FOC_CURRENT_LOOP_ISR_DIVIDER == 0U) ? 1U : (uint8_t)FOC_CURRENT_LOOP_ISR_DIVIDER;
    g_fast_current_loop_div_counter++;
    if (g_fast_current_loop_div_counter < divider)
    {
        return;
    }
    g_fast_current_loop_div_counter = 0U;

    g_motor.iq_target = g_fast_current_loop_iq_target;
    if (FOC_PWM_FREQ_KHZ == 0U)
    {
        current_loop_dt_sec = FOC_CONTROL_DT_SEC;
    }
    else
    {
        current_loop_dt_sec = (float)divider / ((float)FOC_PWM_FREQ_KHZ * 1000.0f);
    }

    /* 读取电流采样（如果需要） */
    if (MotorControlService_RequiresCurrentSample() != 0U)
    {
        MotorControlService_ReadCurrentSensorSnapshot(&g_fast_current_sensor_snapshot);

        if (g_fast_current_sensor_snapshot.adc_valid == 0U)
        {
            return;
        }
        current_sensor = &g_fast_current_sensor_snapshot;
    }

    /* 执行电流环控制（Id/Iq PI + 逆Park + SVPWM） */
    MotorControlService_RunCurrentLoopControlTask(&g_motor,
                                                  &g_torque_current_pid,
                                                  current_sensor,
                                                  g_fast_current_loop_electrical_angle,
                                                  current_loop_dt_sec);
}

/* 服务任务触发器（中速调度） */
static void Service_Task_Trigger(void)
{
    FOC_App_UpdateIndicators();
	Runtime_GetSnapshot(&g_StateSnapshot);

    g_service_task_pending = 1U;
}

/* 监控任务触发器（低速调度） */
static void Monitor_Task_Trigger(void)
{
    g_monitor_task_pending = 1U;
}

/* 电机控制主回路（快速调度） */
static void Motor_Control_Loop(void)
{
    if (g_reinit_in_progress != 0U)
    {
        return;
    }

    if (g_StateSnapshot.runtime.system_fault != 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    /* 读取全部传感器快照 */
    MotorControlService_ReadAllSensorSnapshot(&g_sensor_snapshot);

    runtime_step_signal_t step_input = {0};
    step_input.sensor_state_updated = 1U;
    step_input.adc_valid = g_sensor_snapshot.adc_valid;
    step_input.encoder_valid = g_sensor_snapshot.encoder_valid;
    step_input.undervoltage_vbus = g_sensor_snapshot.vbus_voltage_filtered;

    Runtime_UpdateSignals(&step_input);

    /* 电机未使能时进入安全输出 */
    if (g_StateSnapshot.control_cfg.motor_enabled == 0U)
    {
        FOC_App_EnterSafeOutputState(1U);
        return;
    }

    /* 齿槽标定模式下的特殊处理 */
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    if (MotorControlService_CoggingCalibIsBusy(&g_motor) != 0U)
    {
        /* 标定模式下：切换到开环，绕过正常外环 */
        g_motor.current_soft_switch_status.configured_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        g_motor.current_soft_switch_status.enabled = 0U;

        MotorControlService_CoggingCalibSampleStep(&g_motor, &g_sensor_snapshot, FOC_CONTROL_DT_SEC);

        g_fast_current_loop_iq_target = g_motor.iq_target;
        g_fast_current_loop_electrical_angle = g_motor.electrical_phase_angle;
        g_fast_current_loop_enabled = 1U;
    }
    else
#endif
    {
        FOC_App_RunControlAlgorithm(&g_sensor_snapshot);
    }
}