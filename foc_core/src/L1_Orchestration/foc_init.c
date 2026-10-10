#include <stdio.h>

#include "L2_Core/foc_motor_aggregate.h"
#include "L1_Orchestration/foc_init.h"

#include <stdio.h>
#include <string.h>

#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Runtime/foc_queue.h"
#include "L2_Core/Runtime/foc_task_scheduler.h"
#include "L2_Core/Runtime/foc_debug_stream.h"
#include "L2_Core/Runtime/foc_runtime_types.h"
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void FOC_Init_Runtime(foc_system_t *sys, foc_motor_t *motor,
                      FOC_Platform_IsrCallback_t tick_cb,
                      FOC_Platform_IsrCallback_t service_cb,
                      FOC_Platform_IsrCallback_t control_cb,
                      FOC_Platform_IsrCallback_t monitor_cb,
                      FOC_Platform_IsrCallback_t pwm_cb,
                      FOC_Platform_IsrCallback_t current_loop_cb)
{
    if ((sys == 0) || (motor == 0)) return;

    sys->runtime.tasks.service_pending = 0U;
    sys->runtime.tasks.monitor_pending = 0U;
    sys->runtime.indicator.comm_pulse_counter = 0U;
    sys->runtime.indicator.run_blink_counter = 0U;
    sys->runtime.comm.source_rr = 0U;
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    sys->runtime.monitor.frame_active = 0U;
    sys->runtime.monitor.osc_collect_count = 0U;
#endif

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init(&sys->runtime.scheduler);
    FOC_Platform_SetControlTickCallback(tick_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_SERVICE, service_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_FAST_CONTROL, control_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_MONITOR, monitor_cb);
    FOC_Platform_SetControlInterruptsEnabled(0U);

    FOC_Platform_CommInit();
    FOC_OutputMgr_Init(sys);

#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    FIFO_Init(&sys->runtime.monitor.elem_fifo,
              (uint8_t *)sys->runtime.monitor.elem_buffer,
              sizeof(monitor_element_t),
              FOC_MONITOR_ELEM_QUEUE_DEPTH);
#endif

    FOC_Protocol_Init(&sys->cfg.report);
#if ((DEBUG_STREAM_ENABLE_SEMANTIC_REPORT == FOC_CFG_ENABLE) || \
     (DEBUG_STREAM_ENABLE_OSC_REPORT == FOC_CFG_ENABLE))
    DebugStream_Init(&sys->runtime.monitor.stream);
#endif
    FOC_ControlPlatform_InitHardware(motor);
    FOC_Platform_SetPwmUpdateCallback(pwm_cb);
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    FOC_Platform_AuxTimerInit(FOC_AUX_TIMER_CURRENT_LOOP,
                              FOC_CURRENT_LOOP_ISR_FREQ_HZ,
                              current_loop_cb);
#endif
}

void FOC_Init_Motor(foc_motor_t *motor)
{
    if (motor == 0) return;

    FOC_MotorInit(motor,
                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_MOTOR_INIT_MAX_PHASE_VOLTAGE_DEFAULT,
                  FOC_MOTOR_INIT_RESISTANCE_OHM,
                  FOC_MOTOR_INIT_STATOR_INDUCTANCE_HENRY,
                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);
    FOC_Control_ApplyConfig(&motor->ctrl,
                            &motor->torque_current_pid,
                            &motor->speed_pid,
                            &motor->angle_pid,
                            &motor->cfg,
                            &motor->params);

    /* 初始化所有编译启用的 Source 私有状态 */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimSMO_Init(&motor->estim_smo_state, &motor->params);
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimHFI_Init(&motor->estim_hfi_state);
#endif
}

/* 单一母线电压安全判定（上电门与运行期 trip 共用） */
uint8_t FOC_Init_IsVbusSafe(const sensor_data_t *sensor)
{
#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if ((sensor->vbus_valid != 0U) &&
        (sensor->vbus.filtered >= FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT))
    {
        return 1U;
    }
    return 0U;
#else
    (void)sensor;
    return 1U;
#endif
}

/* 就绪前母线电压门：多次采样 + 阈值判定（唯一电压安全判定入口） */
uint8_t FOC_Init_VbusGate(sensor_data_t *sensor)
{
    uint8_t i;

    if (sensor == 0) return 0U;

    for (i = 0U; i < FOC_VBUS_GATE_SAMPLE_COUNT; i++)
    {
        Sensor_ReadVBUS(sensor);
        FOC_Platform_WaitMs(FOC_VBUS_GATE_SAMPLE_INTERVAL_MS);
    }

    return FOC_Init_IsVbusSafe(sensor);
}

/* 校验位可读名（失败报告用） */
static const struct
{
    uint16_t bit;
    const char *name;
} k_init_check_names[] =
{
    { RUNTIME_INIT_CHECK_SENSOR,   "SENSOR"   },
    { RUNTIME_INIT_CHECK_MOTOR,    "MOTOR"    },
    { RUNTIME_INIT_CHECK_VBUS,     "VBUS"     },
    { RUNTIME_INIT_CHECK_PWM,      "PWM"      },
    { RUNTIME_INIT_CHECK_DEBUG,    "DEBUG"    },
    { RUNTIME_INIT_CHECK_COMMAND,  "COMMAND"  },
    { RUNTIME_INIT_CHECK_PROTOCOL, "PROTOCOL" },
    { RUNTIME_INIT_CHECK_COMM,     "COMM"     }
};

static void FOC_Init_ReportChecksFailed(uint16_t bad)
{
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];
    uint8_t first = 1U;
    uint16_t i;
    int n;

    n = snprintf(out, sizeof(out), "init: checks failed [");
    for (i = 0U; i < (uint16_t)(sizeof(k_init_check_names) / sizeof(k_init_check_names[0])); i++)
    {
        if ((bad & k_init_check_names[i].bit) == 0U) continue;

        n += snprintf(out + n, (size_t)sizeof(out) - (size_t)n, "%s%s",
                      (first != 0U) ? "" : ",",
                      k_init_check_names[i].name);
        first = 0U;
    }
    snprintf(out + n, (size_t)sizeof(out) - (size_t)n, "]\r\n");
    FOC_Platform_WriteDebugText(out);
}

void FOC_Init_Verify_Static(foc_motor_t *motor, uint8_t vbus_ok)
{
    uint16_t missing;
    uint16_t bad;

    if (motor == 0) return;

    /* 静态可判定项：不含电机参数位（由 FOC_Init_Verify_Motor 在对齐完成后判定） */
    motor->state.init_check_mask = RUNTIME_INIT_CHECK_COMMAND |
                                    RUNTIME_INIT_CHECK_COMM |
                                    RUNTIME_INIT_CHECK_PROTOCOL |
                                    RUNTIME_INIT_CHECK_DEBUG |
                                    RUNTIME_INIT_CHECK_PWM;

#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((motor->sensor.adc_valid != 0U) && (motor->sensor.encoder_valid != 0U))
#else
    if (motor->sensor.adc_valid != 0U)
#endif
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }

    if (vbus_ok != 0U)
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_VBUS;
    }

    missing = (uint16_t)(RUNTIME_INIT_CHECK_VBUS |
               RUNTIME_INIT_CHECK_PWM |
               RUNTIME_INIT_CHECK_SENSOR |
               RUNTIME_INIT_CHECK_DEBUG |
               RUNTIME_INIT_CHECK_COMMAND |
               RUNTIME_INIT_CHECK_PROTOCOL |
               RUNTIME_INIT_CHECK_COMM) & (~motor->state.init_check_mask);

    if ((motor->state.init_fail_mask == 0U) && (missing == 0U))
    {
        motor->state.system_running = 1U;
        motor->state.system_fault = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        FOC_Platform_WriteDebugText("init: static checks passed\r\n");
        return;
    }

    bad = (uint16_t)(motor->state.init_fail_mask | missing);
    motor->state.system_running = 0U;
    motor->state.system_fault = 1U;
    motor->state.last_fault_code = ((bad & (uint16_t)(~RUNTIME_INIT_CHECK_VBUS)) == 0U) ?
        (uint8_t)FOC_FAULT_UNDERVOLTAGE : (uint8_t)FOC_FAULT_INIT_FAILED;
    FOC_Init_ReportChecksFailed(bad);
}

/* 就绪后：电机参数就绪判定（STARTUP 对齐完成时调用；ISR 安全，仅状态字段 + fast 短码） */
uint8_t FOC_Init_Verify_Motor(foc_motor_t *motor)
{
    if (motor == 0) return 0U;

    if (FOC_Control_IsMotorParamCalibrated(&motor->params) != 0U)
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_MOTOR;
        motor->state.init_fail_mask &= (uint16_t)(~RUNTIME_INIT_CHECK_MOTOR);
        motor->state.system_running = 1U;
        motor->state.system_fault = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        FOC_Platform_WriteDebugFast("init: OK\r\n");
        return 1U;
    }

    motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_MOTOR;
    motor->state.system_running = 0U;
    motor->state.system_fault = 1U;
    motor->state.last_fault_code = (uint8_t)FOC_FAULT_INIT_FAILED;
    FOC_Platform_WriteDebugFast("FAULT PARAM\r\n");
    return 0U;
}
