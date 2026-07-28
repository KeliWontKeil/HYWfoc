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
                      FOC_Platform_TickCallback_t tick_cb,
                      FOC_Platform_TickCallback_t service_cb,
                      FOC_Platform_TickCallback_t control_cb,
                      FOC_Platform_TickCallback_t monitor_cb,
                      FOC_Platform_PwmIsrCallback_t pwm_cb)
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
    sys->runtime.monitor.osc_collect_offset = 0U;
    sys->runtime.monitor.osc_collect_buf[0] = '\0';
#endif

    FOC_Platform_ControlTickSourceInit();
    ControlScheduler_Init(&sys->runtime.scheduler);
    FOC_Platform_SetControlTickCallback(tick_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_SERVICE, service_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_FAST_CONTROL, control_cb);
    ControlScheduler_SetCallback(&sys->runtime.scheduler, FOC_TASK_RATE_MONITOR, monitor_cb);
    FOC_Platform_SetControlRuntimeInterrupts(0U);

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
}

void FOC_Init_MotorAndCalib(foc_motor_t *motor)
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
    FOC_Control_ApplyConfig(motor);

    /* 初始化所有编译启用的 Source 私有状态 */
#if (FOC_ESTIMATOR_SMO_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimSMO_Init(motor);
#endif
#if (FOC_ESTIMATOR_HFI_ENABLE == FOC_CFG_ENABLE)
    FOC_EstimHFI_Init(motor);
#endif
}

void FOC_Init_Verify(foc_motor_t *motor, const sensor_data_t *sensor)
{
    uint16_t missing;

    if ((motor == 0) || (sensor == 0)) return;

    motor->state.init_check_mask = RUNTIME_INIT_CHECK_COMMAND |
                                    RUNTIME_INIT_CHECK_COMM |
                                    RUNTIME_INIT_CHECK_PROTOCOL |
                                    RUNTIME_INIT_CHECK_DEBUG |
                                    RUNTIME_INIT_CHECK_PWM;

#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    if ((sensor->adc_valid != 0U) && (sensor->encoder_valid != 0U))
#else
    if (sensor->adc_valid != 0U)
#endif
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }

    if ((motor->params.direction != FOC_DIR_UNDEFINED) &&
        (motor->params.mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
      if (sensor->vbus.filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
#else
    motor->state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
#endif

    missing = (uint16_t)(RUNTIME_INIT_CHECK_VBUS |
               RUNTIME_INIT_CHECK_MOTOR |
               RUNTIME_INIT_CHECK_PWM |
               RUNTIME_INIT_CHECK_SENSOR |
               RUNTIME_INIT_CHECK_DEBUG |
               RUNTIME_INIT_CHECK_COMMAND |
               RUNTIME_INIT_CHECK_PROTOCOL |
               RUNTIME_INIT_CHECK_COMM) & (~motor->state.init_check_mask);

    if ((motor->state.init_check_mask != 0U) &&
        (motor->state.init_fail_mask == 0U) &&
        (missing == 0U))
    {
        motor->state.system_running = 1U;
        motor->state.system_fault = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        FOC_Platform_WriteDebugText("init: all checks passed\r\n");
    }
    else
    {
        motor->state.system_running = 0U;
        motor->state.system_fault = 1U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_INIT_FAILED;
        FOC_Platform_WriteDebugText("init: checks failed or missing\r\n");
    }
}
