#include "L1_Orchestration/foc_service_handler.h"

#include <stdio.h>

#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Control/foc_ctrl_executor.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_cogging_calib.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L2_Core/Protocol/foc_protocol_handler.h"
#include "L2_Core/Runtime/foc_runtime_types.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* ================================================================
 * 电机硬件初始化
 * ================================================================ */

void FOC_Service_InitMotor(foc_motor_t *motor)
{
    FOC_MotorInit(motor,
                  FOC_MOTOR_INIT_VBUS_DEFAULT,
                  FOC_MOTOR_INIT_SET_VOLTAGE_DEFAULT,
                  FOC_MOTOR_INIT_PHASE_RES_DEFAULT,
                  FOC_MOTOR_INIT_POLE_PAIRS_DEFAULT,
                  FOC_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD,
                  FOC_MOTOR_INIT_DIRECTION_DEFAULT);

    /* 应用配置：PID 初始化在 FOC_MotorInit -> FOC_ControlConfigResetDefault 中完成 */
    FOC_Control_ApplyConfig(motor);
}

/* ================================================================
 * init_check 完整性校验
 * ================================================================ */

void FOC_Service_VerifyInitChecks(foc_motor_t *motor, const sensor_data_t *sensor)
{
    uint16_t missing;

    if ((motor == 0) || (sensor == 0)) return;

    motor->state.init_check_mask = RUNTIME_INIT_CHECK_COMMAND |
                                    RUNTIME_INIT_CHECK_COMM |
                                    RUNTIME_INIT_CHECK_PROTOCOL |
                                    RUNTIME_INIT_CHECK_DEBUG |
                                    RUNTIME_INIT_CHECK_PWM;

    if ((sensor->adc_valid != 0U) && (sensor->encoder_valid != 0U))
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }

    /* 标定完整性检查：direction 和 mech_zero 必须已定义 */
    if ((motor->direction != FOC_DIR_UNDEFINED) &&
        (motor->mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (sensor->vbus_voltage_filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
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

    /* 检查初始化是否全部通过 */
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

/* ================================================================
 * 配置脏检查提交
 * ================================================================ */

void FOC_Service_ApplyCfgDirty(foc_motor_t *motor)
{
    if (motor->state.cfg_dirty != 0U)
    {
        FOC_Control_ApplyConfig(motor);
        FOC_Protocol_Commit(motor);
    }
}

/* ================================================================
 * 控制循环结果处理
 * ================================================================ */

void FOC_Service_HandleControlResult(foc_motor_t *motor, uint8_t cycle_result)
{
    switch (cycle_result)
    {
    case FOC_CYCLE_OK:
        motor->state.system_running = 1U;
        break;

    case FOC_CYCLE_FAULT_SENSOR:
        motor->state.system_fault = 1U;
        motor->state.system_running = 0U;
        FOC_OutputMgr_WriteDirect("sensor invalid threshold reached\r\n");
        break;

    case FOC_CYCLE_FAULT_UVLO:
        motor->state.system_fault = 1U;
        motor->state.system_running = 0U;
        FOC_ControlExecutor_SafeOutput(motor);
        break;

    default:
        break;
    }
}

/* ================================================================
 * LED 指示器更新
 * ================================================================ */

void FOC_Service_UpdateIndicators(foc_motor_t *motor, foc_runtime_ctx_t *runtime)
{
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (motor->state.system_fault != 0U)
    {
        led_fault_on = 1U;
        runtime->indicator.led_run_blink_counter = 0U;
    }
    else if (motor->state.system_running != 0U)
    {
        runtime->indicator.led_run_blink_counter++;
        if (runtime->indicator.led_run_blink_counter >= (2U * FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS))
        {
            runtime->indicator.led_run_blink_counter = 0U;
        }
        led_run_on = (runtime->indicator.led_run_blink_counter < FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS) ? 1U : 0U;
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, led_fault_on);

    if (runtime->indicator.comm_pulse_counter > 0U)
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
        runtime->indicator.comm_pulse_counter--;
    }
    else
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 0U);
    }
}

/* ================================================================
 * Service 任务：处理数据输出（dump/export）和协议后续动作
 * 注意：reinit 和 cogging_calib 已由控制任务通过阶段路由处理，
 *       此处不再包含阻塞控制动作。
 * ================================================================ */

uint8_t FOC_Service_Process(foc_motor_t *motor)
{
    uint8_t activity = 0U;

    if (motor == 0) return 0U;

    /* 齿槽标定数据导出（由模块内部完成时自动触发，或由协议命令触发） */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    if (FOC_CoggingCalibIsDumpPending(motor) != 0U)
    {
        FOC_CoggingCalibClearDumpPending(motor);
        FOC_CoggingCalibDumpTable(motor);
        activity = 1U;
    }
    if (FOC_CoggingCalibIsExportPending(motor) != 0U)
    {
        FOC_CoggingCalibClearExportPending(motor);
        FOC_CoggingCalibExportTable(motor);
        activity = 1U;
    }
#endif

    return activity;
}
