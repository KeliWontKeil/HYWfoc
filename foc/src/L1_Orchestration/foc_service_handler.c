#include "L1_Orchestration/foc_service_handler.h"

#include <stdio.h>

#include "L2/Control/foc_ctrl_executor.h"
#include "L2/Control/foc_ctrl_cfg.h"
#include "L2/Control/foc_ctrl_init.h"
#include "L2/Control/foc_ctrl_compensation.h"
#include "L2/Control/foc_ctrl_current_loop.h"
#include "L3/foc_svpwm.h"
#include "L3/foc_sensor.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"
#include "L2/Protocol/foc_protocol_handler.h"

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

    /* 应用配置：PID 初始化在 FOC_MotorInit -> FOC_Control_Init / FOC_ControlConfigResetDefault 中完成 */
    FOC_Control_ApplyConfig(motor);
}

/* ================================================================
 * 电机重初始化
 * ================================================================ */

void FOC_Service_ReInitMotor(foc_motor_t *motor)
{
    FOC_Platform_WriteDebugText("\r\n=== ReInitMotor started ===\r\n");

    /* 停止快速电流环 + 开环归零 */
    FOC_ControlExecutor_Stop(motor);
    FOC_CurrentControlOpenLoopStep(motor, 0.0f, 0.0f,
                                   FOC_CONTROL_DT_SEC);
    SVPWM_ApplyDirectDuty(0U, 0.0f, 0.0f, 0.0f);

    /* 设置重初始化标志（ISR 检测到后会提前退出） */
    motor->state.reinit_pending = 1U;

    /* 重新初始化硬件+控制 */
    FOC_Service_InitMotor(motor);

    /* 清除重初始化标志 */
    motor->state.reinit_pending = 0U;

    /* 日志输出 */
    {
        char info[120];
        snprintf(info, sizeof(info),
                 "reinit done: mech_zero=%.4f rad, dir=%d, poles=%d, vbus=%.2fV\r\n",
                 (double)motor->mech_angle_at_elec_zero_rad,
                 (int)motor->direction,
                 (int)motor->pole_pairs,
                 (double)motor->vbus_voltage);
        FOC_Platform_WriteDebugText(info);
    }
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
 * Service 任务：协议处理 + 系统命令
 * ================================================================ */

uint8_t FOC_Service_Process(foc_motor_t *motor)
{
    uint8_t comm_active = 0U;

    if (motor == 0) return 0U;

    /* 重初始化处理 */
    if (motor->state.reinit_pending != 0U)
    {
        FOC_Service_ReInitMotor(motor);
        FOC_Control_ApplyConfig(motor);
    }

    /* 协议处理：读取帧→解析→修改 motor */
    comm_active = FOC_Protocol_Process(motor, 1U);

    /* 配置脏检查：协议修改参数后应用 */
    if (motor->state.cfg_dirty != 0U)
    {
        FOC_Control_ApplyConfig(motor);
        Sensor_ADCSampleTimeOffset(motor->cfg.sensor_sample_offset_percent);
        FOC_Protocol_Commit(motor);
    }

    /* 系统命令处理（Y通道） */
    if (motor->state.pending_system_action != FOC_SYSACTION_NONE)
    {
        switch (motor->state.pending_system_action)
        {
        case FOC_SYSACTION_COGGING_START:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
            FOC_CoggingCalibRequestStart(motor);
#endif
            break;
        case FOC_SYSACTION_COGGING_DUMP:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
            motor->cogging_calib_state.request_dump = 1U;
#endif
            break;
        case FOC_SYSACTION_COGGING_EXPORT:
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
            motor->cogging_calib_state.request_export = 1U;
#endif
            break;
        default:
            break;
        }
        motor->state.pending_system_action = FOC_SYSACTION_NONE;
    }

    /* 齿槽标定数据导出 */
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    if (motor->cogging_calib_state.request_dump != 0U)
    {
        motor->cogging_calib_state.request_dump = 0U;
        FOC_CoggingCalibDumpTable(motor);
    }
    if (motor->cogging_calib_state.request_export != 0U)
    {
        motor->cogging_calib_state.request_export = 0U;
        FOC_CoggingCalibExportTable(motor);
    }
#endif

    return comm_active;
}
