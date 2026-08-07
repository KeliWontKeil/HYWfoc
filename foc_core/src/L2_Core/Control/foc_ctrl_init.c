#include "L2_Core/foc_motor_aggregate.h"
#include "L2_Core/Control/foc_ctrl_init.h"

#include <stdio.h>
#include <math.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L2_Core/Control/foc_ctrl_param_learn.h"
#include "L2_Core/Control/foc_ctrl_compensation.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"
#include "LS_Config/foc_config.h"
#include "LS_Config/foc_cogging_table.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_executor.h"

void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor)
{
    float calib_uq;
    float backup_ud;
    float backup_uq;
    int8_t direction_est;
    uint8_t pole_pairs_est;
    float mech_zero_rad_est;
    uint8_t need_zero;
    uint8_t need_direction;
    uint8_t need_pole_pairs;

    need_zero = (motor->params.mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) ? 1U : 0U;
    need_direction = (motor->params.direction == FOC_DIR_UNDEFINED) ? 1U : 0U;
    need_pole_pairs = (motor->params.pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ? 1U : 0U;

    if ((need_zero == 0U) && (need_direction == 0U) && (need_pole_pairs == 0U))
    {
        return;
    }

    backup_ud = motor->ctrl.ud;
    backup_uq = motor->ctrl.uq;

    calib_uq = motor->ctrl.max_phase_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO;
    calib_uq = Math_ClampFloat(calib_uq, 0.0f, motor->ctrl.max_phase_voltage);

    motor->ctrl.uq = 0.0f;
    motor->ctrl.ud = calib_uq;

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(&motor->ctrl, &motor->svpwm,
                                            &motor->applied_output, &motor->alpha_beta,
                                            &motor->params,
                                            0.0f,
                                            FOC_CALIB_ZERO_LOCK_SETTLE_MS,
                                            FOC_CALIB_ZERO_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->params.mech_angle_at_elec_zero_rad = mech_zero_rad_est;
            motor->outer_loop.accum_rad = mech_zero_rad_est;
            motor->outer_loop.prev_rad = mech_zero_rad_est;
            motor->outer_loop.prev_valid = 1U;
        }
        else
        {
            motor->params.mech_angle_at_elec_zero_rad = FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED;
            motor->outer_loop.accum_rad = 0.0f;
            motor->outer_loop.prev_rad = 0.0f;
            motor->outer_loop.prev_valid = 0U;
            FOC_Platform_WriteDebugText("init.calib: zero-lock sampling failed, keep zero as undefined\r\n");
        }
    }
    else
    {
        FOC_ControlApplyElectricalAngleDirect(&motor->ctrl, &motor->svpwm,
                                              &motor->applied_output, &motor->alpha_beta,
                                              &motor->params,
                                              0.0f);
        FOC_Platform_WaitMs(FOC_CALIB_ZERO_LOCK_SETTLE_MS);
    }

    if ((need_direction != 0U) || (need_pole_pairs != 0U))
    {
        if (FOC_EstimateDirectionAndPolePairs(&motor->ctrl, &motor->params, &motor->svpwm,
                                              &motor->applied_output, &motor->alpha_beta,
                                              &direction_est, &pole_pairs_est) != 0U)
        {
            if (need_direction != 0U)
            {
                motor->params.direction = direction_est;
            }
            if (need_pole_pairs != 0U)
            {
                motor->params.pole_pairs = pole_pairs_est;
            }
        }
        else
        {
            if (need_direction != 0U)
            {
                motor->params.direction = FOC_DIR_UNDEFINED;
            }
            if (need_pole_pairs != 0U)
            {
                motor->params.pole_pairs = FOC_POLE_PAIRS_UNDEFINED;
            }
            FOC_Platform_WriteDebugText("init.calib: direction/pole-pairs estimation failed, keep as undefined\r\n");
        }
    }

    motor->ctrl.ud = backup_ud;
    motor->ctrl.uq = backup_uq;
    FOC_ControlApplyElectricalAngleDirect(&motor->ctrl, &motor->svpwm,
                                          &motor->applied_output, &motor->alpha_beta,
                                          &motor->params,
                                          0.0f);
}

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float max_phase_voltage,
                   float phase_resistance,
                   float stator_inductance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction)
{
    if (vbus_voltage < 0.0f)
    {
        vbus_voltage = 0.0f;
    }
    max_phase_voltage = Math_ClampFloat(max_phase_voltage, 0.0f, vbus_voltage);

    motor->ctrl.electrical_angle_rad = 0.0f;
    motor->ctrl.ud = 0.0f;
    motor->ctrl.uq = 0.0f;
    motor->ctrl.max_phase_voltage = max_phase_voltage;
    motor->active_source_state.source = FOC_SOURCE_TYPE_NONE;
    motor->active_source_state.state = FOC_SOURCE_STATE_INIT;
    motor->active_source_state.valid = 0U;
    motor->active_source_state.confidence = 0.0f;
    motor->active_source_state.elec_angle_rad = 0.0f;
    motor->active_source_state.mech_angle_rad = 0.0f;
    motor->source_mgr_state.active_source = FOC_SOURCE_TYPE_NONE;
    motor->source_mgr_state.standby_source = FOC_SOURCE_TYPE_NONE;
    motor->source_mgr_state.control_region = FOC_CONTROL_REGION_FULL;
    motor->source_mgr_state.region_state = FOC_REGION_STATE_FULL_ACTIVE;
    motor->source_mgr_state.switch_in_progress = 0U;
    motor->source_mgr_state.switch_counter = 0U;
    motor->source_mgr_state.config_valid = 0U;
    motor->source_mgr_state.degrade_hold_counter = 0U;
    motor->encoder_services.comp_available = 0U;
    motor->encoder_services.comp_active = 0U;
    motor->encoder_services.calib_available = 0U;
    motor->encoder_services.reinit_available = 0U;
    motor->applied_output.valid = 0U;
    motor->applied_output.valid = 0U;
    motor->applied_output.ud = 0.0f;
    motor->applied_output.uq = 0.0f;
    motor->applied_output.electrical_angle_rad = 0.0f;
    motor->phase_output_state.phase = FOC_CONTROL_PHASE_NORMAL;
    motor->phase_output_state.type = FOC_PHASE_OUTPUT_IDLE;
    motor->phase_output_state.valid = 0U;
    motor->phase_output_state.state_id = 0U;
    motor->phase_output_state.duty_a = 0.0f;
    motor->phase_output_state.duty_b = 0.0f;
    motor->phase_output_state.duty_c = 0.0f;
    motor->phase_output_state.sector = 0U;
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    motor->openloop_state.phase = FOC_OPENLOOP_STATE_IDLE;
    motor->openloop_state.virtual_angle_rad = 0.0f;
    motor->openloop_state.virtual_speed_rad_s = 0.0f;
    motor->openloop_state.ramp_rate_rad_s2 = 0.0f;
    motor->openloop_state.target_speed_rad_s = 0.0f;
    motor->openloop_state.mech_speed_rad_s = 0.0f;
#endif
    motor->params.vbus_voltage = vbus_voltage;
    motor->ctrl.iq_target = 0.0f;

    motor->ctrl.iq_measured = 0.0f;
    motor->outer_loop.accum_rad = 0.0f;
    motor->outer_loop.prev_rad = 0.0f;
    motor->outer_loop.prev_valid = 0U;

#if (FOC_MOTOR_MEASUREMENT_TYPE == FOC_MOTOR_MEASUREMENT_TYPE_Y_LINE)
    motor->params.phase_resistance  = phase_resistance / 2.0f;
    motor->params.stator_inductance = stator_inductance / 2.0f;
#elif (FOC_MOTOR_MEASUREMENT_TYPE == FOC_MOTOR_MEASUREMENT_TYPE_DELTA_LINE)
    motor->params.phase_resistance  = phase_resistance * 1.5f;
    motor->params.stator_inductance = stator_inductance * 1.5f;
#else
    motor->params.phase_resistance  = phase_resistance;
    motor->params.stator_inductance = stator_inductance;
#endif
    motor->params.pole_pairs = pole_pairs;
    motor->params.mech_angle_at_elec_zero_rad = mech_angle_at_elec_zero_rad;
    motor->outer_loop.accum_rad = mech_angle_at_elec_zero_rad;
    motor->outer_loop.prev_rad = mech_angle_at_elec_zero_rad;
    motor->outer_loop.prev_valid = 1U;
    motor->params.direction = direction;

    motor->alpha_beta.alpha = 0.0f;
    motor->alpha_beta.beta = 0.0f;
    motor->alpha_beta.phase_a = 0.0f;
    motor->alpha_beta.phase_b = 0.0f;
    motor->alpha_beta.phase_c = 0.0f;
    motor->svpwm.output.duty_a = 0.0f;
    motor->svpwm.output.duty_b = 0.0f;
    motor->svpwm.output.duty_c = 0.0f;
    motor->svpwm.output.sector = 0U;

    /* init per-motor runtime state */
    motor->state.system_running = 0U;
    motor->state.system_fault = 0U;
    motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
    motor->state.motor_enabled = (uint8_t)COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;
    motor->state.control_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    motor->state.control_phase = FOC_CONTROL_PHASE_NORMAL;
    motor->state.init_check_mask = 0U;
    motor->state.init_fail_mask = 0U;
    motor->state.sensor_invalid_consecutive = 0U;
    motor->state.protocol_error_count = 0U;
    motor->state.param_error_count = 0U;
    motor->state.control_skip_count = 0U;
    motor->state.current_loop_ready = 0U;

    /* init control config defaults (write top-level fields directly) */
    motor->cfg.target_angle_rad = COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD;
    motor->cfg.angle_position_speed_rad_s = COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S;
    motor->cfg.speed_only_rad_s = COMMAND_MANAGER_DEFAULT_SPEED_ONLY_RAD_S;
    motor->cfg.sensor_sample_offset_percent = FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT;
    motor->torque_current_pid.kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    motor->torque_current_pid.ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    motor->torque_current_pid.kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    motor->angle_pid.kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    motor->angle_pid.ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    motor->angle_pid.kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    motor->speed_pid.kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    motor->speed_pid.ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    motor->speed_pid.kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
    motor->cfg.min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    motor->cfg.angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    motor->cfg.angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    motor->cfg.speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    motor->cfg.speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
    motor->current_soft_switch_status.configured_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    motor->current_soft_switch_status.auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    motor->current_soft_switch_status.auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    motor->cogging_comp_status.enabled = (uint8_t)FOC_COGGING_COMP_ENABLE;
    motor->cogging_comp_status.speed_ref_rad_s = 0.0f;
    motor->cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;
    motor->cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    motor->cogging_comp_status.calib_gain_k = FOC_COGGING_CALIB_GAIN_K;
#endif

    FOC_ControlConfigResetDefault(&motor->cfg,
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
                                  &motor->current_soft_switch_status,
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
                                  &motor->cogging_comp_status,
                                  motor->cogging_comp_table_q15,
#endif
                                  FOC_COGGING_LUT_POINT_COUNT);
    /* per-motor sub-struct init */
    motor->mode_transition.prev_control_mode = 0U;
    motor->mode_transition.prev_control_mode_valid = 0U;
    motor->mode_transition.prev_control_mode_check = 0xFFU;
    motor->outer_loop.speed_err_accum_rad = 0.0f;
    motor->outer_loop.prev_mech_signed_rad = 0.0f;
    motor->outer_loop.speed_state_valid = 0U;

    FOC_ControlExecutor_Init(motor);

#if (FOC_INIT_CALIBRATION_ENABLE == FOC_CFG_ENABLE)
    FOC_CalibrateElectricalAngleAndDirection(motor);
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    {
        uint8_t table_defined = FOC_CFG_DISABLE;

#if (FOC_COGGING_STATIC_TABLE_DEFINED == FOC_CFG_ENABLE)
        table_defined = FOC_CFG_ENABLE;
#endif
        motor->cogging_comp_status.available = table_defined;
        motor->cogging_comp_status.enabled = table_defined;
        motor->cogging_comp_status.source = table_defined ? FOC_COGGING_COMP_SOURCE_STATIC : FOC_COGGING_COMP_SOURCE_NONE;
        motor->cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
        motor->cogging_comp_status.iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
        motor->cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
        motor->cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;

#if (FOC_COGGING_STATIC_TABLE_DEFINED == FOC_CFG_ENABLE)
        (void)FOC_ControlLoadCoggingCompTableQ15(&motor->cogging_comp_status,
                                                 motor->cogging_comp_table_q15,
                                                 foc_cogging_default_table_q15,
                                                 FOC_COGGING_LUT_POINT_COUNT,
                                                 FOC_COGGING_LUT_IQ_LSB_A,
                                                 FOC_COGGING_COMP_SOURCE_STATIC);
#endif

        if (table_defined != 0U)
        {
            FOC_Platform_WriteDebugText("init.cogging: static table defined, compensation ready\r\n");
        }
        else
        {
            FOC_Platform_WriteDebugText("init.cogging: no table defined, use Y:G to calibrate or set static table\r\n");
        }
    }
#endif

}

/* L2 硬件初始化收口：封装 Sensor/SVPWM/ControlExecutor 初始化序列 */
void FOC_ControlPlatform_InitHardware(foc_motor_t *motor)
{
    Sensor_InitSnapshot(&motor->sensor);
    Sensor_Init();
    Sensor_SetZeroOffset(&motor->sensor);
    /* 初始采样：编码器 + VBUS（电流在 PWM ISR 中由 Sensor_ReadCurrent 接管） */
#if (FOC_SENSOR_ENCODER_ENABLE == FOC_CFG_ENABLE)
    Sensor_ReadEncoder(&motor->sensor, FOC_CONTROL_DT_SEC);
#endif
    Sensor_ReadVBUS(&motor->sensor);
    motor->sensor.adc_valid = 1U;

    SVPWM_Init(&motor->svpwm);

    FOC_ControlExecutor_Init(motor);
}
