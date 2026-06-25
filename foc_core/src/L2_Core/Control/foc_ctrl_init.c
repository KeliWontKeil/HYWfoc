#include "L2_Core/Control/foc_ctrl_init.h"

#include <stdio.h>
#include <math.h>

#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
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

    if (motor == 0)
    {
        return;
    }

    need_zero = (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) ? 1U : 0U;
    need_direction = (motor->direction == FOC_DIR_UNDEFINED) ? 1U : 0U;
    need_pole_pairs = (motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ? 1U : 0U;

    if ((need_zero == 0U) && (need_direction == 0U) && (need_pole_pairs == 0U))
    {
        return;
    }

    backup_ud = motor->ud;
    backup_uq = motor->uq;

    calib_uq = motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO;
    calib_uq = Math_ClampFloat(calib_uq, 0.0f, motor->set_voltage);

    motor->uq = 0.0f;
    motor->ud = calib_uq;

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_ZERO_LOCK_SETTLE_MS,
                                            FOC_CALIB_ZERO_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->mech_angle_at_elec_zero_rad = mech_zero_rad_est;
            motor->mech_angle_accum_rad = mech_zero_rad_est;
            motor->mech_angle_prev_rad = mech_zero_rad_est;
            motor->mech_angle_prev_valid = 1U;
        }
        else
        {
            motor->mech_angle_at_elec_zero_rad = FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED;
            motor->mech_angle_accum_rad = 0.0f;
            motor->mech_angle_prev_rad = 0.0f;
            motor->mech_angle_prev_valid = 0U;
            FOC_OutputMgr_WriteDirect("init.calib: zero-lock sampling failed, keep zero as undefined\r\n");
        }
    }
    else
    {
        FOC_CurrentControlApplyElectricalAngleDirect(motor, 0.0f);
        FOC_Platform_WaitMs(FOC_CALIB_ZERO_LOCK_SETTLE_MS);
    }

    if ((need_direction != 0U) || (need_pole_pairs != 0U))
    {
        if (FOC_EstimateDirectionAndPolePairs(motor, &direction_est, &pole_pairs_est) != 0U)
        {
            if (need_direction != 0U)
            {
                motor->direction = direction_est;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = pole_pairs_est;
            }
        }
        else
        {
            if (need_direction != 0U)
            {
                motor->direction = FOC_DIR_UNDEFINED;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = FOC_POLE_PAIRS_UNDEFINED;
            }
            FOC_OutputMgr_WriteDirect("init.calib: direction/pole-pairs estimation failed, keep as undefined\r\n");
        }
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_CurrentControlApplyElectricalAngleDirect(motor, 0.0f);
}

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction)
{
    if (motor == 0)
    {
        return;
    }

    if (vbus_voltage < 0.0f)
    {
        vbus_voltage = 0.0f;
    }
    set_voltage = Math_ClampFloat(set_voltage, 0.0f, vbus_voltage);

    motor->electrical_phase_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 0.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->iq_target = 0.0f;

    motor->iq_measured = 0.0f;
    motor->cogging_speed_ref_rad_s = 0.0f;
    motor->mech_angle_accum_rad = 0.0f;
    motor->mech_angle_prev_rad = 0.0f;
    motor->mech_angle_prev_valid = 0U;
    motor->phase_resistance = phase_resistance;
    motor->pole_pairs = pole_pairs;
    motor->mech_angle_at_elec_zero_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_accum_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_valid = 1U;
    motor->direction = direction;

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
    motor->state.cfg_dirty = 0U;
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
    motor->target_angle_rad = COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD;
    motor->angle_position_speed_rad_s = COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S;
    motor->speed_only_rad_s = COMMAND_MANAGER_DEFAULT_SPEED_ONLY_RAD_S;
    motor->sensor_sample_offset_percent = FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT;
    motor->torque_current_pid.kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    motor->torque_current_pid.ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    motor->torque_current_pid.kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    motor->angle_pid.kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    motor->angle_pid.ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    motor->angle_pid.kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    motor->speed_pid.kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    motor->speed_pid.ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    motor->speed_pid.kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
    motor->min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    motor->angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    motor->angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    motor->speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    motor->speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
    motor->current_soft_switch_status.enabled = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
    motor->current_soft_switch_status.configured_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    motor->current_soft_switch_status.auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    motor->current_soft_switch_status.auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    motor->cogging_comp_status.enabled = (uint8_t)FOC_COGGING_COMP_ENABLE;
    motor->cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;
    motor->cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    motor->cogging_comp_status.calib_gain_k = FOC_COGGING_CALIB_GAIN_K;
#endif

    FOC_ControlConfigResetDefault(motor);
    /* per-motor sub-struct init */
    motor->mode_transition.prev_control_mode = 0U;
    motor->mode_transition.prev_control_mode_valid = 0U;
    motor->mode_transition.prev_control_mode_check = 0xFFU;
    motor->outer_loop_state.speed_err_accum_rad = 0.0f;
    motor->outer_loop_state.prev_mech_signed_rad = 0.0f;
    motor->outer_loop_state.speed_state_valid = 0U;
    motor->svpwm_lpf.valid = 0U;
    motor->svpwm_lpf.phase_a = 0.0f;
    motor->svpwm_lpf.phase_b = 0.0f;
    motor->svpwm_lpf.phase_c = 0.0f;

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
        (void)FOC_ControlLoadCoggingCompTableQ15(motor,
                                                  foc_cogging_default_table_q15,
                                                  FOC_COGGING_LUT_POINT_COUNT,
                                                  FOC_COGGING_LUT_IQ_LSB_A,
                                                  FOC_COGGING_COMP_SOURCE_STATIC);
#endif

        if (table_defined != 0U)
        {
            FOC_OutputMgr_WriteDirect("init.cogging: static table defined, compensation ready\r\n");
        }
        else
        {
            FOC_OutputMgr_WriteDirect("init.cogging: no table defined, use Y:G to calibrate or set static table\r\n");
        }
    }
#endif

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    motor->ecycle_offset_dyn_a = 0.0f;
    motor->ecycle_offset_dyn_b = 0.0f;
    motor->ecycle_prev_mech_angle = 0.0f;
    motor->ecycle_accu_mech_delta = 0.0f;
    motor->ecycle_sample_count = 0U;
    motor->ecycle_accum_a = 0.0f;
    motor->ecycle_accum_b = 0.0f;
    motor->ecycle_offset_valid = 0U;
#endif
}

/* L2 硬件初始化收口：封装 Sensor/SVPWM/ControlExecutor 初始化序列 */
void FOC_ControlPlatform_InitHardware(foc_motor_t *motor)
{
    if (motor == 0) return;

    Sensor_InitSnapshot(&motor->sensor);
    Sensor_InitSnapshot(&motor->sensor_fast);
    Sensor_Init(FOC_SENSOR_SAMPLE_FREQ_KHZ, FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
    Sensor_SetZeroOffset(motor);
    /* 初始采样：编码器 + VBUS（电流在 PWM ISR 中由 Sensor_ReadCurrent 接管） */
    Sensor_ReadEncoder(motor, &motor->sensor);
    Sensor_ReadVBUS(&motor->sensor);
    motor->sensor.adc_valid = 1U;

    SVPWM_Init(motor, FOC_PWM_FREQ_KHZ, FOC_SVPWM_DEADTIME_PERCENT_DEFAULT);

    FOC_ControlExecutor_Init(motor);
}