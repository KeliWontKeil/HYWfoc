#include "L2_Core/Control/foc_ctrl_cfg.h"

#include <math.h>

#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

static void FOC_ResetSoftSwitchBlendInit(foc_current_soft_switch_status_t *soft_switch)
{
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    soft_switch->blend_initialized = 0U;
#else
    (void)soft_switch;
#endif
}

void FOC_ControlConfigResetDefault(foc_control_cfg_t *cfg,
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
                                   foc_current_soft_switch_status_t *soft_switch,
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
                                   foc_cogging_comp_status_t *comp_status,
                                   int16_t *comp_table_q15,
#endif
                                   uint16_t comp_table_point_count)
{
    cfg->min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    cfg->angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    cfg->angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    cfg->speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    cfg->speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    soft_switch->enabled = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
    soft_switch->configured_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    soft_switch->active_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    soft_switch->blend_factor =
        (soft_switch->configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) ? 0.0f : 1.0f;
    soft_switch->auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    soft_switch->auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    FOC_ResetSoftSwitchBlendInit(soft_switch);
#endif

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    comp_status->enabled = FOC_COGGING_COMP_ENABLE;
    comp_status->available = 0U;
    comp_status->source = FOC_COGGING_COMP_SOURCE_NONE;
    comp_status->point_count = FOC_COGGING_LUT_POINT_COUNT;
    comp_status->iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
    comp_status->speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    comp_status->iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;
    comp_status->calib_gain_k = FOC_COGGING_CALIB_GAIN_K;

    for (uint16_t i = 0U; i < comp_table_point_count; i++)
    {
        comp_table_q15[i] = 0;
    }
#endif
    (void)comp_table_point_count;
}

void FOC_PIDInit(foc_pid_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    if (pid == 0) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

/* FOC_Control_ApplyConfig — 副作用执行器（无 cfg 同步） */
void FOC_Control_ApplyConfig(foc_control_runtime_t *ctrl,
                             foc_pid_t *torque_pid,
                             foc_pid_t *speed_pid,
                             foc_pid_t *angle_pid,
                             foc_control_cfg_t *cfg,
                             const foc_motor_params_t *params)
{
    float phase_res;
    float i_max;

    phase_res = (fabsf(params->phase_resistance) > 1e-6f) ? fabsf(params->phase_resistance) : 1e-6f;
    i_max = ctrl->max_phase_voltage / phase_res;
    if (i_max < 0.0f) i_max = 0.0f;

    torque_pid->out_min = -ctrl->max_phase_voltage;
    torque_pid->out_max = ctrl->max_phase_voltage;
    angle_pid->out_min = -i_max;
    angle_pid->out_max = i_max;
    speed_pid->out_min = -i_max;
    speed_pid->out_max = i_max;
    Sensor_ADCSampleTimeOffset(cfg->sensor_sample_offset_percent);
}
