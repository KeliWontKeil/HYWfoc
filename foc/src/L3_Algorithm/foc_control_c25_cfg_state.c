#include "L3_Algorithm/foc_control_c25_cfg_state.h"

#include "LS_Config/foc_config.h"

#if (FOC_COGGING_LUT_POINT_COUNT > FOC_MOTOR_COGGING_LUT_CAPACITY)
#error "FOC_MOTOR_COGGING_LUT_CAPACITY must be >= FOC_COGGING_LUT_POINT_COUNT"
#endif

static void FOC_ResetSoftSwitchBlendInit(foc_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE))
    motor->current_soft_switch_blend_initialized = 0U;
#else
    (void)motor;
#endif
}

void FOC_ControlConfigResetDefault(foc_motor_t *motor)
{
    uint16_t i;

    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    motor->control_runtime_cfg.angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    motor->control_runtime_cfg.angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    motor->control_runtime_cfg.speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    motor->control_runtime_cfg.speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;

    motor->current_soft_switch_status.enabled = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
    motor->current_soft_switch_status.configured_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    motor->current_soft_switch_status.active_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    motor->current_soft_switch_status.blend_factor =
        (motor->current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) ? 0.0f : 1.0f;
    motor->current_soft_switch_status.auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    motor->current_soft_switch_status.auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    FOC_ResetSoftSwitchBlendInit(motor);

    motor->cogging_comp_status.enabled = FOC_COGGING_COMP_ENABLE;
    motor->cogging_comp_status.available = 0U;
    motor->cogging_comp_status.source = FOC_COGGING_COMP_SOURCE_NONE;
    motor->cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
    motor->cogging_comp_status.iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
    motor->cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    motor->cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;

    for (i = 0U; i < (uint16_t)FOC_MOTOR_COGGING_LUT_CAPACITY; i++)
    {
        motor->cogging_comp_table_q15[i] = 0;
    }
}

const foc_control_runtime_config_t *FOC_ControlGetRuntimeConfig(const foc_motor_t *motor)
{
    if (motor == 0)
    {
        return 0;
    }

    return &motor->control_runtime_cfg;
}

void FOC_ControlSetMinMechAngleAccumDeltaRad(foc_motor_t *motor, float value)
{
    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.min_mech_angle_accum_delta_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetAngleHoldIntegralLimit(foc_motor_t *motor, float value)
{
    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.angle_hold_integral_limit = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetAngleHoldPidDeadbandRad(foc_motor_t *motor, float value)
{
    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.angle_hold_pid_deadband_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetSpeedAngleTransitionStartRad(foc_motor_t *motor, float value)
{
    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.speed_angle_transition_start_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetSpeedAngleTransitionEndRad(foc_motor_t *motor, float value)
{
    if (motor == 0)
    {
        return;
    }

    motor->control_runtime_cfg.speed_angle_transition_end_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetCurrentSoftSwitchEnable(foc_motor_t *motor, uint8_t enable)
{
    if (motor == 0)
    {
        return;
    }

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    motor->current_soft_switch_status.enabled = (enable != 0U) ? FOC_CFG_ENABLE : FOC_CFG_DISABLE;
#else
    (void)enable;
    motor->current_soft_switch_status.enabled = FOC_CFG_DISABLE;
#endif
    FOC_ResetSoftSwitchBlendInit(motor);
}

void FOC_ControlSetCurrentSoftSwitchMode(foc_motor_t *motor, uint8_t mode)
{
    if (motor == 0)
    {
        return;
    }

    if ((mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) &&
        (mode != FOC_CURRENT_SOFT_SWITCH_MODE_AUTO))
    {
        mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
    }

    motor->current_soft_switch_status.configured_mode = mode;
    if (mode != FOC_CURRENT_SOFT_SWITCH_MODE_AUTO)
    {
        motor->current_soft_switch_status.active_mode = mode;
    }
    FOC_ResetSoftSwitchBlendInit(motor);
}

void FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(foc_motor_t *motor, float value)
{
    float sanitized = (value < 0.0f) ? 0.0f : value;

    if (motor == 0)
    {
        return;
    }

    motor->current_soft_switch_status.auto_open_iq_a = sanitized;
    if (motor->current_soft_switch_status.auto_closed_iq_a < motor->current_soft_switch_status.auto_open_iq_a)
    {
        motor->current_soft_switch_status.auto_closed_iq_a = motor->current_soft_switch_status.auto_open_iq_a;
    }
}

void FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(foc_motor_t *motor, float value)
{
    float sanitized = (value < 0.0f) ? 0.0f : value;

    if (motor == 0)
    {
        return;
    }

    if (sanitized < motor->current_soft_switch_status.auto_open_iq_a)
    {
        sanitized = motor->current_soft_switch_status.auto_open_iq_a;
    }

    motor->current_soft_switch_status.auto_closed_iq_a = sanitized;
}

const foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatus(const foc_motor_t *motor)
{
    if (motor == 0)
    {
        return 0;
    }

    return &motor->current_soft_switch_status;
}

void FOC_ControlResetCurrentSoftSwitchState(foc_motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    FOC_ResetSoftSwitchBlendInit(motor);

    if (motor->current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN)
    {
        motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        motor->current_soft_switch_status.blend_factor = 0.0f;
    }
    else if (motor->current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        motor->current_soft_switch_status.blend_factor = 1.0f;
    }
    else
    {
        if ((motor->current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
            (motor->current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
        {
            motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        }

        motor->current_soft_switch_status.blend_factor =
            (motor->current_soft_switch_status.active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) ? 1.0f : 0.0f;
    }
}

void FOC_ControlSetCoggingCompEnable(foc_motor_t *motor, uint8_t enable)
{
    if (motor == 0)
    {
        return;
    }

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    motor->cogging_comp_status.enabled = (enable != 0U) ? FOC_CFG_ENABLE : FOC_CFG_DISABLE;
#else
    (void)enable;
    motor->cogging_comp_status.enabled = FOC_CFG_DISABLE;
#endif
}

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatusMutable(foc_motor_t *motor)
{
    if (motor == 0)
    {
        return 0;
    }

    return &motor->current_soft_switch_status;
}

uint8_t *FOC_ControlGetCurrentSoftSwitchBlendInitFlag(foc_motor_t *motor)
{
    if (motor == 0)
    {
        return 0;
    }

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE))
    return &motor->current_soft_switch_blend_initialized;
#else
    return 0;
#endif
}

