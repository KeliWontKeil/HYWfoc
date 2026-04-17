#include "L2_Service/command_manager.h"

#include "L2_Service/command_manager_diag.h"
#include "L2_Service/command_manager_internal.h"
#include "LS_Config/foc_config.h"

uint8_t CommandManager_ReadParam(char subcommand, float *value_out)
{
    const command_manager_params_t *params = CommandManager_InternalParams();

    if (value_out == 0)
    {
        return 0U;
    }

    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        *value_out = params->target_angle_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        *value_out = params->angle_speed_rad_s;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        *value_out = params->speed_only_rad_s;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        *value_out = params->sensor_sample_offset_percent;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        *value_out = (float)params->semantic_freq_hz;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        *value_out = (float)params->osc_freq_hz;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        *value_out = (float)params->osc_param_mask;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
        *value_out = params->pid_current_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        *value_out = params->pid_current_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        *value_out = params->pid_current_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
        *value_out = params->pid_angle_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        *value_out = params->pid_angle_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        *value_out = params->pid_angle_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
        *value_out = params->pid_speed_kp;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        *value_out = params->pid_speed_ki;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        *value_out = params->pid_speed_kd;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
        *value_out = params->cfg_min_mech_angle_accum_delta_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
        *value_out = params->cfg_angle_hold_integral_limit;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
        *value_out = params->cfg_angle_hold_pid_deadband_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
        *value_out = params->cfg_speed_angle_transition_start_rad;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
        *value_out = params->cfg_speed_angle_transition_end_rad;
        break;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        *value_out = (float)params->control_mode;
        break;

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        *value_out = (float)params->current_soft_switch_mode;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        *value_out = params->current_soft_switch_auto_open_iq_a;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        *value_out = params->current_soft_switch_auto_closed_iq_a;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

void CommandManager_ReportAllParams(void)
{
    float value;
    const char params[] = {
        COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE,
        COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED,
        COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED,
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET,
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV,
        COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI,
        COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD,
#endif
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START,
        COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END,
#endif
        COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE,
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ
#endif
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(params) / sizeof(params[0])); i++)
    {
        if (CommandManager_ReadParam(params[i], &value) != 0U)
        {
            CommandManager_OutputParam(params[i], value);
        }
    }
}

uint8_t CommandManager_ReadState(char subcommand, uint8_t *state_out)
{
    const command_manager_states_t *states = CommandManager_InternalStates();

    if (state_out == 0)
    {
        return 0U;
    }

    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        *state_out = states->motor_enable;
        break;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
        *state_out = states->semantic_enable;
        break;

    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
        *state_out = states->osc_enable;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
        *state_out = states->current_soft_switch_enable;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

void CommandManager_ReportAllStates(void)
{
    uint8_t state;
    const char states[] = {
        COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE,
        COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE
#endif
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(states) / sizeof(states[0])); i++)
    {
        if (CommandManager_ReadState(states[i], &state) != 0U)
        {
            CommandManager_OutputState(states[i], state);
        }
    }
}

float CommandManager_GetTargetAngleRad(void)
{
    return CommandManager_InternalParams()->target_angle_rad;
}

float CommandManager_GetAngleSpeedRadS(void)
{
    return CommandManager_InternalParams()->angle_speed_rad_s;
}

float CommandManager_GetSpeedOnlyRadS(void)
{
    return CommandManager_InternalParams()->speed_only_rad_s;
}

float CommandManager_GetSensorSampleOffsetPercent(void)
{
    return CommandManager_InternalParams()->sensor_sample_offset_percent;
}

uint8_t CommandManager_IsSemanticReportEnabled(void)
{
    return CommandManager_InternalStates()->semantic_enable;
}

uint8_t CommandManager_IsOscilloscopeReportEnabled(void)
{
    return CommandManager_InternalStates()->osc_enable;
}

uint16_t CommandManager_GetSemanticReportFrequencyHz(void)
{
    return CommandManager_InternalParams()->semantic_freq_hz;
}

uint16_t CommandManager_GetOscilloscopeReportFrequencyHz(void)
{
    return CommandManager_InternalParams()->osc_freq_hz;
}

uint16_t CommandManager_GetOscilloscopeParameterMask(void)
{
    return CommandManager_InternalParams()->osc_param_mask;
}

void CommandManager_ClearDirtyFlag(void)
{
    CommandManager_InternalRuntimeState()->params_dirty = 0U;
}

float CommandManager_GetCurrentPidKp(void)
{
    return CommandManager_InternalParams()->pid_current_kp;
}

float CommandManager_GetCurrentPidKi(void)
{
    return CommandManager_InternalParams()->pid_current_ki;
}

float CommandManager_GetCurrentPidKd(void)
{
    return CommandManager_InternalParams()->pid_current_kd;
}

float CommandManager_GetAnglePidKp(void)
{
    return CommandManager_InternalParams()->pid_angle_kp;
}

float CommandManager_GetAnglePidKi(void)
{
    return CommandManager_InternalParams()->pid_angle_ki;
}

float CommandManager_GetAnglePidKd(void)
{
    return CommandManager_InternalParams()->pid_angle_kd;
}

float CommandManager_GetSpeedPidKp(void)
{
    return CommandManager_InternalParams()->pid_speed_kp;
}

float CommandManager_GetSpeedPidKi(void)
{
    return CommandManager_InternalParams()->pid_speed_ki;
}

float CommandManager_GetSpeedPidKd(void)
{
    return CommandManager_InternalParams()->pid_speed_kd;
}

float CommandManager_GetControlMinMechAngleAccumDeltaRad(void)
{
    return CommandManager_InternalParams()->cfg_min_mech_angle_accum_delta_rad;
}

float CommandManager_GetControlAngleHoldIntegralLimit(void)
{
    return CommandManager_InternalParams()->cfg_angle_hold_integral_limit;
}

float CommandManager_GetControlAngleHoldPidDeadbandRad(void)
{
    return CommandManager_InternalParams()->cfg_angle_hold_pid_deadband_rad;
}

float CommandManager_GetControlSpeedAngleTransitionStartRad(void)
{
    return CommandManager_InternalParams()->cfg_speed_angle_transition_start_rad;
}

float CommandManager_GetControlSpeedAngleTransitionEndRad(void)
{
    return CommandManager_InternalParams()->cfg_speed_angle_transition_end_rad;
}

uint8_t CommandManager_GetControlMode(void)
{
    return CommandManager_InternalParams()->control_mode;
}

uint8_t CommandManager_IsMotorEnabled(void)
{
    return CommandManager_InternalStates()->motor_enable;
}

uint8_t CommandManager_IsCurrentSoftSwitchEnabled(void)
{
    return CommandManager_InternalStates()->current_soft_switch_enable;
}

uint8_t CommandManager_GetCurrentSoftSwitchMode(void)
{
    return CommandManager_InternalParams()->current_soft_switch_mode;
}

float CommandManager_GetCurrentSoftSwitchAutoOpenIqA(void)
{
    return CommandManager_InternalParams()->current_soft_switch_auto_open_iq_a;
}

float CommandManager_GetCurrentSoftSwitchAutoClosedIqA(void)
{
    return CommandManager_InternalParams()->current_soft_switch_auto_closed_iq_a;
}
