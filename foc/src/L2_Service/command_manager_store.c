#include "L2_Service/command_manager.h"

#include "L2_Service/command_manager_internal.h"
#include "LS_Config/foc_config.h"

static uint8_t CommandManager_IsInRange(float value, float min_value, float max_value)
{
    if ((value < min_value) || (value > max_value))
    {
        return 0U;
    }
    return 1U;
}

uint8_t CommandManager_WriteParam(char subcommand, float value)
{
    command_manager_runtime_state_t *runtime_state = CommandManager_InternalRuntimeState();
    command_manager_params_t *params = CommandManager_InternalParams();

    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD,
                                     COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U)
        {
            return 0U;
        }
        params->target_angle_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S,
                                     COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->angle_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_SPEED_ONLY_MIN_RAD_S,
                                     COMMAND_MANAGER_PARAM_SPEED_ONLY_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->speed_only_rad_s = value;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MIN_PERCENT,
                                     COMMAND_MANAGER_PARAM_SENSOR_SAMPLE_OFFSET_MAX_PERCENT) == 0U)
        {
            return 0U;
        }
        params->sensor_sample_offset_percent = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SEMANTIC_DIV:
        if ((value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ) ||
            (value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ))
        {
            return 0U;
        }
        params->semantic_freq_hz = (uint16_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_DIV:
        if ((value < COMMAND_MANAGER_PARAM_REPORT_FREQ_MIN_HZ) ||
            (value > COMMAND_MANAGER_PARAM_REPORT_FREQ_MAX_HZ))
        {
            return 0U;
        }
        params->osc_freq_hz = (uint16_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_OSC_PARAM_MASK:
        if ((value < COMMAND_MANAGER_PARAM_OSC_MASK_MIN) ||
            (value > COMMAND_MANAGER_PARAM_OSC_MASK_MAX))
        {
            return 0U;
        }
        params->osc_param_mask = (uint16_t)value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_CURRENT_KD_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_ANGLE_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_ANGLE_KD_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_SPEED_PID_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KP:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        if (CommandManager_IsInRange(value,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MIN,
                                     COMMAND_MANAGER_PARAM_PID_SPEED_KD_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_kd = value;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_MIN_MECH_DELTA:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cfg_min_mech_angle_accum_delta_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_I_LIMIT:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cfg_angle_hold_integral_limit = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_HOLD_DEADBAND:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cfg_angle_hold_pid_deadband_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_START:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cfg_speed_angle_transition_start_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CFG_BLEND_END:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cfg_speed_angle_transition_end_rad = value;
        break;
#endif

    case COMMAND_MANAGER_PARAM_SUBCMD_CONTROL_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CONTROL_MODE_MIN) ||
            (value > COMMAND_MANAGER_PARAM_CONTROL_MODE_MAX))
        {
            return 0U;
        }
        params->control_mode = (uint8_t)value;
        break;

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MIN) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_MODE_MAX))
        {
            return 0U;
        }
        params->current_soft_switch_mode = (uint8_t)value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MIN_A) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_MAX_A) ||
            (value > params->current_soft_switch_auto_closed_iq_a))
        {
            return 0U;
        }
        params->current_soft_switch_auto_open_iq_a = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ:
        if ((value < COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MIN_A) ||
            (value > COMMAND_MANAGER_PARAM_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_MAX_A) ||
            (value < params->current_soft_switch_auto_open_iq_a))
        {
            return 0U;
        }
        params->current_soft_switch_auto_closed_iq_a = value;
        break;
#endif

    default:
        return 0U;
    }

    runtime_state->params_dirty = 1U;
    return 1U;
}

uint8_t CommandManager_WriteState(char subcommand, uint8_t state)
{
    command_manager_runtime_state_t *runtime_state = CommandManager_InternalRuntimeState();
    command_manager_states_t *states = CommandManager_InternalStates();
    uint8_t normalized_state = (state != 0U) ? COMMAND_MANAGER_ENABLED_ENABLE : COMMAND_MANAGER_ENABLED_DISABLE;

    switch (subcommand)
    {
    case COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE:
        states->motor_enable = normalized_state;
        break;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE:
        states->semantic_enable = normalized_state;
        break;

    case COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE:
        states->osc_enable = normalized_state;
        break;
#endif

#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE:
        states->current_soft_switch_enable = normalized_state;
        runtime_state->params_dirty = 1U;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}
