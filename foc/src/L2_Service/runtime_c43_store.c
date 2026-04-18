#include "L2_Service/runtime_c43_store.h"

#include <stdio.h>
#include <string.h>

#include "LS_Config/foc_config.h"

#define RUNTIME_C43_SYSTEM_INIT 0U
#define RUNTIME_C43_SYSTEM_RUNNING 1U
#define RUNTIME_C43_SYSTEM_FAULT 2U

#define RUNTIME_C43_COMM_IDLE 0U
#define RUNTIME_C43_COMM_ACTIVE 1U
#define RUNTIME_C43_COMM_ERROR 2U

#define RUNTIME_C43_REPORT_OFF 0U
#define RUNTIME_C43_REPORT_SEMANTIC_ONLY 1U
#define RUNTIME_C43_REPORT_OSC_ONLY 2U
#define RUNTIME_C43_REPORT_BOTH 3U

#define RUNTIME_C43_DIAG_NOT_EXECUTED 0U

#define RUNTIME_C43_FAULT_NONE 0U

static runtime_c43_runtime_state_t g_runtime;
static runtime_c43_params_t g_params;
static runtime_c43_states_t g_states;

static uint8_t RuntimeC43_IsInRange(float value, float min_value, float max_value)
{
    if ((value < min_value) || (value > max_value))
    {
        return 0U;
    }

    return 1U;
}

void RuntimeC43_ResetStorageDefaults(void)
{
    (void)memset(&g_runtime, 0, sizeof(g_runtime));
    (void)memset(&g_params, 0, sizeof(g_params));
    (void)memset(&g_states, 0, sizeof(g_states));

    g_runtime.system_state = RUNTIME_C43_SYSTEM_INIT;
    g_runtime.comm_state = RUNTIME_C43_COMM_IDLE;
    g_runtime.report_mode = RUNTIME_C43_REPORT_SEMANTIC_ONLY;
    g_runtime.init_diag = RUNTIME_C43_DIAG_NOT_EXECUTED;
    g_runtime.last_fault_code = RUNTIME_C43_FAULT_NONE;

    g_params.target_angle_rad = COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD;
    g_params.angle_speed_rad_s = COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S;
    g_params.speed_only_rad_s = COMMAND_MANAGER_DEFAULT_SPEED_ONLY_RAD_S;
    g_params.sensor_sample_offset_percent = FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT;
    g_params.semantic_freq_hz = COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ;
    g_params.osc_freq_hz = COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ;
    g_params.osc_param_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
    g_params.pid_current_kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    g_params.pid_current_ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    g_params.pid_current_kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    g_params.pid_angle_kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    g_params.pid_angle_ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    g_params.pid_angle_kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    g_params.pid_speed_kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    g_params.pid_speed_ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    g_params.pid_speed_kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
    g_params.cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    g_params.cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    g_params.cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    g_params.cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    g_params.cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
    g_params.control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    g_params.current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_params.current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    g_params.current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;

    g_states.motor_enable = COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;
    g_states.semantic_enable = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
    g_states.osc_enable = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
    g_states.current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
}

runtime_c43_runtime_state_t *RuntimeC43_Runtime(void)
{
    return &g_runtime;
}

runtime_c43_params_t *RuntimeC43_Params(void)
{
    return &g_params;
}

runtime_c43_states_t *RuntimeC43_States(void)
{
    return &g_states;
}

uint8_t RuntimeC43_WriteParam(char subcommand, float value)
{
    runtime_c43_runtime_state_t *runtime = RuntimeC43_Runtime();
    runtime_c43_params_t *params = RuntimeC43_Params();

    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD,
                                 COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U)
        {
            return 0U;
        }
        params->target_angle_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S,
                                 COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->angle_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_SPEED_ONLY_MIN_RAD_S,
                                 COMMAND_MANAGER_PARAM_SPEED_ONLY_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->speed_only_rad_s = value;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        if (RuntimeC43_IsInRange(value,
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
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        if (RuntimeC43_IsInRange(value,
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
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        if (RuntimeC43_IsInRange(value,
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
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        if (RuntimeC43_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        if (RuntimeC43_IsInRange(value,
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

    runtime->params_dirty = 1U;
    return 1U;
}

uint8_t RuntimeC43_WriteState(char subcommand, uint8_t state)
{
    runtime_c43_runtime_state_t *runtime = RuntimeC43_Runtime();
    runtime_c43_states_t *states = RuntimeC43_States();
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
        runtime->params_dirty = 1U;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

uint8_t RuntimeC43_ReadParam(char subcommand, float *value_out)
{
    const runtime_c43_params_t *params = RuntimeC43_Params();

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

uint8_t RuntimeC43_ReadState(char subcommand, uint8_t *state_out)
{
    const runtime_c43_states_t *states = RuntimeC43_States();

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

void RuntimeC43_ReportAllParams(void)
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
        if (RuntimeC43_ReadParam(params[i], &value) != 0U)
        {
            RuntimeC44_OutputParam(params[i], value);
        }
    }
}

void RuntimeC43_OutputParam(char subcommand, float value)
{
    RuntimeC44_OutputParam(subcommand, value);
}

void RuntimeC43_ReportAllStates(void)
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
        if (RuntimeC43_ReadState(states[i], &state) != 0U)
        {
            RuntimeC44_OutputState(states[i], state);
        }
    }
}

void RuntimeC43_OutputState(char subcommand, uint8_t value)
{
    RuntimeC44_OutputState(subcommand, value);
}

void RuntimeC43_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));

    snapshot->runtime.system_running = (g_runtime.system_state == RUNTIME_C43_SYSTEM_RUNNING) ? 1U : 0U;
    snapshot->runtime.system_fault = (g_runtime.system_state == RUNTIME_C43_SYSTEM_FAULT) ? 1U : 0U;
    snapshot->runtime.params_dirty = g_runtime.params_dirty;
    snapshot->runtime.last_exec_ok = g_runtime.last_exec_ok;

    snapshot->control_cfg.control_mode = g_params.control_mode;
    snapshot->control_cfg.target_angle_rad = g_params.target_angle_rad;
    snapshot->control_cfg.angle_position_speed_rad_s = g_params.angle_speed_rad_s;
    snapshot->control_cfg.speed_only_rad_s = g_params.speed_only_rad_s;
    snapshot->control_cfg.sensor_sample_offset_percent = g_params.sensor_sample_offset_percent;
    snapshot->control_cfg.pid_current_kp = g_params.pid_current_kp;
    snapshot->control_cfg.pid_current_ki = g_params.pid_current_ki;
    snapshot->control_cfg.pid_current_kd = g_params.pid_current_kd;
    snapshot->control_cfg.pid_angle_kp = g_params.pid_angle_kp;
    snapshot->control_cfg.pid_angle_ki = g_params.pid_angle_ki;
    snapshot->control_cfg.pid_angle_kd = g_params.pid_angle_kd;
    snapshot->control_cfg.pid_speed_kp = g_params.pid_speed_kp;
    snapshot->control_cfg.pid_speed_ki = g_params.pid_speed_ki;
    snapshot->control_cfg.pid_speed_kd = g_params.pid_speed_kd;
    snapshot->control_cfg.cfg_min_mech_angle_accum_delta_rad = g_params.cfg_min_mech_angle_accum_delta_rad;
    snapshot->control_cfg.cfg_angle_hold_integral_limit = g_params.cfg_angle_hold_integral_limit;
    snapshot->control_cfg.cfg_angle_hold_pid_deadband_rad = g_params.cfg_angle_hold_pid_deadband_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_start_rad = g_params.cfg_speed_angle_transition_start_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_end_rad = g_params.cfg_speed_angle_transition_end_rad;
    snapshot->control_cfg.motor_enabled = g_states.motor_enable;
    snapshot->control_cfg.current_soft_switch_enable = g_states.current_soft_switch_enable;
    snapshot->control_cfg.current_soft_switch_mode = g_params.current_soft_switch_mode;
    snapshot->control_cfg.current_soft_switch_auto_open_iq_a = g_params.current_soft_switch_auto_open_iq_a;
    snapshot->control_cfg.current_soft_switch_auto_closed_iq_a = g_params.current_soft_switch_auto_closed_iq_a;

    snapshot->telemetry.semantic_report_enabled = g_states.semantic_enable;
    snapshot->telemetry.osc_report_enabled = g_states.osc_enable;
    snapshot->telemetry.semantic_report_freq_hz = g_params.semantic_freq_hz;
    snapshot->telemetry.osc_report_freq_hz = g_params.osc_freq_hz;
    snapshot->telemetry.osc_parameter_mask = g_params.osc_param_mask;
}

void RuntimeC43_ClearDirty(void)
{
    g_runtime.params_dirty = 0U;
}

void RuntimeC43_OutputDiag(const char *level, const char *module, const char *detail)
{
    RuntimeC44_OutputDiag(level, module, detail);
}

void RuntimeC43_OutputRuntimeSummary(void)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "STATE SYS=%u COMM=%u REPORT=%u DIRTY=%u LAST=%u INIT=%u FAULT=%s SENS_INV=%u PROTO_ERR=%lu PARAM_ERR=%lu CTRL_SKIP=%lu\r\n",
             (unsigned int)g_runtime.system_state,
             (unsigned int)g_runtime.comm_state,
             (unsigned int)g_runtime.report_mode,
             (unsigned int)g_runtime.params_dirty,
             (unsigned int)g_runtime.last_exec_ok,
             (unsigned int)g_runtime.init_diag,
             RuntimeC44_GetFaultName(g_runtime.last_fault_code),
             (unsigned int)g_runtime.sensor_invalid_consecutive,
             (unsigned long)g_runtime.protocol_error_count,
             (unsigned long)g_runtime.param_error_count,
             (unsigned long)g_runtime.control_skip_count);
    RuntimeC44_WriteText(out);
#endif
}

void RuntimeC43_OutputFaultControlSummary(void)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "FAULT_CTRL state=%u fault=%s proto_err=%lu param_err=%lu ctrl_skip=%lu\r\n",
             (unsigned int)g_runtime.system_state,
             RuntimeC44_GetFaultName(g_runtime.last_fault_code),
             (unsigned long)g_runtime.protocol_error_count,
             (unsigned long)g_runtime.param_error_count,
             (unsigned long)g_runtime.control_skip_count);
    RuntimeC44_WriteText(out);
#endif
}

const char *RuntimeC43_GetFaultName(uint8_t fault_code)
{
    return RuntimeC44_GetFaultName(fault_code);
}

void RuntimeC43_WriteText(const char *text)
{
    RuntimeC44_WriteText(text);
}

void RuntimeC43_WriteStatusByte(uint8_t status)
{
    RuntimeC44_WriteStatusByte(status);
}
