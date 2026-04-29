#include "L2_Service/runtime_c4_runtime_core.h"

#include <stdio.h>
#include <string.h>

#include "L2_Service/runtime_c5_output_adapter.h"
#include "L3_Algorithm/protocol_core.h"
#include "LS_Config/foc_config.h"

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
#include "L3_Algorithm/foc_control_c24_compensation.h"
#endif

#define RUNTIME_STORE_SYSTEM_INIT 0U
#define RUNTIME_STORE_SYSTEM_RUNNING 1U
#define RUNTIME_STORE_SYSTEM_FAULT 2U

#define RUNTIME_STORE_COMM_IDLE 0U
#define RUNTIME_STORE_COMM_ACTIVE 1U
#define RUNTIME_STORE_COMM_ERROR 2U

#define RUNTIME_STORE_REPORT_OFF 0U
#define RUNTIME_STORE_REPORT_SEMANTIC_ONLY 1U
#define RUNTIME_STORE_REPORT_OSC_ONLY 2U
#define RUNTIME_STORE_REPORT_BOTH 3U

#define RUNTIME_STORE_DIAG_NOT_EXECUTED 0U

#define RUNTIME_STORE_FAULT_NONE 0U

static runtime_c4_runtime_view_t g_runtime;
static runtime_c4_params_view_t g_params;
static runtime_c4_states_view_t g_states;

static uint8_t RuntimeC4Store_IsInRange(float value, float min_value, float max_value)
{
    if ((value < min_value) || (value > max_value))
    {
        return 0U;
    }

    return 1U;
}

void RuntimeC4Store_ResetStorageDefaults(void)
{
    (void)memset(&g_runtime, 0, sizeof(g_runtime));
    (void)memset(&g_params, 0, sizeof(g_params));
    (void)memset(&g_states, 0, sizeof(g_states));

    g_runtime.system_state = RUNTIME_STORE_SYSTEM_INIT;
    g_runtime.comm_state = RUNTIME_STORE_COMM_IDLE;
    g_runtime.report_mode = RUNTIME_STORE_REPORT_SEMANTIC_ONLY;
    g_runtime.init_diag = RUNTIME_STORE_DIAG_NOT_EXECUTED;
    g_runtime.last_fault_code = RUNTIME_STORE_FAULT_NONE;

    g_params.target_angle_rad = COMMAND_MANAGER_DEFAULT_TARGET_ANGLE_RAD;
    g_params.angle_speed_rad_s = COMMAND_MANAGER_DEFAULT_ANGLE_SPEED_RAD_S;
    g_params.speed_only_rad_s = COMMAND_MANAGER_DEFAULT_SPEED_ONLY_RAD_S;
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    g_params.sensor_sample_offset_percent = FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT;
#endif
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    g_params.semantic_freq_hz = COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ;
    g_params.osc_freq_hz = COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ;
    g_params.osc_param_mask = COMMAND_MANAGER_DEFAULT_OSC_PARAM_MASK;
#endif
    g_params.pid_current_kp = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KP;
    g_params.pid_current_ki = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KI;
    g_params.pid_current_kd = COMMAND_MANAGER_DEFAULT_PID_CURRENT_KD;
    g_params.pid_angle_kp = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KP;
    g_params.pid_angle_ki = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KI;
    g_params.pid_angle_kd = COMMAND_MANAGER_DEFAULT_PID_ANGLE_KD;
    g_params.pid_speed_kp = COMMAND_MANAGER_DEFAULT_PID_SPEED_KP;
    g_params.pid_speed_ki = COMMAND_MANAGER_DEFAULT_PID_SPEED_KI;
    g_params.pid_speed_kd = COMMAND_MANAGER_DEFAULT_PID_SPEED_KD;
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    g_params.cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    g_params.cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    g_params.cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    g_params.cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    g_params.cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
#endif
    g_params.control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    g_params.current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_params.current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    g_params.current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    g_params.cogging_comp_iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;
    g_params.cogging_comp_speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    g_params.cogging_calib_gain_k = FOC_COGGING_CALIB_GAIN_K;

    g_states.motor_enable = COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    g_states.semantic_enable = COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED;
    g_states.osc_enable = COMMAND_MANAGER_DEFAULT_OSC_ENABLED;
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    g_states.current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
#endif
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    g_states.cogging_comp_enable = (uint8_t)FOC_COGGING_COMP_ENABLE;
#endif
}

runtime_c4_runtime_view_t *RuntimeC4Store_Runtime(void)
{
    return &g_runtime;
}

runtime_c4_params_view_t *RuntimeC4Store_Params(void)
{
    return &g_params;
}

runtime_c4_states_view_t *RuntimeC4Store_States(void)
{
    return &g_states;
}

uint8_t RuntimeC4Store_WriteParam(char subcommand, float value)
{
    runtime_c4_runtime_view_t *runtime = RuntimeC4Store_Runtime();
    runtime_c4_params_view_t *params = RuntimeC4Store_Params();

    switch (subcommand)
    {
    case COMMAND_MANAGER_PARAM_SUBCMD_TARGET_ANGLE:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_TARGET_ANGLE_MIN_RAD,
                                 COMMAND_MANAGER_PARAM_TARGET_ANGLE_MAX_RAD) == 0U)
        {
            return 0U;
        }
        params->target_angle_rad = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_ANGLE_SPEED:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_ANGLE_SPEED_MIN_RAD_S,
                                 COMMAND_MANAGER_PARAM_ANGLE_SPEED_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->angle_speed_rad_s = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_SPEED_ONLY_SPEED:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_SPEED_ONLY_MIN_RAD_S,
                                 COMMAND_MANAGER_PARAM_SPEED_ONLY_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->speed_only_rad_s = value;
        break;

#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_SENSOR_SAMPLE_OFFSET:
        if (RuntimeC4Store_IsInRange(value,
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
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KI:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_CURRENT_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_current_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_CURRENT_KD:
        if (RuntimeC4Store_IsInRange(value,
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
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KI:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_ANGLE_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_angle_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_ANGLE_KD:
        if (RuntimeC4Store_IsInRange(value,
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
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KP_MIN,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KP_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_kp = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KI:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KI_MIN,
                                 COMMAND_MANAGER_PARAM_PID_SPEED_KI_MAX) == 0U)
        {
            return 0U;
        }
        params->pid_speed_ki = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_PID_SPEED_KD:
        if (RuntimeC4Store_IsInRange(value,
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

#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MIN_A,
                                 COMMAND_MANAGER_PARAM_COGGING_COMP_IQ_LIMIT_MAX_A) == 0U)
        {
            return 0U;
        }
        params->cogging_comp_iq_limit_a = value;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE:
        if (RuntimeC4Store_IsInRange(value,
                                 COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MIN_RAD_S,
                                 COMMAND_MANAGER_PARAM_COGGING_COMP_SPEED_GATE_MAX_RAD_S) == 0U)
        {
            return 0U;
        }
        params->cogging_comp_speed_gate_rad_s = value;
        break;
#endif

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN:
        if (value < 0.0f)
        {
            return 0U;
        }
        params->cogging_calib_gain_k = value;
        break;
#endif

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

uint8_t RuntimeC4Store_WriteState(char subcommand, uint8_t state)
{
    runtime_c4_runtime_view_t *runtime = RuntimeC4Store_Runtime();
    runtime_c4_states_view_t *states = RuntimeC4Store_States();
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

#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
        states->cogging_comp_enable = normalized_state;
        runtime->params_dirty = 1U;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

uint8_t RuntimeC4Store_ReadParam(char subcommand, float *value_out)
{
    const runtime_c4_params_view_t *params = RuntimeC4Store_Params();

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

#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT:
        *value_out = params->cogging_comp_iq_limit_a;
        break;

    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE:
        *value_out = params->cogging_comp_speed_gate_rad_s;
        break;
#endif

#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN:
        *value_out = params->cogging_calib_gain_k;
        break;
#endif

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

uint8_t RuntimeC4Store_ReadState(char subcommand, uint8_t *state_out)
{
    const runtime_c4_states_view_t *states = RuntimeC4Store_States();

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

#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    case COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE:
        *state_out = states->cogging_comp_enable;
        break;
#endif

    default:
        return 0U;
    }

    return 1U;
}

void RuntimeC4Store_ReportAllParams(void)
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
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_IQ_LIMIT,
        COMMAND_MANAGER_PARAM_SUBCMD_COGGING_COMP_SPEED_GATE,
#endif
#if (FOC_COGGING_CALIB_ENABLE == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_COGGING_CALIB_GAIN,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_MODE,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ,
        COMMAND_MANAGER_PARAM_SUBCMD_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ
#endif
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(params) / sizeof(params[0])); i++)
    {
        if (RuntimeC4Store_ReadParam(params[i], &value) != 0U)
        {
            RuntimeC5_OutputParam(params[i], value);
        }
    }
}

void RuntimeC4Store_OutputParam(char subcommand, float value)
{
    RuntimeC5_OutputParam(subcommand, value);
}

void RuntimeC4Store_ReportAllStates(void)
{
    uint8_t state;
    const char states[] = {
        COMMAND_MANAGER_STATE_SUBCMD_MOTOR_ENABLE,
#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_SEMANTIC_ENABLE,
        COMMAND_MANAGER_STATE_SUBCMD_OSC_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_CURRENT_SOFT_SWITCH_ENABLE,
#endif
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
        COMMAND_MANAGER_STATE_SUBCMD_COGGING_COMP_ENABLE
#endif
    };
    uint16_t i;

    for (i = 0U; i < (uint16_t)(sizeof(states) / sizeof(states[0])); i++)
    {
        if (RuntimeC4Store_ReadState(states[i], &state) != 0U)
        {
            RuntimeC5_OutputState(states[i], state);
        }
    }
}

void RuntimeC4Store_OutputState(char subcommand, uint8_t value)
{
    RuntimeC5_OutputState(subcommand, value);
}

void RuntimeC4Store_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));

    snapshot->runtime.system_running = (g_runtime.system_state == RUNTIME_STORE_SYSTEM_RUNNING) ? 1U : 0U;
    snapshot->runtime.system_fault = (g_runtime.system_state == RUNTIME_STORE_SYSTEM_FAULT) ? 1U : 0U;
    snapshot->runtime.params_dirty = g_runtime.params_dirty;
    snapshot->runtime.last_exec_ok = g_runtime.last_exec_ok;

    snapshot->control_cfg.control_mode = g_params.control_mode;
    snapshot->control_cfg.target_angle_rad = g_params.target_angle_rad;
    snapshot->control_cfg.angle_position_speed_rad_s = g_params.angle_speed_rad_s;
    snapshot->control_cfg.speed_only_rad_s = g_params.speed_only_rad_s;
#if (FOC_PROTOCOL_ENABLE_SENSOR_SAMPLE_OFFSET == FOC_CFG_ENABLE)
    snapshot->control_cfg.sensor_sample_offset_percent = g_params.sensor_sample_offset_percent;
#endif
    snapshot->control_cfg.pid_current_kp = g_params.pid_current_kp;
    snapshot->control_cfg.pid_current_ki = g_params.pid_current_ki;
    snapshot->control_cfg.pid_current_kd = g_params.pid_current_kd;
    snapshot->control_cfg.pid_angle_kp = g_params.pid_angle_kp;
    snapshot->control_cfg.pid_angle_ki = g_params.pid_angle_ki;
    snapshot->control_cfg.pid_angle_kd = g_params.pid_angle_kd;
    snapshot->control_cfg.pid_speed_kp = g_params.pid_speed_kp;
    snapshot->control_cfg.pid_speed_ki = g_params.pid_speed_ki;
    snapshot->control_cfg.pid_speed_kd = g_params.pid_speed_kd;
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    snapshot->control_cfg.cfg_min_mech_angle_accum_delta_rad = g_params.cfg_min_mech_angle_accum_delta_rad;
    snapshot->control_cfg.cfg_angle_hold_integral_limit = g_params.cfg_angle_hold_integral_limit;
    snapshot->control_cfg.cfg_angle_hold_pid_deadband_rad = g_params.cfg_angle_hold_pid_deadband_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_start_rad = g_params.cfg_speed_angle_transition_start_rad;
    snapshot->control_cfg.cfg_speed_angle_transition_end_rad = g_params.cfg_speed_angle_transition_end_rad;
#endif
    snapshot->control_cfg.motor_enabled = g_states.motor_enable;
    snapshot->control_cfg.current_soft_switch_enable = g_states.current_soft_switch_enable;
    snapshot->control_cfg.current_soft_switch_mode = g_params.current_soft_switch_mode;
    snapshot->control_cfg.current_soft_switch_auto_open_iq_a = g_params.current_soft_switch_auto_open_iq_a;
    snapshot->control_cfg.current_soft_switch_auto_closed_iq_a = g_params.current_soft_switch_auto_closed_iq_a;
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    snapshot->control_cfg.cogging_comp_enable = g_states.cogging_comp_enable;
#endif
    snapshot->control_cfg.cogging_comp_iq_limit_a = g_params.cogging_comp_iq_limit_a;
    snapshot->control_cfg.cogging_comp_speed_gate_rad_s = g_params.cogging_comp_speed_gate_rad_s;
    snapshot->control_cfg.cogging_calib_gain_k = g_params.cogging_calib_gain_k;

#if (FOC_PROTOCOL_ENABLE_TELEMETRY_REPORT == FOC_CFG_ENABLE)
    snapshot->telemetry.semantic_report_enabled = g_states.semantic_enable;
    snapshot->telemetry.osc_report_enabled = g_states.osc_enable;
    snapshot->telemetry.semantic_report_freq_hz = g_params.semantic_freq_hz;
    snapshot->telemetry.osc_report_freq_hz = g_params.osc_freq_hz;
    snapshot->telemetry.osc_parameter_mask = g_params.osc_param_mask;
#endif
}

void RuntimeC4Store_ClearDirty(void)
{
    g_runtime.params_dirty = 0U;
}

void RuntimeC4Store_OutputDiag(const char *level, const char *module, const char *detail)
{
    RuntimeC5_OutputDiag(level, module, detail);
}

void RuntimeC4Store_OutputRuntimeSummary(void)
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
             RuntimeC5_GetFaultName(g_runtime.last_fault_code),
             (unsigned int)g_runtime.sensor_invalid_consecutive,
             (unsigned long)g_runtime.protocol_error_count,
             (unsigned long)g_runtime.param_error_count,
             (unsigned long)g_runtime.control_skip_count);
    RuntimeC5_WriteText(out);
#endif
}

void RuntimeC4Store_OutputFaultControlSummary(void)
{
#if (FOC_FEATURE_DIAG_OUTPUT == FOC_CFG_ENABLE)
    char out[COMMAND_MANAGER_REPLY_BUFFER_LEN];

    snprintf(out,
             sizeof(out),
             "FAULT_CTRL state=%u fault=%s proto_err=%lu param_err=%lu ctrl_skip=%lu\r\n",
             (unsigned int)g_runtime.system_state,
             RuntimeC5_GetFaultName(g_runtime.last_fault_code),
             (unsigned long)g_runtime.protocol_error_count,
             (unsigned long)g_runtime.param_error_count,
             (unsigned long)g_runtime.control_skip_count);
    RuntimeC5_WriteText(out);
#endif
}

const char *RuntimeC4Store_GetFaultName(uint8_t fault_code)
{
    return RuntimeC5_GetFaultName(fault_code);
}

void RuntimeC4Store_WriteText(const char *text)
{
    RuntimeC5_WriteText(text);
}

void RuntimeC4Store_WriteStatusByte(uint8_t status)
{
    RuntimeC5_WriteStatusByte(status);
}


void RuntimeC4_Init(void)
{
    RuntimeC4Store_ResetStorageDefaults();
    RuntimeC4Store_OutputDiag("INFO", "command_manager", "READY");
    RuntimeC4Store_OutputDiag("INFO", "protocol_exec", "NOT_EXECUTED");
    RuntimeC4Store_OutputDiag("INFO", "fallback", "KEEP_LAST_VALID");
}

void RuntimeC4_AccumulateInitChecks(uint16_t pass_mask, uint16_t fail_mask)
{
    runtime_c4_runtime_view_t *runtime = RuntimeC4Store_Runtime();

    runtime->init_check_mask = (uint16_t)(runtime->init_check_mask | pass_mask | fail_mask);
    runtime->init_fail_mask = (uint16_t)(runtime->init_fail_mask | fail_mask);
}

uint16_t RuntimeC4_GetInitCheckMask(void)
{
    return RuntimeC4Store_Runtime()->init_check_mask;
}

uint16_t RuntimeC4_GetInitFailMask(void)
{
    return RuntimeC4Store_Runtime()->init_fail_mask;
}

void RuntimeC4_ResetSensorInvalidConsecutive(void)
{
    RuntimeC4Store_Runtime()->sensor_invalid_consecutive = 0U;
}

void RuntimeC4_IncrementSensorInvalidConsecutive(void)
{
    RuntimeC4Store_Runtime()->sensor_invalid_consecutive++;
}

uint16_t RuntimeC4_GetSensorInvalidConsecutive(void)
{
    return RuntimeC4Store_Runtime()->sensor_invalid_consecutive;
}

void RuntimeC4_IncrementControlSkipCount(void)
{
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    RuntimeC4Store_Runtime()->control_skip_count++;
#endif
}

void RuntimeC4_IncrementProtocolErrorCount(void)
{
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    RuntimeC4Store_Runtime()->protocol_error_count++;
#endif
}

void RuntimeC4_IncrementParamErrorCount(void)
{
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    RuntimeC4Store_Runtime()->param_error_count++;
#endif
}

void RuntimeC4_SetSystemState(uint8_t system_state)
{
    RuntimeC4Store_Runtime()->system_state = system_state;
}

uint8_t RuntimeC4_GetSystemState(void)
{
    return RuntimeC4Store_Runtime()->system_state;
}

void RuntimeC4_SetCommState(uint8_t comm_state)
{
    RuntimeC4Store_Runtime()->comm_state = comm_state;
}

void RuntimeC4_SetInitDiag(uint8_t init_diag)
{
    RuntimeC4Store_Runtime()->init_diag = init_diag;
}

uint8_t RuntimeC4_GetInitDiag(void)
{
    return RuntimeC4Store_Runtime()->init_diag;
}

void RuntimeC4_SetLastFaultCode(uint8_t fault_code)
{
    RuntimeC4Store_Runtime()->last_fault_code = fault_code;
}

uint8_t RuntimeC4_GetLastExecOk(void)
{
    return RuntimeC4Store_Runtime()->last_exec_ok;
}

void RuntimeC4_SetLastExecOk(uint8_t last_exec_ok)
{
    RuntimeC4Store_Runtime()->last_exec_ok = last_exec_ok;
}

void RuntimeC4_UpdateReportMode(void)
{
    runtime_c4_runtime_view_t *runtime = RuntimeC4Store_Runtime();
    const runtime_c4_states_view_t *states = RuntimeC4Store_States();

    if ((states->semantic_enable != 0U) && (states->osc_enable != 0U))
    {
        runtime->report_mode = 3U;
    }
    else if (states->semantic_enable != 0U)
    {
        runtime->report_mode = 1U;
    }
    else if (states->osc_enable != 0U)
    {
        runtime->report_mode = 2U;
    }
    else
    {
        runtime->report_mode = 0U;
    }
}

static uint8_t RuntimeC4_ReportSingleParam(char subcommand)
{
    float value;

    if (RuntimeC4Store_ReadParam(subcommand, &value) == 0U)
    {
        return 0U;
    }

    RuntimeC4Store_OutputParam(subcommand, value);
    return 1U;
}

static uint8_t RuntimeC4_ReportSingleState(char subcommand)
{
    uint8_t state;

    if (RuntimeC4Store_ReadState(subcommand, &state) == 0U)
    {
        return 0U;
    }

    RuntimeC4Store_OutputState(subcommand, state);
    return 1U;
}

runtime_c4_exec_result_t RuntimeC4_ExecuteCommand(const protocol_command_t *cmd)
{
    float value = 0.0f;

    if ((cmd == 0) || (cmd->frame_valid == 0U))
    {
        RuntimeC4_WriteStatusFrameError();
        return RUNTIME_C4_EXEC_COMMAND_ERROR;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_PARAM)
    {
        if (cmd->has_param != 0U)
        {
            if (RuntimeC4Store_WriteParam(cmd->subcommand, cmd->param_value) == 0U)
            {
                RuntimeC4_WriteStatusParamInvalid();
                return RUNTIME_C4_EXEC_PARAM_ERROR;
            }

            if (RuntimeC4Store_ReadParam(cmd->subcommand, &value) != 0U)
            {
                RuntimeC4Store_OutputParam(cmd->subcommand, value);
            }
            return RUNTIME_C4_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_PARAM_SUBCMD_READ_ALL)
        {
            RuntimeC4Store_ReportAllParams();
            return RUNTIME_C4_EXEC_OK;
        }

        if (RuntimeC4_ReportSingleParam(cmd->subcommand) == 0U)
        {
            RuntimeC4_WriteStatusParamInvalid();
            return RUNTIME_C4_EXEC_PARAM_ERROR;
        }

        return RUNTIME_C4_EXEC_OK;
    }

    if (cmd->command == COMMAND_MANAGER_CMD_STATE)
    {
        uint8_t state = 0U;

        if (cmd->has_param != 0U)
        {
            if (ProtocolCore_ParseStateValue(cmd->param_value, &state) == 0U)
            {
                RuntimeC4_WriteStatusParamInvalid();
                return RUNTIME_C4_EXEC_PARAM_ERROR;
            }

            if (RuntimeC4Store_WriteState(cmd->subcommand, state) == 0U)
            {
                RuntimeC4_WriteStatusParamInvalid();
                return RUNTIME_C4_EXEC_PARAM_ERROR;
            }

            if (RuntimeC4Store_ReadState(cmd->subcommand, &state) == 0U)
            {
                RuntimeC4_WriteStatusParamInvalid();
                return RUNTIME_C4_EXEC_PARAM_ERROR;
            }

            RuntimeC4Store_OutputState(cmd->subcommand, state);
            return RUNTIME_C4_EXEC_OK;
        }

        if (cmd->subcommand == COMMAND_MANAGER_STATE_SUBCMD_READ_ALL)
        {
            RuntimeC4Store_ReportAllStates();
            return RUNTIME_C4_EXEC_OK;
        }

        if (RuntimeC4_ReportSingleState(cmd->subcommand) == 0U)
        {
            RuntimeC4_WriteStatusParamInvalid();
            return RUNTIME_C4_EXEC_PARAM_ERROR;
        }

        return RUNTIME_C4_EXEC_OK;
    }

    RuntimeC4_WriteStatusCmdInvalid();
    return RUNTIME_C4_EXEC_COMMAND_ERROR;
}

void RuntimeC4_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC4Store_BuildSnapshot(snapshot);
}

void RuntimeC4_ClearDirty(void)
{
    RuntimeC4Store_ClearDirty();
}

void RuntimeC4_OutputDiag(const char *level, const char *module, const char *detail)
{
    RuntimeC4Store_OutputDiag(level, module, detail);
}

void RuntimeC4_OutputRuntimeSummary(void)
{
    RuntimeC4Store_OutputRuntimeSummary();
}

void RuntimeC4_OutputFaultControlSummary(void)
{
    RuntimeC4Store_OutputFaultControlSummary();
}

const char *RuntimeC4_GetFaultName(uint8_t fault_code)
{
    return RuntimeC4Store_GetFaultName(fault_code);
}

void RuntimeC4_WriteText(const char *text)
{
    RuntimeC4Store_WriteText(text);
}

void RuntimeC4_WriteStatusOK(void)
{
    RuntimeC4Store_WriteStatusByte((uint8_t)FOC_PROTOCOL_STATUS_OK_CHAR);
}

void RuntimeC4_WriteStatusFrameError(void)
{
    RuntimeC4Store_WriteStatusByte((uint8_t)FOC_PROTOCOL_STATUS_FRAME_ERROR_CHAR);
}

void RuntimeC4_WriteStatusParamInvalid(void)
{
    RuntimeC4Store_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_PARAM_INVALID_CHAR);
}

void RuntimeC4_WriteStatusCmdInvalid(void)
{
    RuntimeC4Store_WriteStatusByte((uint8_t)COMMAND_MANAGER_STATUS_CMD_INVALID_CHAR);
}

uint8_t RuntimeC4_RecoverFaultAndReinit(void)
{
    runtime_c4_runtime_view_t *runtime = RuntimeC4Store_Runtime();
    runtime_c4_params_view_t *params = RuntimeC4Store_Params();
    runtime_c4_states_view_t *states = RuntimeC4Store_States();

    /* Reset all fault-related counters and states */
    runtime->sensor_invalid_consecutive = 0U;
#if (FOC_FEATURE_DIAG_STATS == FOC_CFG_ENABLE)
    runtime->protocol_error_count = 0U;
    runtime->param_error_count = 0U;
    runtime->control_skip_count = 0U;
#endif
    runtime->last_fault_code = 0U;
    runtime->comm_state = 0U;
    
    /* Reset system state to INIT to allow re-initialization */
    runtime->system_state = RUNTIME_STORE_SYSTEM_RUNNING;
    runtime->init_diag = RUNTIME_STORE_DIAG_NOT_EXECUTED;
    
    /* Clear initialization check masks to allow fresh accumulation */
    runtime->init_check_mask = 0U;
    runtime->init_fail_mask = 0U;
    
    /* Mark parameters as dirty to ensure they are reapplied */
    runtime->params_dirty = 1U;
    runtime->last_exec_ok = 1U;

    /* Reset configurable parameters to defaults */
#if (FOC_PROTOCOL_ENABLE_CONTROL_FINE_TUNING == FOC_CFG_ENABLE)
    params->cfg_min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    params->cfg_angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    params->cfg_angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    params->cfg_speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    params->cfg_speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;
#endif
#if (FOC_PROTOCOL_ENABLE_CURRENT_SOFT_SWITCH == FOC_CFG_ENABLE)
    params->current_soft_switch_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    params->current_soft_switch_auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    params->current_soft_switch_auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    states->current_soft_switch_enable = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
#endif
#if (FOC_PROTOCOL_ENABLE_COGGING_COMP == FOC_CFG_ENABLE)
    states->cogging_comp_enable = (uint8_t)FOC_COGGING_COMP_ENABLE;
#endif

    /* Reset motor enable state to default */
    states->motor_enable = COMMAND_MANAGER_DEFAULT_MOTOR_ENABLE;

    /* Output diagnostic message for recovery */
    RuntimeC4Store_OutputDiag("INFO", "fault_recovery", "system reset completed");

    return 1U;
}



