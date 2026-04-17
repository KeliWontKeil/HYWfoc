#include "L3_Algorithm/foc_control_c21_cfg_state.h"

#include "LS_Config/foc_config.h"

static foc_control_runtime_config_t g_foc_runtime_cfg = {
    FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD,
    FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT,
    FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD,
    FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD,
    FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD
};

static foc_current_soft_switch_status_t g_current_soft_switch_status = {
    COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE,
    (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE,
    (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE,
    0.0f,
    COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A,
    COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A
};

static foc_cogging_comp_status_t g_cogging_comp_status = {
    FOC_COGGING_COMP_ENABLE,
    0U,
    FOC_COGGING_COMP_SOURCE_NONE,
    FOC_COGGING_LUT_POINT_COUNT,
    FOC_COGGING_LUT_IQ_LSB_A,
    FOC_COGGING_COMP_SPEED_GATE_RAD_S,
    FOC_COGGING_COMP_IQ_LIMIT_A
};

static int16_t g_cogging_comp_table_q15[FOC_COGGING_LUT_POINT_COUNT] = {0};

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE))
static uint8_t g_current_soft_switch_blend_initialized = 0U;
#define FOC_SOFT_SWITCH_BLEND_RESET() do { g_current_soft_switch_blend_initialized = 0U; } while (0)
#else
#define FOC_SOFT_SWITCH_BLEND_RESET() ((void)0)
#endif

void FOC_ControlConfigResetDefault(void)
{
    uint16_t i;

    g_foc_runtime_cfg.min_mech_angle_accum_delta_rad = FOC_DEFAULT_MIN_MECH_ANGLE_ACCUM_DELTA_RAD;
    g_foc_runtime_cfg.angle_hold_integral_limit = FOC_DEFAULT_ANGLE_HOLD_INTEGRAL_LIMIT;
    g_foc_runtime_cfg.angle_hold_pid_deadband_rad = FOC_DEFAULT_ANGLE_HOLD_PID_DEADBAND_RAD;
    g_foc_runtime_cfg.speed_angle_transition_start_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_START_RAD;
    g_foc_runtime_cfg.speed_angle_transition_end_rad = FOC_DEFAULT_SPEED_ANGLE_TRANSITION_END_RAD;

    g_current_soft_switch_status.enabled = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_ENABLE;
    g_current_soft_switch_status.configured_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_current_soft_switch_status.active_mode = (uint8_t)COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_MODE;
    g_current_soft_switch_status.blend_factor =
        (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) ? 0.0f : 1.0f;
    g_current_soft_switch_status.auto_open_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_OPEN_IQ_A;
    g_current_soft_switch_status.auto_closed_iq_a = COMMAND_MANAGER_DEFAULT_CURRENT_SOFT_SWITCH_AUTO_CLOSED_IQ_A;
    FOC_SOFT_SWITCH_BLEND_RESET();

    g_cogging_comp_status.enabled = FOC_COGGING_COMP_ENABLE;
    g_cogging_comp_status.available = 0U;
    g_cogging_comp_status.source = FOC_COGGING_COMP_SOURCE_NONE;
    g_cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
    g_cogging_comp_status.iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
    g_cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
    g_cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;

    for (i = 0U; i < (uint16_t)FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        g_cogging_comp_table_q15[i] = 0;
    }
}

const foc_control_runtime_config_t *FOC_ControlGetRuntimeConfig(void)
{
    return &g_foc_runtime_cfg;
}

void FOC_ControlSetMinMechAngleAccumDeltaRad(float value)
{
    g_foc_runtime_cfg.min_mech_angle_accum_delta_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetAngleHoldIntegralLimit(float value)
{
    g_foc_runtime_cfg.angle_hold_integral_limit = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetAngleHoldPidDeadbandRad(float value)
{
    g_foc_runtime_cfg.angle_hold_pid_deadband_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetSpeedAngleTransitionStartRad(float value)
{
    g_foc_runtime_cfg.speed_angle_transition_start_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetSpeedAngleTransitionEndRad(float value)
{
    g_foc_runtime_cfg.speed_angle_transition_end_rad = (value < 0.0f) ? 0.0f : value;
}

void FOC_ControlSetCurrentSoftSwitchEnable(uint8_t enable)
{
#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    g_current_soft_switch_status.enabled = (enable != 0U) ? FOC_CFG_ENABLE : FOC_CFG_DISABLE;
#else
    (void)enable;
    g_current_soft_switch_status.enabled = FOC_CFG_DISABLE;
#endif
    FOC_SOFT_SWITCH_BLEND_RESET();
}

void FOC_ControlSetCurrentSoftSwitchMode(uint8_t mode)
{
    if ((mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) &&
        (mode != FOC_CURRENT_SOFT_SWITCH_MODE_AUTO))
    {
        mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
    }

    g_current_soft_switch_status.configured_mode = mode;
    if (mode != FOC_CURRENT_SOFT_SWITCH_MODE_AUTO)
    {
        g_current_soft_switch_status.active_mode = mode;
    }
    FOC_SOFT_SWITCH_BLEND_RESET();
}

void FOC_ControlSetCurrentSoftSwitchAutoOpenIqA(float value)
{
    float sanitized = (value < 0.0f) ? 0.0f : value;

    g_current_soft_switch_status.auto_open_iq_a = sanitized;
    if (g_current_soft_switch_status.auto_closed_iq_a < g_current_soft_switch_status.auto_open_iq_a)
    {
        g_current_soft_switch_status.auto_closed_iq_a = g_current_soft_switch_status.auto_open_iq_a;
    }
}

void FOC_ControlSetCurrentSoftSwitchAutoClosedIqA(float value)
{
    float sanitized = (value < 0.0f) ? 0.0f : value;

    if (sanitized < g_current_soft_switch_status.auto_open_iq_a)
    {
        sanitized = g_current_soft_switch_status.auto_open_iq_a;
    }

    g_current_soft_switch_status.auto_closed_iq_a = sanitized;
}

const foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatus(void)
{
    return &g_current_soft_switch_status;
}

void FOC_ControlResetCurrentSoftSwitchState(void)
{
    FOC_SOFT_SWITCH_BLEND_RESET();

    if (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN)
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        g_current_soft_switch_status.blend_factor = 0.0f;
    }
    else if (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        g_current_soft_switch_status.blend_factor = 1.0f;
    }
    else
    {
        if ((g_current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
            (g_current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
        {
            g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        }

        g_current_soft_switch_status.blend_factor =
            (g_current_soft_switch_status.active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) ? 1.0f : 0.0f;
    }
}

void FOC_ControlSetCoggingCompEnable(uint8_t enable)
{
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    g_cogging_comp_status.enabled = (enable != 0U) ? FOC_CFG_ENABLE : FOC_CFG_DISABLE;
#else
    (void)enable;
    g_cogging_comp_status.enabled = FOC_CFG_DISABLE;
#endif
}

uint8_t FOC_ControlLoadCoggingCompTableQ15(const int16_t *table_q15,
                                           uint16_t point_count,
                                           float iq_lsb_a,
                                           uint8_t source)
{
    uint16_t i;

    if ((table_q15 == 0) || (point_count < 2U) || (point_count > (uint16_t)FOC_COGGING_LUT_POINT_COUNT))
    {
        return 0U;
    }

    if (iq_lsb_a <= 0.0f)
    {
        return 0U;
    }

    for (i = 0U; i < point_count; i++)
    {
        g_cogging_comp_table_q15[i] = table_q15[i];
    }

    for (; i < (uint16_t)FOC_COGGING_LUT_POINT_COUNT; i++)
    {
        g_cogging_comp_table_q15[i] = 0;
    }

    g_cogging_comp_status.point_count = point_count;
    g_cogging_comp_status.iq_lsb_a = iq_lsb_a;
    g_cogging_comp_status.available = 1U;
    g_cogging_comp_status.source = source;
    return 1U;
}

void FOC_ControlSetCoggingCompUnavailable(uint8_t source)
{
    g_cogging_comp_status.available = 0U;
    g_cogging_comp_status.source = source;
}

const foc_cogging_comp_status_t *FOC_ControlGetCoggingCompStatus(void)
{
    return &g_cogging_comp_status;
}

uint8_t FOC_ControlReadCoggingCompTableQ15(const int16_t **table_q15,
                                           uint16_t *point_count,
                                           float *iq_lsb_a)
{
    if ((table_q15 == 0) || (point_count == 0) || (iq_lsb_a == 0))
    {
        return 0U;
    }

    *table_q15 = g_cogging_comp_table_q15;
    *point_count = g_cogging_comp_status.point_count;
    *iq_lsb_a = g_cogging_comp_status.iq_lsb_a;
    return (g_cogging_comp_status.available != 0U) ? 1U : 0U;
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

foc_current_soft_switch_status_t *FOC_ControlGetCurrentSoftSwitchStatusMutable(void)
{
    return &g_current_soft_switch_status;
}

uint8_t *FOC_ControlGetCurrentSoftSwitchBlendInitFlag(void)
{
#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE))
    return &g_current_soft_switch_blend_initialized;
#else
    return 0;
#endif
}

void FOC_ControlGetCoggingCompContext(const foc_cogging_comp_status_t **status,
                                      const int16_t **table_q15)
{
    if (status != 0)
    {
        *status = &g_cogging_comp_status;
    }

    if (table_q15 != 0)
    {
        *table_q15 = g_cogging_comp_table_q15;
    }
}
