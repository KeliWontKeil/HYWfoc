#include "L3_Algorithm/foc_control.h"

#include "L3_Algorithm/foc_control_internal.h"
#include "L3_Algorithm/foc_control_softswitch.h"
#include "L3_Algorithm/foc_control_compensation.h"
#include "LS_Config/foc_config.h"
#include "LS_Config/foc_shared_types.h"
#include "L41_Math/math_transforms.h"
#include "L3_Algorithm/svpwm.h"

#define FOC_CONTROL_DT_DEFAULT_SEC FOC_CONTROL_DT_SEC

static void FOC_ApplySvpwmPreLpf(float *phase_a, float *phase_b, float *phase_c);

static float g_speed_err_accum_rad = 0.0f;
static float g_prev_mech_signed_rad = 0.0f;
static uint8_t g_speed_state_valid = 0U;
#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
static uint8_t g_prev_control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
static uint8_t g_prev_control_mode_valid = 0U;
#endif
#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_LOOP_IQ_LPF_ENABLE == FOC_CFG_ENABLE))
static uint8_t g_current_iq_lpf_state_valid = 0U;
static float g_current_iq_lpf_state = 0.0f;
#endif
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_svpwm_lpf_state_valid = 0U;
static float g_svpwm_lpf_phase_a = 0.0f;
static float g_svpwm_lpf_phase_b = 0.0f;
static float g_svpwm_lpf_phase_c = 0.0f;
#endif
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

static void FOC_ControlApplyElectricalAngleCore(foc_motor_t *motor,
                                                float electrical_angle,
                                                uint8_t direct_output)
{
    float dq_magnitude;
    float voltage_limit;
    float voltage_command;
    float ud_applied;
    float uq_applied;

    electrical_angle = Math_WrapRad(electrical_angle);
    motor->electrical_phase_angle = electrical_angle;

    voltage_limit = Math_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    dq_magnitude = sqrtf(motor->ud * motor->ud + motor->uq * motor->uq);
    ud_applied = motor->ud;
    uq_applied = motor->uq;

    if ((dq_magnitude > voltage_limit) && (dq_magnitude > 1e-6f))
    {
        float scale = voltage_limit / dq_magnitude;
        ud_applied *= scale;
        uq_applied *= scale;
        dq_magnitude = voltage_limit;
    }

    Math_InverseParkTransform(ud_applied,
                              uq_applied,
                              electrical_angle,
                              &motor->alpha,
                              &motor->beta);

    Math_InverseClarkeTransform(motor->alpha,
                                motor->beta,
                                &motor->phase_a,
                                &motor->phase_b,
                                &motor->phase_c);

    FOC_ApplySvpwmPreLpf(&motor->phase_a,
                         &motor->phase_b,
                         &motor->phase_c);

    voltage_command = Math_ClampFloat(dq_magnitude, 0.0f, voltage_limit);

#if (FOC_ZERO_VECTOR_CLAMP_ENABLE == FOC_CFG_ENABLE)
    if (voltage_command < FOC_ZERO_VECTOR_CLAMP_VOLTAGE_THRESHOLD_V)
    {
        motor->sector = 0U;
        motor->duty_a = 0.5f;
        motor->duty_b = 0.5f;
        motor->duty_c = 0.5f;

        if (direct_output != 0U)
        {
            SVPWM_ApplyDirectDuty(motor->sector,
                                  motor->duty_a,
                                  motor->duty_b,
                                  motor->duty_c);
        }
        else
        {
            SVPWM_SetRuntimeDutyTarget(motor->sector,
                                       motor->duty_a,
                                       motor->duty_b,
                                       motor->duty_c);
        }
        return;
    }
#endif

    if (direct_output != 0U)
    {
        SVPWM_UpdateDirect(motor->phase_a,
                           motor->phase_b,
                           motor->phase_c,
                           voltage_command,
                           motor->vbus_voltage,
                           &motor->sector,
                           &motor->duty_a,
                           &motor->duty_b,
                           &motor->duty_c);
    }
    else
    {
        SVPWM_UpdateRuntime(motor->phase_a,
                            motor->phase_b,
                            motor->phase_c,
                            voltage_command,
                            motor->vbus_voltage,
                            &motor->sector,
                            &motor->duty_a,
                            &motor->duty_b,
                            &motor->duty_c);
    }
}

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

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    g_prev_control_mode = COMMAND_MANAGER_DEFAULT_CONTROL_MODE;
    g_prev_control_mode_valid = 0U;
#endif

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

static float FOC_NormalizeDt(float dt_sec)
{
    return (dt_sec > 0.0f) ? dt_sec : FOC_CONTROL_DT_DEFAULT_SEC;
}

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static float FOC_CurrentLoopComputeKiScale(float iq_ref_abs)
{
    float start_a = FOC_CURRENT_LOOP_KI_LOW_CURRENT_START_A;
    float end_a = FOC_CURRENT_LOOP_KI_LOW_CURRENT_END_A;
    float low_scale = Math_ClampFloat(FOC_CURRENT_LOOP_KI_LOW_CURRENT_SCALE, 0.0f, 1.0f);

    if (end_a <= (start_a + 1e-6f))
    {
        return (iq_ref_abs <= start_a) ? low_scale : 1.0f;
    }

    if (iq_ref_abs <= start_a)
    {
        return low_scale;
    }

    if (iq_ref_abs >= end_a)
    {
        return 1.0f;
    }

    return low_scale + (1.0f - low_scale) * ((iq_ref_abs - start_a) / (end_a - start_a));
}
#endif

static void FOC_CurrentLoopEstimateOpenLoopResistanceModel(const foc_motor_t *motor,
                                                           float iq_ref,
                                                           float id_ref,
                                                           float *ud_out,
                                                           float *uq_out,
                                                           float *iq_estimated_out)
{
    float voltage_limit;
    float phase_resistance;
    float current_limit;
    float iq_estimated;
    float id_estimated;

    if ((motor == 0) || (ud_out == 0) || (uq_out == 0) || (iq_estimated_out == 0))
    {
        return;
    }

    voltage_limit = Math_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    phase_resistance = fabsf(motor->phase_resistance);
    if (phase_resistance < 1e-6f)
    {
        phase_resistance = 1e-6f;
    }

    current_limit = voltage_limit / phase_resistance;
    iq_estimated = Math_ClampFloat(iq_ref, -current_limit, current_limit);
    id_estimated = Math_ClampFloat(id_ref, -current_limit, current_limit);

    *uq_out = iq_estimated * phase_resistance;
    *ud_out = id_estimated * phase_resistance;
    *iq_estimated_out = iq_estimated;
}

static void FOC_CurrentLoopApplyOpenLoopResistanceModel(foc_motor_t *motor,
                                                        float iq_ref,
                                                        float id_ref)
{
    float ud = 0.0f;
    float uq = 0.0f;
    float iq_estimated = 0.0f;

    if (motor == 0)
    {
        return;
    }

    FOC_CurrentLoopEstimateOpenLoopResistanceModel(motor,
                                                   iq_ref,
                                                   id_ref,
                                                   &ud,
                                                   &uq,
                                                   &iq_estimated);

    motor->uq = uq;
    motor->ud = ud;
    motor->iq_measured = iq_estimated;
}

static float FOC_PIDRunCore(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    error = target - measurement;
    pid->integral += error * dt_sec;
    derivative = (error - pid->prev_error) / dt_sec;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
    }

    pid->prev_error = error;
    return output;
}

float FOC_ControlMechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad)
{
    float elec_period_rad;
    float mech_delta_mod;

    if (motor == 0)
    {
        return 0.0f;
    }

    if ((motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
        (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        return motor->electrical_phase_angle;
    }

    elec_period_rad = FOC_MATH_TWO_PI / (float)motor->pole_pairs;
    mech_delta_mod = fmodf(Math_WrapRadDelta(mech_angle_rad - motor->mech_angle_at_elec_zero_rad), elec_period_rad);
    if (mech_delta_mod < 0.0f)
    {
        mech_delta_mod += elec_period_rad;
    }

    return Math_WrapRad(motor->direction * mech_delta_mod * (float)motor->pole_pairs);
}

static void FOC_UpdateAccumulatedMechanicalAngle(foc_motor_t *motor, float mech_angle_rad)
{
    float delta;

    if (motor == 0)
    {
        return;
    }

    if (motor->mech_angle_prev_valid == 0U)
    {
        motor->mech_angle_prev_rad = mech_angle_rad;
        motor->mech_angle_accum_rad = mech_angle_rad;
        motor->mech_angle_prev_valid = 1U;
        return;
    }

    delta = Math_WrapRadDelta(mech_angle_rad - motor->mech_angle_prev_rad);
    if (fabsf(delta) >= g_foc_runtime_cfg.min_mech_angle_accum_delta_rad)
    {
        motor->mech_angle_accum_rad += delta;
    }

    motor->mech_angle_prev_rad = mech_angle_rad;
}

static void FOC_ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void FOC_ResetSpeedState(void)
{
    g_speed_err_accum_rad = 0.0f;
    g_prev_mech_signed_rad = 0.0f;
    g_speed_state_valid = 0U;
}

static void FOC_ApplySvpwmPreLpf(float *phase_a, float *phase_b, float *phase_c)
{
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
    if ((phase_a == 0) || (phase_b == 0) || (phase_c == 0))
    {
        return;
    }

    if (g_svpwm_lpf_state_valid == 0U)
    {
        g_svpwm_lpf_phase_a = *phase_a;
        g_svpwm_lpf_phase_b = *phase_b;
        g_svpwm_lpf_phase_c = *phase_c;
        g_svpwm_lpf_state_valid = 1U;
        return;
    }

    *phase_a = Math_FirstOrderLpf(*phase_a,
                                  &g_svpwm_lpf_phase_a,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &g_svpwm_lpf_state_valid);
    *phase_b = Math_FirstOrderLpf(*phase_b,
                                  &g_svpwm_lpf_phase_b,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &g_svpwm_lpf_state_valid);
    *phase_c = Math_FirstOrderLpf(*phase_c,
                                  &g_svpwm_lpf_phase_c,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &g_svpwm_lpf_state_valid);
#else
    (void)phase_a;
    (void)phase_b;
    (void)phase_c;
#endif
}

static float FOC_UpdateSpeedAngleError(foc_motor_t *motor,
                                       float mech_angle_rad,
                                       float speed_ref_rad_s,
                                       float dt_sec)
{
    float mech_signed_rad;
    float mech_delta_rad;
    float speed_cmd_delta_rad;

    if (motor == 0)
    {
        return 0.0f;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    mech_signed_rad = motor->direction * mech_angle_rad;

    if (g_speed_state_valid == 0U)
    {
        g_prev_mech_signed_rad = mech_signed_rad;
        g_speed_err_accum_rad = 0.0f;
        g_speed_state_valid = 1U;
        return 0.0f;
    }

    mech_delta_rad = Math_WrapRadDelta(mech_signed_rad - g_prev_mech_signed_rad);
    g_prev_mech_signed_rad = mech_signed_rad;

    /* Control loop: rad/s * dt(s) => rad increment per control cycle */
    speed_cmd_delta_rad = speed_ref_rad_s * dt_sec;
    g_speed_err_accum_rad += speed_cmd_delta_rad - mech_delta_rad;

    g_speed_err_accum_rad = Math_ClampFloat(g_speed_err_accum_rad,
                                            -FOC_SPEED_ERR_ACCUM_LIMIT_RAD,
                                            FOC_SPEED_ERR_ACCUM_LIMIT_RAD);

    return g_speed_err_accum_rad;
}

void FOC_ControlRebaseMechanicalAngleAccum(foc_motor_t *motor, float mech_angle_rad)
{
    if (motor == 0)
    {
        return;
    }

    motor->mech_angle_accum_rad = mech_angle_rad;
    motor->mech_angle_prev_rad = mech_angle_rad;
    motor->mech_angle_prev_valid = 1U;
}

void FOC_ControlResetSpeedLoopState(void)
{
    FOC_ResetSpeedState();
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

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static float FOC_CurrentLoopPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float error_effective;
    float derivative;
    float output;
    float ki_scale;
    float ki_effective;
    float integral_leak;

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    error = target - measurement;
    if (fabsf(error) <= FOC_CURRENT_LOOP_ERROR_DEADBAND_A)
    {
        error_effective = 0.0f;
    }
    else
    {
        error_effective = error;
    }

    integral_leak = Math_ClampFloat(FOC_CURRENT_LOOP_INTEGRAL_SUPPRESS_LEAK, 0.0f, 1.0f);
    if (error_effective == 0.0f)
    {
        pid->integral *= integral_leak;
    }
    else
    {
        pid->integral += error_effective * dt_sec;
    }

    ki_scale = FOC_CurrentLoopComputeKiScale(fabsf(target));
    ki_effective = pid->ki * ki_scale;

    derivative = (error_effective - pid->prev_error) / dt_sec;
    output = pid->kp * error_effective + ki_effective * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (ki_effective > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error_effective - pid->kd * derivative) / ki_effective;
        float i_max = (pid->out_max - pid->kp * error_effective - pid->kd * derivative) / ki_effective;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
    }

    pid->prev_error = error_effective;
    return output;
}
#endif

static float FOC_AngleHoldPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    error = target - measurement;
    if (fabsf(error) <= g_foc_runtime_cfg.angle_hold_pid_deadband_rad)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        return 0.0f;
    }

    pid->integral += error * dt_sec;
    pid->integral = Math_ClampFloat(pid->integral,
                                    -g_foc_runtime_cfg.angle_hold_integral_limit,
                                    g_foc_runtime_cfg.angle_hold_integral_limit);

    derivative = (error - pid->prev_error) / dt_sec;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, i_min, i_max);
        pid->integral = Math_ClampFloat(pid->integral,
                                        -g_foc_runtime_cfg.angle_hold_integral_limit,
                                        g_foc_runtime_cfg.angle_hold_integral_limit);
    }

    pid->prev_error = error;
    return output;
}

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static void FOC_CurrentControlClosedLoopStep(foc_motor_t *motor,
                                             foc_pid_t *current_pid,
                                             const sensor_data_t *sensor,
                                             float dt_sec)
{
    float i_alpha;
    float i_beta;
    float id_measured;
    float iq_measured;
    float voltage_limit;
    float uq_cmd;

    Math_ClarkeTransform(sensor->current_a.output_value,
                         sensor->current_b.output_value,
                         sensor->current_c.output_value,
                         &i_alpha,
                         &i_beta);

    Math_ParkTransform(i_alpha,
                       i_beta,
                       motor->electrical_phase_angle,
                       &id_measured,
                       &iq_measured);
    (void)id_measured;

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_LOOP_IQ_LPF_ENABLE == FOC_CFG_ENABLE))
    iq_measured = Math_FirstOrderLpf(iq_measured,
                                     &g_current_iq_lpf_state,
                                     FOC_CURRENT_LOOP_IQ_LPF_ALPHA,
                                     &g_current_iq_lpf_state_valid);
#endif

    voltage_limit = Math_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    uq_cmd = FOC_CurrentLoopPIDRun(current_pid, motor->iq_target, iq_measured, dt_sec);
    motor->iq_measured = iq_measured;

    motor->ud = 0.0f;
    motor->uq = Math_ClampFloat(uq_cmd, -voltage_limit, voltage_limit);
}

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
static uint8_t FOC_CurrentSoftSwitchResolveActiveMode(float iq_ref_abs)
{
    float open_threshold;
    float closed_threshold;

    if (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN)
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        return g_current_soft_switch_status.active_mode;
    }

    if (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        return g_current_soft_switch_status.active_mode;
    }

    open_threshold = Math_ClampFloat(g_current_soft_switch_status.auto_open_iq_a, 0.0f, 1e6f);
    closed_threshold = Math_ClampFloat(g_current_soft_switch_status.auto_closed_iq_a, 0.0f, 1e6f);
    if (closed_threshold < (open_threshold + 1e-6f))
    {
        closed_threshold = open_threshold + 1e-6f;
    }

    if ((g_current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (g_current_soft_switch_status.active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
    {
        g_current_soft_switch_status.active_mode =
            (iq_ref_abs >= closed_threshold) ? FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED : FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        return g_current_soft_switch_status.active_mode;
    }

    if ((g_current_soft_switch_status.active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) &&
        (iq_ref_abs <= open_threshold))
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    }
    else if ((g_current_soft_switch_status.active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
             (iq_ref_abs >= closed_threshold))
    {
        g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
    }

    return g_current_soft_switch_status.active_mode;
}

static void FOC_CurrentControlSoftSwitchStep(foc_motor_t *motor,
                                             foc_pid_t *current_pid,
                                             const sensor_data_t *sensor,
                                             float dt_sec)
{
    float open_ud = 0.0f;
    float open_uq = 0.0f;
    float open_iq = 0.0f;
    float closed_ud;
    float closed_uq;
    float closed_iq;
    float blend_factor;
    float target_blend;
    uint8_t active_mode;

    if ((motor == 0) || (current_pid == 0) || (sensor == 0))
    {
        return;
    }

    FOC_CurrentControlClosedLoopStep(motor,
                                     current_pid,
                                     sensor,
                                     dt_sec);
    closed_ud = motor->ud;
    closed_uq = motor->uq;
    closed_iq = motor->iq_measured;

    FOC_CurrentLoopEstimateOpenLoopResistanceModel(motor,
                                                   motor->iq_target,
                                                   0.0f,
                                                   &open_ud,
                                                   &open_uq,
                                                   &open_iq);

    active_mode = FOC_CurrentSoftSwitchResolveActiveMode(fabsf(motor->iq_target));
    target_blend = (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) ? 1.0f : 0.0f;
    blend_factor = FOC_ControlSoftSwitchUpdateBlend(g_current_soft_switch_status.blend_factor,
                                                    &g_current_soft_switch_blend_initialized,
                                                    target_blend,
                                                    dt_sec);
    g_current_soft_switch_status.blend_factor = blend_factor;

    motor->ud = open_ud + (closed_ud - open_ud) * blend_factor;
    motor->uq = open_uq + (closed_uq - open_uq) * blend_factor;
    motor->iq_measured = open_iq + (closed_iq - open_iq) * blend_factor;
}
#endif
#endif

void FOC_CurrentControlStep(foc_motor_t *motor,
                            foc_pid_t *current_pid,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec)
{
    if (motor == 0)
    {
        return;
    }

    motor->electrical_phase_angle = Math_WrapRad(electrical_angle);
    dt_sec = FOC_NormalizeDt(dt_sec);

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    if ((sensor == 0) || (current_pid == 0))
    {
        return;
    }

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (g_current_soft_switch_status.enabled != 0U)
    {
        FOC_CurrentControlSoftSwitchStep(motor,
                                         current_pid,
                                         sensor,
                                         dt_sec);
    }
    else
#endif
    {
        FOC_SOFT_SWITCH_BLEND_RESET();

        if (g_current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN)
        {
            g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
            g_current_soft_switch_status.blend_factor = 0.0f;
            FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->iq_target, 0.0f);
        }
        else
        {
            g_current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
            g_current_soft_switch_status.blend_factor = 1.0f;
            FOC_CurrentControlClosedLoopStep(motor,
                                             current_pid,
                                             sensor,
                                             dt_sec);
        }
    }
#else
    (void)current_pid;
    (void)sensor;
    FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->iq_target, 0.0f);
#endif

    FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
}

void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 0U);
}

void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 1U);
}

void FOC_OpenLoopStep(foc_motor_t *motor,float voltage, float turn_speed)
{
    motor->electrical_phase_angle = Math_WrapRad(
        motor->electrical_phase_angle +
        FOC_MATH_TWO_PI * turn_speed * motor->pole_pairs * FOC_CONTROL_DT_DEFAULT_SEC * motor->direction);

    motor->ud = 0.0f;
    motor->uq = Math_ClampFloat(voltage, 0.0f, motor->set_voltage);

    FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
}

void FOC_SpeedOuterLoopStep(foc_motor_t *motor,
                            foc_pid_t *speed_pid,
                            float speed_ref_rad_s,
                            const sensor_data_t *sensor,
                            float dt_sec)
{
    float speed_angle_error_rad;

    if ((motor == 0) || (speed_pid == 0) || (sensor == 0))
    {
        return;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor,
                                                      sensor->mech_angle_rad.output_value,
                                                      speed_ref_rad_s,
                                                      dt_sec);

    motor->iq_target = FOC_PIDRunCore(speed_pid,
                                      speed_angle_error_rad,
                                      0.0f,
                                      dt_sec);
    motor->iq_target += FOC_ControlCoggingLookupIq(&g_cogging_comp_status,
                                                   g_cogging_comp_table_q15,
                                                   sensor->mech_angle_rad.output_value,
                                                   speed_ref_rad_s);
    motor->electrical_phase_angle = FOC_ControlMechanicalToElectricalAngle(motor,
                                                                            sensor->mech_angle_rad.output_value);
}

void FOC_SpeedAngleOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 float angle_ref_rad,
                                 float angle_position_speed_rad_s,
                                 const sensor_data_t *sensor,
                                 float dt_sec)
{
    float torque_ref_speed;
    float torque_ref_hold;
    float mech_signed_total_rad;
    float angle_error_rad;
    float abs_angle_error_rad;
    float transition_span_rad;
    float speed_blend;
    float speed_ref_rad_s;
    float speed_angle_error_rad;

    if ((motor == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (sensor == 0))
    {
        return;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

    angle_ref_rad *= motor->direction;

    FOC_UpdateAccumulatedMechanicalAngle(motor, sensor->mech_angle_rad.output_value);
    mech_signed_total_rad = motor->direction * motor->mech_angle_accum_rad;
    angle_error_rad = angle_ref_rad - mech_signed_total_rad;
    abs_angle_error_rad = fabsf(angle_error_rad);

    transition_span_rad = g_foc_runtime_cfg.speed_angle_transition_end_rad -
                          g_foc_runtime_cfg.speed_angle_transition_start_rad;
    if (transition_span_rad < 1e-6f)
    {
        transition_span_rad = 1e-6f;
    }

    if (abs_angle_error_rad <= g_foc_runtime_cfg.speed_angle_transition_start_rad)
    {
        speed_blend = 0.0f;
    }
    else if (abs_angle_error_rad >= g_foc_runtime_cfg.speed_angle_transition_end_rad)
    {
        speed_blend = 1.0f;
    }
    else
    {
        speed_blend = (abs_angle_error_rad - g_foc_runtime_cfg.speed_angle_transition_start_rad) / transition_span_rad;
    }

    speed_ref_rad_s = ((angle_error_rad >= 0.0f) ? fabsf(angle_position_speed_rad_s) : -fabsf(angle_position_speed_rad_s)) * speed_blend;

    if (speed_blend < 1e-4f)
    {
        FOC_ResetPIDState(speed_pid);
        FOC_ResetSpeedState();
        torque_ref_speed = 0.0f;
    }
    else
    {
        speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor,
                                  sensor->mech_angle_rad.output_value,
                                  speed_ref_rad_s,
                                  dt_sec);
        torque_ref_speed = FOC_PIDRunCore(speed_pid,
                                          speed_angle_error_rad,
                                          0.0f,
                                          dt_sec);
    }

    torque_ref_hold = FOC_AngleHoldPIDRun(angle_hold_pid,
                                          angle_ref_rad,
                                          mech_signed_total_rad,
                                          dt_sec);

    motor->iq_target = (1.0f - speed_blend) * torque_ref_hold + speed_blend * torque_ref_speed;
    motor->iq_target += FOC_ControlCoggingLookupIq(&g_cogging_comp_status,
                                                   g_cogging_comp_table_q15,
                                                   sensor->mech_angle_rad.output_value,
                                                   speed_ref_rad_s);
    motor->electrical_phase_angle = FOC_ControlMechanicalToElectricalAngle(motor,
                                                                            sensor->mech_angle_rad.output_value);
}

uint8_t FOC_ControlOuterLoopStep(foc_motor_t *motor,
                                 foc_pid_t *current_pid,
                                 foc_pid_t *speed_pid,
                                 foc_pid_t *angle_hold_pid,
                                 const sensor_data_t *sensor,
                                 uint8_t control_mode,
                                 float speed_only_rad_s,
                                 float target_angle_rad,
                                 float angle_position_speed_rad_s,
                                 float dt_sec)
{
    if ((motor == 0) || (speed_pid == 0) || (sensor == 0))
    {
        return 0U;
    }

#if (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ONLY)
    (void)current_pid;
    (void)angle_hold_pid;
    (void)control_mode;
    (void)target_angle_rad;
    (void)angle_position_speed_rad_s;
    FOC_SpeedOuterLoopStep(motor,
                           speed_pid,
                           speed_only_rad_s,
                           sensor,
                           dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY)
    (void)current_pid;
    (void)control_mode;
    (void)speed_only_rad_s;
    if (angle_hold_pid == 0)
    {
        return 0U;
    }
    FOC_SpeedAngleOuterLoopStep(motor,
                                speed_pid,
                                angle_hold_pid,
                                target_angle_rad,
                                angle_position_speed_rad_s,
                                sensor,
                                dt_sec);
    return 1U;
#elif (FOC_BUILD_CONTROL_ALGO_SET == FOC_CTRL_ALGO_BUILD_FULL)
    if (angle_hold_pid == 0)
    {
        return 0U;
    }

    if ((control_mode != COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY) &&
        (control_mode != COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE))
    {
        return 0U;
    }

    if (g_prev_control_mode_valid == 0U)
    {
        g_prev_control_mode = control_mode;
        g_prev_control_mode_valid = 1U;
    }

    if (control_mode != g_prev_control_mode)
    {
        if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ANGLE)
        {
            FOC_ControlRebaseMechanicalAngleAccum(motor, sensor->mech_angle_rad.output_value);
        }

        FOC_ResetPIDState(current_pid);
        FOC_ResetPIDState(speed_pid);
        FOC_ResetPIDState(angle_hold_pid);
        FOC_ControlResetSpeedLoopState();
        g_prev_control_mode = control_mode;
    }

    if (control_mode == COMMAND_MANAGER_CONTROL_MODE_SPEED_ONLY)
    {
        FOC_SpeedOuterLoopStep(motor,
                               speed_pid,
                               speed_only_rad_s,
                               sensor,
                               dt_sec);
    }
    else
    {
        FOC_SpeedAngleOuterLoopStep(motor,
                                    speed_pid,
                                    angle_hold_pid,
                                    target_angle_rad,
                                    angle_position_speed_rad_s,
                                    sensor,
                                    dt_sec);
    }

    return 1U;
#else
#error "Unsupported FOC_BUILD_CONTROL_ALGO_SET"
#endif
}

uint8_t FOC_ControlRequiresCurrentSample(void)
{
#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    return 1U;
#else
    return 0U;
#endif
}
