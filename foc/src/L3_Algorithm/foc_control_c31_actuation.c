#include "L3_Algorithm/foc_control_c31_actuation.h"

#include <math.h>

#include "L3_Algorithm/svpwm.h"
#include "L41_Math/math_transforms.h"
#include "L41_Math/foc_math_lut.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_svpwm_lpf_state_valid = 0U;
static float g_svpwm_lpf_phase_a = 0.0f;
static float g_svpwm_lpf_phase_b = 0.0f;
static float g_svpwm_lpf_phase_c = 0.0f;

static void FOC_ApplySvpwmPreLpf(float *phase_a, float *phase_b, float *phase_c)
{
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
}
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
                                
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)

    FOC_ApplySvpwmPreLpf(&motor->phase_a,
                         &motor->phase_b,
                         &motor->phase_c);

#endif

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

uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
                                        float electrical_angle,
                                        uint16_t settle_ms,
                                        uint16_t sample_count,
                                        float *mech_angle_rad)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t i;

    if ((motor == 0) || (mech_angle_rad == 0) || (sample_count == 0U))
    {
        return 0U;
    }

    FOC_ControlApplyElectricalAngleDirect(motor, electrical_angle);
    FOC_Platform_WaitMs(settle_ms);

    for (i = 0U; i < sample_count; i++)
    {
        float sample_rad;

        if (FOC_Platform_ReadMechanicalAngleRad(&sample_rad) == 0U)
        {
            continue;
        }

        sin_sum += FOC_MathLut_Sin(sample_rad);
        cos_sum += FOC_MathLut_Sin(sample_rad + FOC_MATH_PI * 0.5f);
        FOC_Platform_WaitMs(FOC_CALIB_SETTLE_MS);
    }

    if ((fabsf(sin_sum) < 1e-6f) && (fabsf(cos_sum) < 1e-6f))
    {
        return 0U;
    }

    *mech_angle_rad = Math_WrapRad(FOC_MathLut_Atan2(sin_sum, cos_sum));
    return 1U;
}

void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 0U);
}

void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 1U);
}
