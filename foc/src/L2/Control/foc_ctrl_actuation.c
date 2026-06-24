#include "L2/Control/foc_ctrl_actuation.h"

#include <math.h>

#include "L3/foc_svpwm.h"
#include "L3/foc_math_transforms.h"
#include "L3/foc_math_lut.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* SVPWM 前置低通滤波（per-motor 状态）*/
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
static void FOC_ApplySvpwmPreLpf(foc_motor_t *motor,
                                 float *phase_a,
                                 float *phase_b,
                                 float *phase_c)
{
    if ((phase_a == 0) || (phase_b == 0) || (phase_c == 0))
    {
        return;
    }

    if (motor->svpwm_lpf_state_valid == 0U)
    {
        motor->svpwm_lpf_phase_a = *phase_a;
        motor->svpwm_lpf_phase_b = *phase_b;
        motor->svpwm_lpf_phase_c = *phase_c;
        motor->svpwm_lpf_state_valid = 1U;
        return;
    }

    *phase_a = Math_FirstOrderLpf(*phase_a,
                                  &motor->svpwm_lpf_phase_a,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &motor->svpwm_lpf_state_valid);
    *phase_b = Math_FirstOrderLpf(*phase_b,
                                  &motor->svpwm_lpf_phase_b,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &motor->svpwm_lpf_state_valid);
    *phase_c = Math_FirstOrderLpf(*phase_c,
                                  &motor->svpwm_lpf_phase_c,
                                  FOC_SVPWM_PRE_LPF_ALPHA,
                                  &motor->svpwm_lpf_state_valid);
}
#endif

/* 核心：将电机dq电压+电角度转换为三相PWM占空比输出 */
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

    /* SVPWM过调制限制：dq矢量幅度超过电压限制时等比缩放 */
    if ((dq_magnitude > voltage_limit) && (dq_magnitude > 1e-6f))
    {
        float scale = voltage_limit / dq_magnitude;
        ud_applied *= scale;
        uq_applied *= scale;
        dq_magnitude = voltage_limit;
    }

    /* 逆Park变换：dq -> alpha-beta */
    Math_InverseParkTransform(ud_applied,
                              uq_applied,
                              electrical_angle,
                              &motor->alpha,
                              &motor->beta);

    /* 逆Clarke变换：alpha-beta -> 三相电压 */
    Math_InverseClarkeTransform(motor->alpha,
                                motor->beta,
                                &motor->phase_a,
                                &motor->phase_b,
                                &motor->phase_c);

#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
    FOC_ApplySvpwmPreLpf(motor,
                         &motor->phase_a,
                         &motor->phase_b,
                         &motor->phase_c);
#endif

    voltage_command = Math_ClampFloat(dq_magnitude, 0.0f, voltage_limit);

    /* 零矢量钳位：电压命令过低时直接输出50%占空比（中点）以降低开关损耗 */
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

    /* 正常SVPWM输出 */
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

/* C31：将机械角度转换为电角度（基于极对数和机械零点） */
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

/* C31：采样锁定的机械角度（用于电机零点标定） */
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

    /* 在锁定状态下多次采样，通过sin/cos矢量平均抑制噪声 */
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

/* C31：运行时应用电角度（使用插值SVPWM输出）*/
void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 0U);
}

/* C31：直接应用电角度（直接写入占空比，不插值） */
void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 1U);
}