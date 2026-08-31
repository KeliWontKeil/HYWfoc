#include "L2_Core/Control/foc_ctrl_actuation.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* 核心：将电机dq电压+电角度转换为三相PWM占空比输出 */
static void FOC_ControlApplyElectricalAngleCore(foc_control_runtime_t *ctrl,
                                                svpwm_interp_state_t *svpwm,
                                                foc_applied_output_state_t *applied,
                                                foc_alpha_beta_phase_t *alpha_beta,
                                                const foc_motor_params_t *params,
                                                float electrical_angle,
                                                uint8_t direct_output)
{
    float dq_magnitude;
    float voltage_limit;
    float voltage_command;
    float ud_applied;
    float uq_applied;
    float sin_theta;
    float cos_theta;

    electrical_angle = Math_WrapRad(electrical_angle);

    voltage_limit = Math_ClampFloat(ctrl->max_phase_voltage, 0.0f, params->vbus_voltage);

    /* SVPWM 最大占空比限制，保护低侧电流采样 */
    {
        float duty_limit_v = params->vbus_voltage * (2.0f * FOC_SVPWM_MAX_DUTY_CYCLE - 1.0f);
        if (duty_limit_v < 0.0f) duty_limit_v = 0.0f;
        if (voltage_limit > duty_limit_v) voltage_limit = duty_limit_v;
    }

    dq_magnitude = sqrtf(ctrl->ud * ctrl->ud + ctrl->uq * ctrl->uq);
    ud_applied = ctrl->ud;
    uq_applied = ctrl->uq;

    /* SVPWM过调制限制：dq矢量幅度超过电压限制时等比缩放 */
    if ((dq_magnitude > voltage_limit) && (dq_magnitude > 1e-6f))
    {
        float scale = voltage_limit / dq_magnitude;
        ud_applied *= scale;
        uq_applied *= scale;
        dq_magnitude = voltage_limit;
    }

    applied->valid = 1U;
    if (direct_output != 0U)
    {
        ctrl->electrical_angle_rad = electrical_angle;
    }
    applied->ud = ud_applied;
    applied->uq = uq_applied;
    applied->electrical_angle_rad = electrical_angle;

    /* 逆Park变换：dq -> alpha-beta（sin/cos 联合查表一次取得） */
    FOC_MathLut_SinCos(electrical_angle, &sin_theta, &cos_theta);
    Math_InverseParkTransformSC(ud_applied,
                                uq_applied,
                                sin_theta,
                                cos_theta,
                                &alpha_beta->alpha,
                                &alpha_beta->beta);

    voltage_command = Math_ClampFloat(dq_magnitude, 0.0f, voltage_limit);

    /* 零矢量钳位：电压命令过低时直接输出50%占空比（中点）以降低开关损耗 */
#if (FOC_ZERO_VECTOR_CLAMP_ENABLE == FOC_CFG_ENABLE)
    if (voltage_command < FOC_ZERO_VECTOR_CLAMP_VOLTAGE_THRESHOLD_V)
    {
        svpwm->output.sector = 0U;
        svpwm->output.duty_a = 0.5f;
        svpwm->output.duty_b = 0.5f;
        svpwm->output.duty_c = 0.5f;

        if (direct_output != 0U)
        {
            SVPWM_ApplyDirectDuty(svpwm,
                                  svpwm->output.sector,
                                  svpwm->output.duty_a,
                                  svpwm->output.duty_b,
                                  svpwm->output.duty_c);
        }
        else
        {
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
            SVPWM_SetRuntimeDutyTarget(svpwm,
                                       svpwm->output.sector,
                                       svpwm->output.duty_a,
                                       svpwm->output.duty_b,
                                       svpwm->output.duty_c);
#else
            SVPWM_ApplyDirectDuty(svpwm,
                                  svpwm->output.sector,
                                  svpwm->output.duty_a,
                                  svpwm->output.duty_b,
                                  svpwm->output.duty_c);
#endif
        }
        return;
    }
#endif

    /* 正常SVPWM输出：αβ 直通（逆 Park 结果即 SVPWM 输入，不再经三相往返） */
    SVPWM_Update(svpwm,
                 alpha_beta->alpha,
                 alpha_beta->beta,
                 voltage_command,
                 params->vbus_voltage,
                 direct_output);
}

void FOC_ControlRecordPhaseOutputDqAngle(foc_phase_output_state_t *phase_output,
                                         foc_control_runtime_t *ctrl,
                                         uint8_t phase,
                                         uint8_t state_id,
                                         float electrical_angle,
                                         float ud,
                                         float uq)
{
    phase_output->phase = phase;
    phase_output->type = FOC_PHASE_OUTPUT_DQ_VOLTAGE_ANGLE;
    phase_output->valid = 1U;
    phase_output->state_id = state_id;
    ctrl->electrical_angle_rad = Math_WrapRad(electrical_angle);
    ctrl->ud = ud;
    ctrl->uq = uq;
    phase_output->duty_a = 0.0f;
    phase_output->duty_b = 0.0f;
    phase_output->duty_c = 0.0f;
    phase_output->sector = 0U;
}

void FOC_ControlRecordPhaseOutputZero(foc_phase_output_state_t *phase_output,
                                      foc_control_runtime_t *ctrl,
                                      foc_outer_loop_private_t *outer_loop,
                                      uint8_t phase,
                                      uint8_t state_id)
{
    phase_output->phase = phase;
    phase_output->type = FOC_PHASE_OUTPUT_ZERO;
    phase_output->valid = 1U;
    phase_output->state_id = state_id;
    phase_output->duty_a = 0.0f;
    phase_output->duty_b = 0.0f;
    phase_output->duty_c = 0.0f;
    phase_output->sector = 0U;
    ctrl->ud = 0.0f;
    ctrl->uq = 0.0f;
    outer_loop->ramped_speed_rad_s = 0.0f;
}

void FOC_ControlApplyPhaseOutputRuntime(foc_control_runtime_t *ctrl,
                                        svpwm_interp_state_t *svpwm,
                                        foc_applied_output_state_t *applied,
                                        foc_alpha_beta_phase_t *alpha_beta,
                                        foc_phase_output_state_t *phase_output,
                                        const foc_motor_params_t *params)
{
    if (phase_output->valid == 0U) return;

    switch (phase_output->type)
    {
    case FOC_PHASE_OUTPUT_ZERO:
        ctrl->ud = 0.0f;
        ctrl->uq = 0.0f;
        applied->valid = 1U;
        applied->ud = 0.0f;
        applied->uq = 0.0f;
        applied->electrical_angle_rad = ctrl->electrical_angle_rad;
        SVPWM_ApplyDirectDuty(svpwm, 0U, 0.0f, 0.0f, 0.0f);
        break;

    case FOC_PHASE_OUTPUT_DQ_VOLTAGE_ANGLE:
        FOC_ControlApplyElectricalAngleRuntime(ctrl, svpwm, applied, alpha_beta, params,
                                               ctrl->electrical_angle_rad);
        break;

    case FOC_PHASE_OUTPUT_DIRECT_DUTY:
        applied->valid = 1U;
        applied->ud = ctrl->ud;
        applied->uq = ctrl->uq;
        applied->electrical_angle_rad = ctrl->electrical_angle_rad;
        SVPWM_ApplyDirectDuty(svpwm,
                              phase_output->sector,
                              phase_output->duty_a,
                              phase_output->duty_b,
                              phase_output->duty_c);
        break;

    default:
        break;
    }
}

/* C31：将机械角度转换为电角度（基于极对数和机械零点） */
float FOC_ControlMechanicalToElectricalAngle(const foc_motor_params_t *params,
                                             float fallback_elec_angle_rad,
                                             float mech_angle_rad)
{
    float mech_delta;

    if ((params->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
        (params->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        return fallback_elec_angle_rad;
    }

    mech_delta = Math_WrapRadDelta(mech_angle_rad - params->mech_angle_at_elec_zero_rad);
    return Math_WrapRad((float)params->direction * mech_delta * (float)params->pole_pairs);
}

/* C31：采样锁定的机械角度（用于电机零点标定） */
uint8_t FOC_SampleLockedMechanicalAngle(foc_control_runtime_t *ctrl,
                                        svpwm_interp_state_t *svpwm,
                                        foc_applied_output_state_t *applied,
                                        foc_alpha_beta_phase_t *alpha_beta,
                                        const foc_motor_params_t *params,
                                        float electrical_angle,
                                        uint16_t settle_ms,
                                        uint16_t sample_count,
                                        float *mech_angle_rad)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t i;

    if ((mech_angle_rad == 0) || (sample_count == 0U))
    {
        return 0U;
    }

    FOC_ControlApplyElectricalAngleDirect(ctrl, svpwm, applied, alpha_beta, params,
                                          electrical_angle);
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

/* C31：运行时应用电角度（插值启用走插值路径，裁剪后直接写占空比）*/
void FOC_ControlApplyElectricalAngleRuntime(foc_control_runtime_t *ctrl,
                                            svpwm_interp_state_t *svpwm,
                                            foc_applied_output_state_t *applied,
                                            foc_alpha_beta_phase_t *alpha_beta,
                                            const foc_motor_params_t *params,
                                            float electrical_angle)
{
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    FOC_ControlApplyElectricalAngleCore(ctrl, svpwm, applied, alpha_beta, params,
                                        electrical_angle, 0U);
#else
    FOC_ControlApplyElectricalAngleCore(ctrl, svpwm, applied, alpha_beta, params,
                                        electrical_angle, 1U);
#endif
}

/* C31：直接应用电角度（直接写入占空比，不插值） */
void FOC_ControlApplyElectricalAngleDirect(foc_control_runtime_t *ctrl,
                                           svpwm_interp_state_t *svpwm,
                                           foc_applied_output_state_t *applied,
                                           foc_alpha_beta_phase_t *alpha_beta,
                                           const foc_motor_params_t *params,
                                           float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(ctrl, svpwm, applied, alpha_beta, params,
                                        electrical_angle, 1U);
}
