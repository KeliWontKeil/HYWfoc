#include "L2/Control/foc_ctrl_actuation.h"

#include <math.h>

#include "L2/Control/foc_svpwm.h"
#include "L3/foc_math_transforms.h"
#include "L3/foc_math_lut.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* SVPWM鍓嶇疆浣庨€氭护娉㈢姸鎬?*/
#if (FOC_SVPWM_PRE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_svpwm_lpf_state_valid = 0U;
static float g_svpwm_lpf_phase_a = 0.0f;
static float g_svpwm_lpf_phase_b = 0.0f;
static float g_svpwm_lpf_phase_c = 0.0f;

/* 瀵筍VPWM杈撳叆涓夌浉鐢靛帇杩涜涓€闃朵綆閫氭护娉?*/
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

/* 鏍稿績锛氬皢鐢垫満dq鐢靛帇+鐢佃搴﹁浆鎹负涓夌浉PWM鍗犵┖姣旇緭鍑?*/
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

    /* SVPWM杩囪皟鍒堕檺鍒讹細dq鐭㈤噺骞呭害瓒呰繃鐢靛帇闄愬埗鏃剁瓑姣旂缉鏀?*/
    if ((dq_magnitude > voltage_limit) && (dq_magnitude > 1e-6f))
    {
        float scale = voltage_limit / dq_magnitude;
        ud_applied *= scale;
        uq_applied *= scale;
        dq_magnitude = voltage_limit;
    }

    /* 閫哖ark鍙樻崲锛歞q -> alpha-beta */
    Math_InverseParkTransform(ud_applied,
                              uq_applied,
                              electrical_angle,
                              &motor->alpha,
                              &motor->beta);

    /* 閫咰larke鍙樻崲锛歛lpha-beta -> 涓夌浉鐢靛帇 */
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

    /* 闆剁煝閲忛挸浣嶏細鐢靛帇鍛戒护杩囦綆鏃剁洿鎺ヨ緭鍑?0%鍗犵┖姣旓紙涓偣锛変互闄嶄綆寮€鍏虫崯鑰?*/
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

    /* 姝ｅ父SVPWM杈撳嚭 */
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

/* C31锛氬皢鏈烘瑙掑害杞崲涓虹數瑙掑害锛堝熀浜庢瀬瀵规暟鍜屾満姊伴浂鐐癸級 */
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

/* C31锛氶噰鏍烽攣瀹氱殑鏈烘瑙掑害锛堢敤浜庣數鏈洪浂鐐规爣瀹氾級 */
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

    /* 鍦ㄩ攣瀹氱姸鎬佷笅澶氭閲囨牱锛岄€氳繃sin/cos鐭㈤噺骞冲潎鎶戝埗鍣０ */
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

/* C31锛氳繍琛屾椂搴旂敤鐢佃搴︼紙浣跨敤鎻掑€糞VPWM杈撳嚭锛?*/
void FOC_ControlApplyElectricalAngleRuntime(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 0U);
}

/* C31锛氱洿鎺ュ簲鐢ㄧ數瑙掑害锛堢洿鎺ュ啓鍏ュ崰绌烘瘮锛屼笉鎻掑€硷級 */
void FOC_ControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle)
{
    FOC_ControlApplyElectricalAngleCore(motor, electrical_angle, 1U);
}