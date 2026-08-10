#include "L3_Hal/foc_svpwm.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

static float SVPWM_Sqrt(float value)
{
    return sqrtf(value);
}

static float SVPWM_Sin(float value)
{
    return FOC_MathLut_Sin(value);
}

static float SVPWM_Atan2(float y, float x)
{
    return FOC_MathLut_Atan2(y, x);
}

static float SVPWM_Clamp01(float value)
{
    if (value < 0.0f)
    {
        return 0.0f;
    }

    if (value > 1.0f)
    {
        return 1.0f;
    }

    return value;
}

static uint8_t SVPWM_DetermineSector(float alpha, float beta)
{
    float u1 = beta;
    float u2 = FOC_MATH_SQRT3_BY_2 * alpha - 0.5f * beta;
    float u3 = -FOC_MATH_SQRT3_BY_2 * alpha - 0.5f * beta;
    uint8_t code = 0;

    if (u1 > 0.0f)
    {
        code |= 0x01U;
    }

    if (u2 > 0.0f)
    {
        code |= 0x02U;
    }

    if (u3 > 0.0f)
    {
        code |= 0x04U;
    }

    switch (code)
    {
        case 3U: return 1U;
        case 1U: return 2U;
        case 5U: return 3U;
        case 4U: return 4U;
        case 6U: return 5U;
        case 2U: return 6U;
        default: return 0U;
    }
}

static void SVPWM_CalculateDuty(float phase_a,
                                float phase_b,
                                float phase_c,
                                float voltage_command,
                                float vbus_voltage,
                                uint8_t *sector_out,
                                float *duty_a,
                                float *duty_b,
                                float *duty_c)
{
    float alpha;
    float beta;
    float modulation;
    float magnitude;
    float theta;
    float theta_sector;
    float t_sum;
    float t1;
    float t2;
    float t0;
    uint8_t sector_id;

    if ((sector_out == 0) || (duty_a == 0) || (duty_b == 0) || (duty_c == 0))
    {
        return;
    }

    if (vbus_voltage <= FOC_MATH_EPSILON)
    {
        *sector_out = 0U;
        *duty_a = 0.5f;
        *duty_b = 0.5f;
        *duty_c = 0.5f;
        return;
    }

    modulation = voltage_command / vbus_voltage;
    if (modulation < 0.0f)
    {
        modulation = -modulation;
    }
    if (modulation > 1.0f)
    {
        modulation = 1.0f;
    }

    alpha = phase_a;
    beta = (phase_b - phase_c) / FOC_MATH_SQRT3;
    magnitude = SVPWM_Sqrt(alpha * alpha + beta * beta);

    if (magnitude < FOC_MATH_EPSILON)
    {
        *sector_out = 0U;
        *duty_a = 0.5f;
        *duty_b = 0.5f;
        *duty_c = 0.5f;
        return;
    }

    alpha /= magnitude;
    beta /= magnitude;
    magnitude = modulation;

    theta = SVPWM_Atan2(beta, alpha);
    if (theta < 0.0f)
    {
        theta += FOC_MATH_TWO_PI;
    }

    sector_id = SVPWM_DetermineSector(alpha, beta);
    if (sector_id == 0U)
    {
        sector_id = (uint8_t)(theta / FOC_MATH_PI_BY_3) + 1U;
        if (sector_id > 6U)
        {
            sector_id = 6U;
        }
    }

    theta_sector = theta - (float)(sector_id - 1U) * FOC_MATH_PI_BY_3;

    t1 = magnitude * SVPWM_Sin(FOC_MATH_PI_BY_3 - theta_sector);
    t2 = magnitude * SVPWM_Sin(theta_sector);

    if (t1 < 0.0f)
    {
        t1 = 0.0f;
    }
    if (t2 < 0.0f)
    {
        t2 = 0.0f;
    }

    t_sum = t1 + t2;
    if (t_sum > 1.0f)
    {
        t1 /= t_sum;
        t2 /= t_sum;
        t_sum = 1.0f;
    }

    t0 = 1.0f - t_sum;

    switch (sector_id)
    {
        case 1U:
            *duty_a = t1 + t2 + 0.5f * t0;
            *duty_b = t2 + 0.5f * t0;
            *duty_c = 0.5f * t0;
            break;
        case 2U:
            *duty_a = t1 + 0.5f * t0;
            *duty_b = t1 + t2 + 0.5f * t0;
            *duty_c = 0.5f * t0;
            break;
        case 3U:
            *duty_a = 0.5f * t0;
            *duty_b = t1 + t2 + 0.5f * t0;
            *duty_c = t2 + 0.5f * t0;
            break;
        case 4U:
            *duty_a = 0.5f * t0;
            *duty_b = t1 + 0.5f * t0;
            *duty_c = t1 + t2 + 0.5f * t0;
            break;
        case 5U:
            *duty_a = t2 + 0.5f * t0;
            *duty_b = 0.5f * t0;
            *duty_c = t1 + t2 + 0.5f * t0;
            break;
        case 6U:
            *duty_a = t1 + t2 + 0.5f * t0;
            *duty_b = 0.5f * t0;
            *duty_c = t1 + 0.5f * t0;
            break;
        default:
            *duty_a = 0.5f;
            *duty_b = 0.5f;
            *duty_c = 0.5f;
            break;
    }

    *duty_a = SVPWM_Clamp01(*duty_a);
    *duty_b = SVPWM_Clamp01(*duty_b);
    *duty_c = SVPWM_Clamp01(*duty_c);
    *sector_out = sector_id;
}

void SVPWM_Init(svpwm_interp_state_t *svpwm)
{
    FOC_Platform_PWMInit();
    FOC_Platform_PWMSetDutyCycleTripleFloat(0.0f, 0.0f, 0.0f);
    FOC_Platform_PWMStart();

#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    svpwm->interp_steps_total = (FOC_PWM_FREQ_KHZ > 0U) ? FOC_PWM_FREQ_KHZ : 1U;
    svpwm->interp_step_index = svpwm->interp_steps_total;
#endif
    svpwm->duty_a_current = 0.0f;
    svpwm->duty_b_current = 0.0f;
    svpwm->duty_c_current = 0.0f;
}

#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
void SVPWM_SetRuntimeDutyTarget(svpwm_interp_state_t *svpwm,
                                uint8_t sector,
                                float duty_a,
                                float duty_b,
                                float duty_c)
{
    svpwm->output.sector = sector;
    svpwm->output.duty_a = SVPWM_Clamp01(duty_a);
    svpwm->output.duty_b = SVPWM_Clamp01(duty_b);
    svpwm->output.duty_c = SVPWM_Clamp01(duty_c);

#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    /* 三 ISR 模式：写端(电流环 ISR)仅写 pending 目标；步长由 PWM ISR 取走时按当时 duty_current 计算，
     * 消除"电流环读 duty_current 算步长"的跨 ISR 步长基座竞态。 */
    svpwm->pending_duty_a_target = svpwm->output.duty_a;
    svpwm->pending_duty_b_target = svpwm->output.duty_b;
    svpwm->pending_duty_c_target = svpwm->output.duty_c;

    FOC_Platform_MemoryBarrier();
    svpwm->target_pending = 1U;
#else
    /* 双 ISR 模式：同 ISR 内写入，无竞争 */
    svpwm->duty_a_target = svpwm->output.duty_a;
    svpwm->duty_b_target = svpwm->output.duty_b;
    svpwm->duty_c_target = svpwm->output.duty_c;

    svpwm->duty_a_step = (svpwm->duty_a_target - svpwm->duty_a_current) / (float)svpwm->interp_steps_total;
    svpwm->duty_b_step = (svpwm->duty_b_target - svpwm->duty_b_current) / (float)svpwm->interp_steps_total;
    svpwm->duty_c_step = (svpwm->duty_c_target - svpwm->duty_c_current) / (float)svpwm->interp_steps_total;
    svpwm->interp_step_index = 0U;
#endif
}
#endif

void SVPWM_ApplyDirectDuty(svpwm_interp_state_t *svpwm,
                           uint8_t sector,
                           float duty_a,
                           float duty_b,
                           float duty_c)
{
    svpwm->output.sector = sector;
    svpwm->output.duty_a = SVPWM_Clamp01(duty_a);
    svpwm->output.duty_b = SVPWM_Clamp01(duty_b);
    svpwm->output.duty_c = SVPWM_Clamp01(duty_c);

    svpwm->duty_a_current = svpwm->output.duty_a;
    svpwm->duty_b_current = svpwm->output.duty_b;
    svpwm->duty_c_current = svpwm->output.duty_c;
#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    svpwm->duty_a_target  = svpwm->output.duty_a;
    svpwm->duty_b_target  = svpwm->output.duty_b;
    svpwm->duty_c_target  = svpwm->output.duty_c;
    svpwm->duty_a_step = 0.0f;
    svpwm->duty_b_step = 0.0f;
    svpwm->duty_c_step = 0.0f;
    svpwm->interp_step_index = svpwm->interp_steps_total;
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    svpwm->pending_duty_a_target = svpwm->output.duty_a;
    svpwm->pending_duty_b_target = svpwm->output.duty_b;
    svpwm->pending_duty_c_target = svpwm->output.duty_c;
    FOC_Platform_MemoryBarrier();
    svpwm->target_pending = 1U;
#endif
#endif

    FOC_Platform_PWMSetDutyCycleTripleFloat(svpwm->duty_a_current,
                                            svpwm->duty_b_current,
                                            svpwm->duty_c_current);
}

void SVPWM_Update(svpwm_interp_state_t *svpwm,
                  float phase_a,
                  float phase_b,
                  float phase_c,
                  float voltage_command,
                  float vbus_voltage,
                  uint8_t direct_output)
{
    uint8_t sector;
    float duty_a, duty_b, duty_c;

    SVPWM_CalculateDuty(phase_a, phase_b, phase_c,
                        voltage_command, vbus_voltage,
                        &sector, &duty_a, &duty_b, &duty_c);

#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
    if (direct_output != 0U)
    {
        SVPWM_ApplyDirectDuty(svpwm, sector, duty_a, duty_b, duty_c);
    }
    else
    {
        SVPWM_SetRuntimeDutyTarget(svpwm, sector, duty_a, duty_b, duty_c);
    }
#else
    /* 插值裁剪：一律直接写占空比 */
    (void)direct_output;
    SVPWM_ApplyDirectDuty(svpwm, sector, duty_a, duty_b, duty_c);
#endif
}

#if (FOC_SVPWM_INTERP_ENABLE == FOC_CFG_ENABLE)
void SVPWM_InterpolationISR(svpwm_interp_state_t *svpwm)
{
#if (FOC_CURRENT_LOOP_ISR_MODE == FOC_ISR_MODE_3ISR)
    /* 三 ISR 模式：PWM ISR 入口原子取走 pending 目标，并基于当时 duty_current 即时计算步长，
     * 保证每次重启插值都从真实当前占空比朝目标平滑过渡（消除跨 ISR 步长基座竞态）。 */
    if (svpwm->target_pending != 0U)
    {
        svpwm->duty_a_target = svpwm->pending_duty_a_target;
        svpwm->duty_b_target = svpwm->pending_duty_b_target;
        svpwm->duty_c_target = svpwm->pending_duty_c_target;
        svpwm->duty_a_step = (svpwm->duty_a_target - svpwm->duty_a_current) / (float)svpwm->interp_steps_total;
        svpwm->duty_b_step = (svpwm->duty_b_target - svpwm->duty_b_current) / (float)svpwm->interp_steps_total;
        svpwm->duty_c_step = (svpwm->duty_c_target - svpwm->duty_c_current) / (float)svpwm->interp_steps_total;
        svpwm->interp_step_index = 0U;
        svpwm->target_pending = 0U;
    }
#endif

    if (svpwm->interp_step_index < svpwm->interp_steps_total)
    {
        svpwm->duty_a_current += svpwm->duty_a_step;
        svpwm->duty_b_current += svpwm->duty_b_step;
        svpwm->duty_c_current += svpwm->duty_c_step;
        svpwm->interp_step_index++;

        if (svpwm->interp_step_index >= svpwm->interp_steps_total)
        {
            svpwm->duty_a_current = svpwm->duty_a_target;
            svpwm->duty_b_current = svpwm->duty_b_target;
            svpwm->duty_c_current = svpwm->duty_c_target;
        }
    }

    FOC_Platform_PWMSetDutyCycleTripleFloat(svpwm->duty_a_current, svpwm->duty_b_current, svpwm->duty_c_current);
}
#endif

const svpwm_output_t* SVPWM_GetOutput(const svpwm_interp_state_t *svpwm)
{
    if (svpwm == 0) return 0;
    return &svpwm->output;
}
