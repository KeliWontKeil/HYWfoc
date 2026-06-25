#include "L3_Hal/foc_svpwm.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

static float SVPWM_Sqrt(float value)
{
    return (float)sqrt((double)value);
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

void SVPWM_Init(foc_motor_t *motor, uint16_t freq_kHz, uint8_t deadtime_percent)
{
    svpwm_interp_state_t *sv = (motor != 0) ? &motor->svpwm : 0;

    FOC_Platform_PWMInit((uint8_t)freq_kHz, deadtime_percent);
    FOC_Platform_PWMSetDutyCycleTripleFloat(0.0f, 0.0f, 0.0f);
    FOC_Platform_PWMStart();

    if (sv != 0)
    {
        sv->interp_steps_total = (freq_kHz > 0U) ? freq_kHz : 1U;
        sv->interp_step_index = sv->interp_steps_total;
    }
}

void SVPWM_SetRuntimeDutyTarget(foc_motor_t *motor,
                                uint8_t sector,
                                float duty_a,
                                float duty_b,
                                float duty_c)
{
    svpwm_interp_state_t *sv;

    if (motor == 0) return;
    sv = &motor->svpwm;

    sv->output.sector = sector;
    sv->output.duty_a = SVPWM_Clamp01(duty_a);
    sv->output.duty_b = SVPWM_Clamp01(duty_b);
    sv->output.duty_c = SVPWM_Clamp01(duty_c);

    sv->duty_a_target = sv->output.duty_a;
    sv->duty_b_target = sv->output.duty_b;
    sv->duty_c_target = sv->output.duty_c;

    sv->duty_a_step = (sv->duty_a_target - sv->duty_a_current) / (float)sv->interp_steps_total;
    sv->duty_b_step = (sv->duty_b_target - sv->duty_b_current) / (float)sv->interp_steps_total;
    sv->duty_c_step = (sv->duty_c_target - sv->duty_c_current) / (float)sv->interp_steps_total;
    sv->interp_step_index = 0U;
}

void SVPWM_ApplyDirectDuty(foc_motor_t *motor,
                           uint8_t sector,
                           float duty_a,
                           float duty_b,
                           float duty_c)
{
    svpwm_interp_state_t *sv;

    if (motor == 0) return;
    sv = &motor->svpwm;

    sv->output.sector = sector;
    sv->output.duty_a = SVPWM_Clamp01(duty_a);
    sv->output.duty_b = SVPWM_Clamp01(duty_b);
    sv->output.duty_c = SVPWM_Clamp01(duty_c);

    sv->duty_a_current = sv->output.duty_a;
    sv->duty_b_current = sv->output.duty_b;
    sv->duty_c_current = sv->output.duty_c;
    sv->duty_a_target  = sv->output.duty_a;
    sv->duty_b_target  = sv->output.duty_b;
    sv->duty_c_target  = sv->output.duty_c;
    sv->duty_a_step = 0.0f;
    sv->duty_b_step = 0.0f;
    sv->duty_c_step = 0.0f;
    sv->interp_step_index = sv->interp_steps_total;

    FOC_Platform_PWMSetDutyCycleTripleFloat(sv->duty_a_current,
                                            sv->duty_b_current,
                                            sv->duty_c_current);
}

void SVPWM_UpdateRuntime(foc_motor_t *motor,
                         float phase_a,
                         float phase_b,
                         float phase_c,
                         float voltage_command,
                         float vbus_voltage)
{
    uint8_t sector;
    float duty_a, duty_b, duty_c;

    if (motor == 0) return;

    SVPWM_CalculateDuty(phase_a, phase_b, phase_c,
                        voltage_command, vbus_voltage,
                        &sector, &duty_a, &duty_b, &duty_c);

    SVPWM_SetRuntimeDutyTarget(motor, sector, duty_a, duty_b, duty_c);
}

void SVPWM_UpdateDirect(foc_motor_t *motor,
                        float phase_a,
                        float phase_b,
                        float phase_c,
                        float voltage_command,
                        float vbus_voltage)
{
    uint8_t sector;
    float duty_a, duty_b, duty_c;

    if (motor == 0) return;

    SVPWM_CalculateDuty(phase_a, phase_b, phase_c,
                        voltage_command, vbus_voltage,
                        &sector, &duty_a, &duty_b, &duty_c);

    SVPWM_ApplyDirectDuty(motor, sector, duty_a, duty_b, duty_c);
}

void SVPWM_InterpolationISR(foc_motor_t *motor)
{
    svpwm_interp_state_t *sv;

    if (motor == 0) return;
    sv = &motor->svpwm;

    if (sv->interp_step_index < sv->interp_steps_total)
    {
        sv->duty_a_current += sv->duty_a_step;
        sv->duty_b_current += sv->duty_b_step;
        sv->duty_c_current += sv->duty_c_step;
        sv->interp_step_index++;

        if (sv->interp_step_index >= sv->interp_steps_total)
        {
            sv->duty_a_current = sv->duty_a_target;
            sv->duty_b_current = sv->duty_b_target;
            sv->duty_c_current = sv->duty_c_target;
        }
    }

    FOC_Platform_PWMSetDutyCycleTripleFloat(sv->duty_a_current, sv->duty_b_current, sv->duty_c_current);
}

const svpwm_output_t* SVPWM_GetOutput(const foc_motor_t *motor)
{
    if (motor == 0) return 0;
    return &motor->svpwm.output;
}