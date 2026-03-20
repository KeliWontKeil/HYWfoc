#include "svpwm.h"
#include <math.h>

#define SVPWM_SQRT3          1.7320508f
#define SVPWM_SQRT3_BY_2     0.8660254f
#define SVPWM_EPSILON        1e-6f
#define SVPWM_PI             3.1415926f
#define SVPWM_TWO_PI         (2.0f * SVPWM_PI)
#define SVPWM_PI_BY_3        (SVPWM_PI / 3.0f)

static svpwm_output_t s_output;

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
    float u2 = SVPWM_SQRT3_BY_2 * alpha - 0.5f * beta;
    float u3 = -SVPWM_SQRT3_BY_2 * alpha - 0.5f * beta;
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

void SVPWM_Init(uint16_t freq_kHz,uint8_t deadtime_percent)
{
    /* Initialize PWM (TIMER0 as slave) */
    PWM_Init(freq_kHz, deadtime_percent);
    PWM_SetDutyCycle(PWM_CHANNEL_0, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_1, 0);
    PWM_SetDutyCycle(PWM_CHANNEL_2, 0);
    PWM_Start();

    s_output.sector = 0;
    s_output.duty_a = 0.5f;
    s_output.duty_b = 0.5f;
    s_output.duty_c = 0.5f;
}

void SVPWM_Update(float phase_a,
                  float phase_b,
                  float phase_c,
                  float set_voltage,
                  float vbus_voltage,
                  uint8_t *sector_out,
                  float *duty_a,
                  float *duty_b,
                  float *duty_c)
{
    float alpha;
    float beta;
    float voltage_ratio;
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

    voltage_ratio = set_voltage / vbus_voltage;
    if (voltage_ratio < 0.0f)
    {
        voltage_ratio = -voltage_ratio;
    }
    if (voltage_ratio > 1.0f)
    {
        voltage_ratio = 1.0f;
    }

    phase_a *= voltage_ratio;
    phase_b *= voltage_ratio;
    phase_c *= voltage_ratio;

    /* Reconstruct alpha-beta from inverse Clarke output (three-phase voltages). */
    alpha = phase_a;
    beta = (phase_b - phase_c) / SVPWM_SQRT3;
    magnitude = sqrtf(alpha * alpha + beta * beta);

    if (magnitude > 1.0f)
    {
        alpha /= magnitude;
        beta /= magnitude;
        magnitude = 1.0f;
    }

    if (magnitude < SVPWM_EPSILON)
    {
        *sector_out = 0U;
        *duty_a = 0.5f;
        *duty_b = 0.5f;
        *duty_c = 0.5f;

        s_output.sector = *sector_out;
        s_output.duty_a = *duty_a;
        s_output.duty_b = *duty_b;
        s_output.duty_c = *duty_c;
        return;
    }

    theta = atan2f(beta, alpha);
    if (theta < 0.0f)
    {
        theta += SVPWM_TWO_PI;
    }

    sector_id = SVPWM_DetermineSector(alpha, beta);
    if (sector_id == 0U)
    {
        sector_id = (uint8_t)(theta / SVPWM_PI_BY_3) + 1U;
        if (sector_id > 6U)
        {
            sector_id = 6U;
        }
    }

    theta_sector = theta - (float)(sector_id - 1U) * SVPWM_PI_BY_3;

    t1 = magnitude * sinf(SVPWM_PI_BY_3 - theta_sector);
    t2 = magnitude * sinf(theta_sector);

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

    s_output.sector = *sector_out;
    s_output.duty_a = *duty_a;
    s_output.duty_b = *duty_b;
    s_output.duty_c = *duty_c;
}

const svpwm_output_t* SVPWM_GetOutput(void)
{
    return &s_output;
}
