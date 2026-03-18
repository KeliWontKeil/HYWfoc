#include "svpwm.h"
#include <math.h>

#define SVPWM_SQRT3          1.7320508f
#define SVPWM_SQRT3_BY_2     0.8660254f
#define SVPWM_EPSILON        1e-6f
#define SVPWM_PI             3.1415926f
#define SVPWM_TWO_PI         (2.0f * SVPWM_PI)
#define SVPWM_PI_BY_3        (SVPWM_PI / 3.0f)

static float s_vbus = 12.0f;
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

void SVPWM_Init(float vbus_voltage)
{
    SVPWM_SetBusVoltage(vbus_voltage);

    s_output.sector = 0;
    s_output.duty_a = 0.5f;
    s_output.duty_b = 0.5f;
    s_output.duty_c = 0.5f;
}

void SVPWM_SetBusVoltage(float vbus_voltage)
{
    if (vbus_voltage < SVPWM_EPSILON)
    {
        s_vbus = 12.0f;
        return;
    }

    s_vbus = vbus_voltage;
}

void SVPWM_Update(const svpwm_input_t *input)
{
    float alpha;
    float beta;
    float magnitude;
    float theta;
    float theta_sector;
    float t_sum;
    float t1;
    float t2;
    float t0;
    uint8_t sector;

    if (input == 0)
    {
        return;
    }

    /* Use normalized alpha-beta vector for pure six-sector SVPWM timing. */
    alpha = input->alpha;
    beta = input->beta;
    magnitude = sqrtf(alpha * alpha + beta * beta);

    if (magnitude > 1.0f)
    {
        alpha /= magnitude;
        beta /= magnitude;
        magnitude = 1.0f;
    }

    if (magnitude < SVPWM_EPSILON)
    {
        s_output.sector = 0;
        s_output.duty_a = 0.5f;
        s_output.duty_b = 0.5f;
        s_output.duty_c = 0.5f;
        return;
    }

    theta = atan2f(beta, alpha);
    if (theta < 0.0f)
    {
        theta += SVPWM_TWO_PI;
    }

    sector = SVPWM_DetermineSector(alpha, beta);
    if (sector == 0U)
    {
        sector = (uint8_t)(theta / SVPWM_PI_BY_3) + 1U;
        if (sector > 6U)
        {
            sector = 6U;
        }
    }

    theta_sector = theta - (float)(sector - 1U) * SVPWM_PI_BY_3;

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

    switch (sector)
    {
        case 1U:
            s_output.duty_a = t1 + t2 + 0.5f * t0;
            s_output.duty_b = t2 + 0.5f * t0;
            s_output.duty_c = 0.5f * t0;
            break;
        case 2U:
            s_output.duty_a = t1 + 0.5f * t0;
            s_output.duty_b = t1 + t2 + 0.5f * t0;
            s_output.duty_c = 0.5f * t0;
            break;
        case 3U:
            s_output.duty_a = 0.5f * t0;
            s_output.duty_b = t1 + t2 + 0.5f * t0;
            s_output.duty_c = t2 + 0.5f * t0;
            break;
        case 4U:
            s_output.duty_a = 0.5f * t0;
            s_output.duty_b = t1 + 0.5f * t0;
            s_output.duty_c = t1 + t2 + 0.5f * t0;
            break;
        case 5U:
            s_output.duty_a = t2 + 0.5f * t0;
            s_output.duty_b = 0.5f * t0;
            s_output.duty_c = t1 + t2 + 0.5f * t0;
            break;
        case 6U:
            s_output.duty_a = t1 + t2 + 0.5f * t0;
            s_output.duty_b = 0.5f * t0;
            s_output.duty_c = t1 + 0.5f * t0;
            break;
        default:
            s_output.duty_a = 0.5f;
            s_output.duty_b = 0.5f;
            s_output.duty_c = 0.5f;
            break;
    }

    float duty = input ->set_voltage / s_vbus;

    s_output.duty_a = duty * SVPWM_Clamp01(s_output.duty_a);
    s_output.duty_b = duty * SVPWM_Clamp01(s_output.duty_b);
    s_output.duty_c = duty * SVPWM_Clamp01(s_output.duty_c);

    s_output.sector = sector;
}

const svpwm_output_t* SVPWM_GetOutput(void)
{
    return &s_output;
}
