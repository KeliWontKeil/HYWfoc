#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_math_lut.h"

float Math_WrapRad(float angle)
{
    while (angle >= FOC_MATH_TWO_PI)
    {
        angle -= FOC_MATH_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += FOC_MATH_TWO_PI;
    }
    return angle;
}

float Math_WrapRadDelta(float angle)
{
    while (angle > FOC_MATH_PI)
    {
        angle -= FOC_MATH_TWO_PI;
    }
    while (angle < -FOC_MATH_PI)
    {
        angle += FOC_MATH_TWO_PI;
    }
    return angle;
}

float Math_WrapNearest(float reference, float target)
{
    float two_pi = FOC_MATH_TWO_PI;
    float offset = target - reference;
    float cycles = floorf((offset + FOC_MATH_PI) / two_pi);
    return target - cycles * two_pi;
}

float Math_ClampFloat(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }
    if (value > max_val)
    {
        return max_val;
    }
    return value;
}

float Math_NormalizeDt(float dt_sec, float fallback_dt_sec)
{
    return (dt_sec > 0.0f) ? dt_sec : fallback_dt_sec;
}

void Math_FloatToFixed(float value, uint8_t decimals, int32_t *ipart_out, int32_t *fpart_out)
{
    float scale = 1.0f;
    uint8_t i;
    int32_t v;

    if ((ipart_out == 0) || (fpart_out == 0))
    {
        return;
    }

    /* 防御：限制输入规模，避免浮点放大后超出 int32 范围 */
    if (value > 20000.0f)
    {
        value = 20000.0f;
    }
    else if (value < -20000.0f)
    {
        value = -20000.0f;
    }

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10.0f;
    }
    if (scale < 1.0f)
    {
        scale = 1.0f;
    }

    if (value < 0.0f)
    {
        v = (int32_t)(value * scale - 0.5f);
    }
    else
    {
        v = (int32_t)(value * scale + 0.5f);
    }

    *ipart_out = v / (int32_t)scale;
    *fpart_out = v % (int32_t)scale;
    if (*fpart_out < 0)
    {
        *fpart_out = -*fpart_out;
    }
}

float Math_FirstOrderLpf(float input, float *state, float alpha, uint8_t *state_valid)
{
    float alpha_clamped;

    if ((state == 0) || (state_valid == 0))
    {
        return input;
    }

    alpha_clamped = Math_ClampFloat(alpha, 0.0f, 1.0f);

    if (*state_valid == 0U)
    {
        *state = input;
        *state_valid = 1U;
        return input;
    }

    *state += alpha_clamped * (input - *state);
    return *state;
}

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta = (b - c) * FOC_MATH_INV_SQRT3;
}

void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + FOC_MATH_SQRT3_BY_2 * beta;
    *c = -0.5f * alpha - FOC_MATH_SQRT3_BY_2 * beta;
}

void Math_ParkTransform(float alpha, float beta, float theta, float *d, float *q)
{
    float sin_theta;
    float cos_theta;

    FOC_MathLut_SinCos(theta, &sin_theta, &cos_theta);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

void Math_ParkTransformSC(float alpha, float beta, float sin_theta, float cos_theta,
                          float *d, float *q)
{
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta)
{
    float sin_theta;
    float cos_theta;

    FOC_MathLut_SinCos(theta, &sin_theta, &cos_theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}

void Math_InverseParkTransformSC(float d, float q, float sin_theta, float cos_theta,
                                 float *alpha, float *beta)
{
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}
