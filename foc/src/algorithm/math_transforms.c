#include "algorithm/math_transforms.h"

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
    *beta = FOC_MATH_SQRT3_BY_2 * (b - c);
}

void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + FOC_MATH_SQRT3_BY_2 * beta;
    *c = -0.5f * alpha - FOC_MATH_SQRT3_BY_2 * beta;
}

void Math_ParkTransform(float alpha, float beta, float theta,float *d, float *q)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    *d = alpha * cos_theta + beta * sin_theta;
    *q = -alpha * sin_theta + beta * cos_theta;
}

void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    *alpha = d * cos_theta - q * sin_theta;
    *beta = d * sin_theta + q * cos_theta;
}
