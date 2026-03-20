#include "math_transforms.h"

#define MATH_SQRT3_BY_2 0.8660254f

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta = MATH_SQRT3_BY_2 * (b - c);
}

void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5f * alpha + MATH_SQRT3_BY_2 * beta;
    *c = -0.5f * alpha - MATH_SQRT3_BY_2 * beta;
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
