#include "math_transforms.h"

#define MATH_SQRT3_BY_2 0.8660254f

void Math_ClarkeTransform(const abc_frame_t *abc, alpha_beta_frame_t *alpha_beta)
{
    if ((abc == 0) || (alpha_beta == 0))
    {
        return;
    }

    alpha_beta->alpha = abc->a;
    alpha_beta->beta = MATH_SQRT3_BY_2 * (abc->b - abc->c);
}

void Math_InverseClarkeTransform(const alpha_beta_frame_t *alpha_beta, abc_frame_t *abc)
{
    if ((alpha_beta == 0) || (abc == 0))
    {
        return;
    }

    abc->a = alpha_beta->alpha;
    abc->b = -0.5f * alpha_beta->alpha + MATH_SQRT3_BY_2 * alpha_beta->beta;
    abc->c = -0.5f * alpha_beta->alpha - MATH_SQRT3_BY_2 * alpha_beta->beta;
}

void Math_ParkTransform(const alpha_beta_frame_t *alpha_beta, float sin_theta, float cos_theta, dq_frame_t *dq)
{
    if ((alpha_beta == 0) || (dq == 0))
    {
        return;
    }

    dq->d = alpha_beta->alpha * cos_theta + alpha_beta->beta * sin_theta;
    dq->q = -alpha_beta->alpha * sin_theta + alpha_beta->beta * cos_theta;
}

void Math_InverseParkTransform(const dq_frame_t *dq, float sin_theta, float cos_theta, alpha_beta_frame_t *alpha_beta)
{
    if ((dq == 0) || (alpha_beta == 0))
    {
        return;
    }

    alpha_beta->alpha = dq->d * cos_theta - dq->q * sin_theta;
    alpha_beta->beta = dq->d * sin_theta + dq->q * cos_theta;
}
