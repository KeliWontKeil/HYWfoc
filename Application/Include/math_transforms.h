#ifndef _MATH_TRANSFORMS_H_
#define _MATH_TRANSFORMS_H_

#include <stdint.h>

typedef struct {
    float a;
    float b;
    float c;
} abc_frame_t;

typedef struct {
    float alpha;
    float beta;
} alpha_beta_frame_t;

typedef struct {
    float d;
    float q;
} dq_frame_t;

void Math_ClarkeTransform(const abc_frame_t *abc, alpha_beta_frame_t *alpha_beta);
void Math_InverseClarkeTransform(const alpha_beta_frame_t *alpha_beta, abc_frame_t *abc);
void Math_ParkTransform(const alpha_beta_frame_t *alpha_beta, float sin_theta, float cos_theta, dq_frame_t *dq);
void Math_InverseParkTransform(const dq_frame_t *dq, float sin_theta, float cos_theta, alpha_beta_frame_t *alpha_beta);

#endif /* _MATH_TRANSFORMS_H_ */
