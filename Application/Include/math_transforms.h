#ifndef _MATH_TRANSFORMS_H_
#define _MATH_TRANSFORMS_H_

#include "gd32f30x.h"
#include <math.h>

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta);
void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c);
void Math_ParkTransform(float alpha, float beta, float theta, float *d, float *q);
void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta);

#endif /* _MATH_TRANSFORMS_H_ */
