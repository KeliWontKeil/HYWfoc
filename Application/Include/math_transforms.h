#ifndef _MATH_TRANSFORMS_H_
#define _MATH_TRANSFORMS_H_

#include <math.h>

#define MATH_PI 3.1415926f
#define MATH_TWO_PI 6.2831852f
#define MATH_SQRT3_BY_2 0.8660254f

float Math_WrapRad(float angle);
float Math_WrapRadDelta(float angle);
float Math_ClampFloat(float value, float min_val, float max_val);

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta);
void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c);
void Math_ParkTransform(float alpha, float beta, float theta, float *d, float *q);
void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta);

#endif /* _MATH_TRANSFORMS_H_ */
