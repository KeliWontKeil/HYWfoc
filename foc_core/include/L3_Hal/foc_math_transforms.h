#ifndef FOC_MATH_TRANSFORMS_H
#define FOC_MATH_TRANSFORMS_H

#include <math.h>
#include <stdint.h>

float Math_WrapRad(float angle);
float Math_WrapRadDelta(float angle);
float Math_WrapNearest(float reference, float target);
float Math_ClampFloat(float value, float min_val, float max_val);
float Math_FirstOrderLpf(float input, float *state, float alpha, uint8_t *state_valid);
float Math_NormalizeDt(float dt_sec, float fallback_dt_sec);

/* 定点浮点分解：value → 带符号整数部分 + 无符号小数部分（decimals 位，按精度补零），
 * 供调用方以 %d.%0Nd 格式输出，避免 %f 引入 double 库（32 位平台非原子 64 位运算）。 */
void Math_FloatToFixed(float value, uint8_t decimals, int32_t *ipart_out, int32_t *fpart_out);

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta);
void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c);
void Math_ParkTransform(float alpha, float beta, float theta, float *d, float *q);
void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta);

#endif /* FOC_MATH_TRANSFORMS_H */
