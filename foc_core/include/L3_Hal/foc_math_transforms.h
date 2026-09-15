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
 * 供调用方以 %d.%0Nd 格式输出，避免 %f 引入 double 库（32 位平台非原子 64 位运算）。
 * 注意：|value| < 1 的负值整数部分为 0，%d 无法表达负号，文本化请改用 Math_FormatFixed。 */
void Math_FloatToFixed(float value, uint8_t decimals, int32_t *ipart_out, int32_t *fpart_out);

/* 定点浮点文本化：value → 带符号十进制文本（如 "-0.500"、"12.34"），
 * 符号与幅值分离处理，保证 (-1, 0) 区间的负值不会丢失负号。
 * 返回写入字符数（不含结尾 '\0'）；参数非法或缓冲不足返回 0。 */
uint16_t Math_FormatFixed(char *out, uint16_t max_len, float value, uint8_t decimals);

void Math_ClarkeTransform(float a, float b, float c, float *alpha, float *beta);
void Math_InverseClarkeTransform(float alpha, float beta, float *a, float *b, float *c);
void Math_ParkTransform(float alpha, float beta, float theta, float *d, float *q);
void Math_ParkTransformSC(float alpha, float beta, float sin_theta, float cos_theta,
                          float *d, float *q);
void Math_InverseParkTransform(float d, float q, float theta, float *alpha, float *beta);
void Math_InverseParkTransformSC(float d, float q, float sin_theta, float cos_theta,
                                 float *alpha, float *beta);

#endif /* FOC_MATH_TRANSFORMS_H */
