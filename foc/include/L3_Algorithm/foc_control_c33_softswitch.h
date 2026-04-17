#ifndef FOC_CONTROL_C33_SOFTSWITCH_H
#define FOC_CONTROL_C33_SOFTSWITCH_H

#include <stdint.h>

float FOC_ControlSoftSwitchUpdateBlend(float current_blend,
                                       uint8_t *blend_initialized,
                                       float target_blend,
                                       float dt_sec);

#endif /* FOC_CONTROL_C33_SOFTSWITCH_H */
