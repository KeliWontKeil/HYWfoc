#include "L3_Algorithm/foc_control_c33_softswitch.h"

#include "LS_Config/foc_config.h"
#include "L41_Math/math_transforms.h"

float FOC_ControlSoftSwitchUpdateBlend(float current_blend,
                                       uint8_t *blend_initialized,
                                       float target_blend,
                                       float dt_sec)
{
    float alpha;

    target_blend = Math_ClampFloat(target_blend, 0.0f, 1.0f);
    if ((blend_initialized != 0U) && (*blend_initialized == 0U))
    {
        if (blend_initialized != 0U)
        {
            *blend_initialized = 1U;
        }
        return target_blend;
    }

    alpha = dt_sec / (FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_DEFAULT_SEC + dt_sec);
    alpha = Math_ClampFloat(alpha, 0.0f, 1.0f);

    current_blend += (target_blend - current_blend) * alpha;
    return Math_ClampFloat(current_blend, 0.0f, 1.0f);
}
