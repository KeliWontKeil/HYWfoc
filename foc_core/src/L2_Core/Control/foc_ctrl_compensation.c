#include "L2_Core/Control/foc_ctrl_compensation.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)

static inline float Q15ToFloat(int16_t q15, float lsb)
{
    return (float)q15 * lsb;
}

float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s)
{
    uint16_t idx0;
    uint16_t idx1;
    float frac;
    float val0;
    float val1;
    float iq_comp;
    float speed_abs;

    if ((status == 0) || (table_q15 == 0))
    {
        return 0.0f;
    }

    speed_abs = (speed_ref_rad_s >= 0.0f) ? speed_ref_rad_s : -speed_ref_rad_s;
    if (speed_abs > status->speed_gate_rad_s)
    {
        return 0.0f;
    }

    if ((status->available == 0U) || (status->point_count == 0U))
    {
        return 0.0f;
    }

    {
        float step = FOC_MATH_TWO_PI / (float)status->point_count;
        float pos  = mech_angle_rad / step;
        idx0 = (uint16_t)pos;
        frac = pos - (float)idx0;

        if (idx0 >= status->point_count)
        {
            idx0 = (uint16_t)(status->point_count - 1U);
        }

        idx1 = idx0 + 1U;
        if (idx1 >= status->point_count)
        {
            idx1 = 0U;
        }
    }

    val0 = Q15ToFloat(table_q15[idx0], status->iq_lsb_a);
    val1 = Q15ToFloat(table_q15[idx1], status->iq_lsb_a);
    iq_comp = val0 + frac * (val1 - val0);

    return iq_comp;
}

void FOC_ControlApplyCoggingCompensation(foc_motor_t *motor,
                                          float mech_angle_rad,
                                          float speed_ref_rad_s)
{
    float iq_comp;

    if (motor == 0)
    {
        return;
    }

    if ((motor->cogging_comp_status.enabled == 0U) ||
        (motor->cogging_comp_status.available == 0U))
    {
        return;
    }

    iq_comp = FOC_ControlCoggingLookupIq(&motor->cogging_comp_status,
                                          motor->cogging_comp_table_q15,
                                          mech_angle_rad,
                                          speed_ref_rad_s);

    motor->iq_target += iq_comp * motor->cogging_comp_status.calib_gain_k;
}

uint8_t FOC_ControlLoadCoggingCompTableQ15(foc_motor_t *motor,
                                            const int16_t *table_q15,
                                            uint16_t point_count,
                                            float iq_lsb_a,
                                            uint8_t source)
{
    if ((motor == 0) || (table_q15 == 0))
    {
        return 0U;
    }

    if (point_count > FOC_COGGING_LUT_POINT_COUNT)
    {
        point_count = FOC_COGGING_LUT_POINT_COUNT;
    }

    (void)memcpy((void *)motor->cogging_comp_table_q15,
                 (const void *)table_q15,
                 (size_t)point_count * sizeof(int16_t));

    motor->cogging_comp_status.point_count = point_count;
    motor->cogging_comp_status.iq_lsb_a    = iq_lsb_a;
    motor->cogging_comp_status.source      = source;
    motor->cogging_comp_status.available   = 1U;

    return 1U;
}

void FOC_ControlSetCoggingCompUnavailable(foc_motor_t *motor, uint8_t source)
{
    if (motor == 0)
    {
        return;
    }
    motor->cogging_comp_status.available = 0U;
    motor->cogging_comp_status.source    = source;
}

#endif /* FOC_COGGING_COMP_ENABLE */