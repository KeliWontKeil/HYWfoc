#include "L3_Algorithm/foc_control_c34_compensation.h"

#include <math.h>

#include "LS_Config/foc_config.h"
#include "L41_Math/math_transforms.h"

float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s)
{
    float angle_rad;
    float index_f;
    uint16_t index_0;
    uint16_t index_1;
    float frac;
    float iq_0;
    float iq_1;
    float iq_comp;
    float speed_abs;

    if ((status == 0) || (table_q15 == 0))
    {
        return 0.0f;
    }

    if ((status->enabled == 0U) ||
        (status->available == 0U) ||
        (status->point_count < 2U))
    {
        return 0.0f;
    }

    angle_rad = Math_WrapRad(mech_angle_rad);
    index_f = angle_rad * ((float)status->point_count / FOC_MATH_TWO_PI);
    index_0 = (uint16_t)index_f;
    if (index_0 >= status->point_count)
    {
        index_0 = (uint16_t)(status->point_count - 1U);
    }
    index_1 = (uint16_t)((index_0 + 1U) % status->point_count);
    frac = index_f - (float)index_0;

    iq_0 = (float)table_q15[index_0] * status->iq_lsb_a;
    iq_1 = (float)table_q15[index_1] * status->iq_lsb_a;
    iq_comp = iq_0 + (iq_1 - iq_0) * frac;

    speed_abs = fabsf(speed_ref_rad_s);
    if (status->speed_gate_rad_s > 1e-6f)
    {
        if (speed_abs >= status->speed_gate_rad_s)
        {
            return 0.0f;
        }

        iq_comp *= (1.0f - (speed_abs / status->speed_gate_rad_s));
    }

    return Math_ClampFloat(iq_comp,
                           -status->iq_limit_a,
                           status->iq_limit_a);
}
