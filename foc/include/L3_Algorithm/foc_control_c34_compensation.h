#ifndef FOC_CONTROL_C34_COMPENSATION_H
#define FOC_CONTROL_C34_COMPENSATION_H

#include "L3_Algorithm/foc_control_c21_cfg_state.h"

float FOC_ControlCoggingLookupIq(const foc_cogging_comp_status_t *status,
                                 const int16_t *table_q15,
                                 float mech_angle_rad,
                                 float speed_ref_rad_s);

#endif /* FOC_CONTROL_C34_COMPENSATION_H */
