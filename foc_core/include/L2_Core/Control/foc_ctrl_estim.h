#ifndef FOC_CTRL_ESTIM_H
#define FOC_CTRL_ESTIM_H

#include <stdint.h>
#include "L2_Core/foc_ctrl_types.h"

/* Estimator states share the Source state value space. */
#define FOC_ESTIMATOR_STATE_INIT        FOC_SOURCE_STATE_INIT
#define FOC_ESTIMATOR_STATE_CONVERGING  FOC_SOURCE_STATE_CONVERGING
#define FOC_ESTIMATOR_STATE_LOCKED      FOC_SOURCE_STATE_LOCKED
#define FOC_ESTIMATOR_STATE_DIVERGED    FOC_SOURCE_STATE_DIVERGED

#endif /* FOC_CTRL_ESTIM_H */
