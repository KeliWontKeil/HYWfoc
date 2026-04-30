#ifndef FOC_SHARED_TYPES_H
#define FOC_SHARED_TYPES_H

/*
 * =====================================================================
 *  Unified entry for all shared type definitions.
 *
 *  Types are split into three category-specific files for clarity:
 *    - foc_math_types.h      — kalman_filter_t, foc_pid_t
 *    - foc_motor_types.h     — motor aggregate struct, sensor, cogging, soft-switch
 *    - foc_scheduler_types.h — FOC_TaskRate_t scheduling enum
 *
 *  This wrapper includes all three; existing code that includes
 *  "LS_Config/foc_shared_types.h" continues to work without changes.
 *
 *  Direct includes of the sub-files are also permitted for callers
 *  that need only a specific category.
 * =====================================================================
 */

#include "LS_Config/foc_math_types.h"
#include "LS_Config/foc_motor_types.h"
#include "LS_Config/foc_scheduler_types.h"

#endif /* FOC_SHARED_TYPES_H */
