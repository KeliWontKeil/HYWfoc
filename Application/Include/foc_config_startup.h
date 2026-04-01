#ifndef FOC_CONFIG_STARTUP_H
#define FOC_CONFIG_STARTUP_H

#include "foc_shared_types.h"

/* Runtime parameter defaults. */
#define COMMAND_MANAGER_DEFAULT_SEMANTIC_ENABLED 0U
#define COMMAND_MANAGER_DEFAULT_OSC_ENABLED 1U


#define COMMAND_MANAGER_DEFAULT_SEMANTIC_FREQ_HZ 2U
#define COMMAND_MANAGER_DEFAULT_OSC_FREQ_HZ 100U


/* Telemetry and diagnostic output configuration. */
#define DEBUG_STREAM_ENABLE_SEMANTIC_REPORT 1U
#define DEBUG_STREAM_ENABLE_OSC_REPORT 1U


/* Diagnostic output and statistics trimming switches. */
#define FOC_FEATURE_DIAG_OUTPUT 1U
#define FOC_FEATURE_DIAG_STATS 1U


/* Undervoltage protection feature switch (trim-able). */
#define FOC_FEATURE_UNDERVOLTAGE_PROTECTION 0U

#define FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT 9.0f
#define FOC_UNDERVOLTAGE_RECOVER_VBUS_DEFAULT 9.5f


/*
 * Control-algorithm build set selection.
 * Note: SPEED_ONLY and SPEED_ANGLE are parallel algorithms, not feature superset/subset.
 * FULL keeps both algorithms and allows runtime mode switching.
 */
#define FOC_BUILD_CONTROL_ALGO_SET FOC_CTRL_ALGO_BUILD_FULL

#define FOC_CTRL_ALGO_BUILD_FULL 0U
#define FOC_CTRL_ALGO_BUILD_SPEED_ONLY 1U
#define FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY 2U
#if ((FOC_FEATURE_DIAG_OUTPUT != 0U) && (FOC_FEATURE_DIAG_OUTPUT != 1U))
#error "FOC_FEATURE_DIAG_OUTPUT must be 0 or 1"
#endif

#if ((FOC_FEATURE_DIAG_STATS != 0U) && (FOC_FEATURE_DIAG_STATS != 1U))
#error "FOC_FEATURE_DIAG_STATS must be 0 or 1"
#endif



#if ((FOC_BUILD_CONTROL_ALGO_SET != FOC_CTRL_ALGO_BUILD_FULL) && \
	(FOC_BUILD_CONTROL_ALGO_SET != FOC_CTRL_ALGO_BUILD_SPEED_ONLY) && \
	(FOC_BUILD_CONTROL_ALGO_SET != FOC_CTRL_ALGO_BUILD_SPEED_ANGLE_ONLY))
#error "FOC_BUILD_CONTROL_ALGO_SET must be a valid FOC_CTRL_ALGO_BUILD_* value"
#endif

/* Default motor init parameters for board bring-up and calibration. */
#define FOC_APP_MOTOR_INIT_VBUS_DEFAULT 12.0f
#define FOC_APP_MOTOR_INIT_SET_VOLTAGE_DEFAULT 11.4f
#define FOC_APP_MOTOR_INIT_PHASE_RES_DEFAULT 13.2f
#define FOC_APP_MOTOR_INIT_POLE_PAIRS_DEFAULT 7U
#define FOC_APP_MOTOR_INIT_MECH_ZERO_DEFAULT_RAD 3.157f
#define FOC_APP_MOTOR_INIT_DIRECTION_DEFAULT FOC_DIR_REVERSED

/* Default SVPWM deadtime percent used by app init. */
#define FOC_APP_SVPWM_DEADTIME_PERCENT_DEFAULT 2U


/* LED behavior configuration (indices are mapped in Utilities/LED). */
#define FOC_LED_RUN_INDEX 1U
#define FOC_LED_COMM_INDEX 2U
#define FOC_LED_FAULT_INDEX 3U
#define FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS 50U
#define FOC_LED_COMM_PULSE_TICKS 10U

#endif /* FOC_CONFIG_STARTUP_H */
