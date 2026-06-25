#ifndef FOC_SERVICE_HANDLER_H
#define FOC_SERVICE_HANDLER_H

#include "L2_Core/foc_ctrl_types.h"
#include "L1_Orchestration/foc_system_types.h"

/*
 * L1 service handler: orchestrates initialisation, re-initialisation,
 * and the periodic service task (protocol, config commit, system commands).
 */

/* Initialise motor hardware: calls FOC_MotorInit + calibrate + apply config */
void FOC_Service_InitMotor(foc_motor_t *motor);

/* Re-initialise motor: stop current loop → open-loop zero → reinit */
void FOC_Service_ReInitMotor(foc_motor_t *motor);

/* Run one service task cycle: reinit check → protocol → cfg dirty → system actions.
 * Returns 1 if there was communication activity (frame processed), 0 otherwise. */
uint8_t FOC_Service_Process(foc_motor_t *motor);

/* Run the initial init_check mask verification after init.
 * Sets system_running / system_fault accordingly.
 * Should be called once after FOC_Service_InitMotor. */
void FOC_Service_VerifyInitChecks(foc_motor_t *motor, const sensor_data_t *sensor);

/* Check cfg_dirty flag and apply + commit configuration. */
void FOC_Service_ApplyCfgDirty(foc_motor_t *motor);

/* Update system state based on control cycle result (FOC_CYCLE_OK / FAULT_*). */
void FOC_Service_HandleControlResult(foc_motor_t *motor, uint8_t cycle_result);

/* Update LED indicators based on motor state and runtime indicator state. */
void FOC_Service_UpdateIndicators(foc_motor_t *motor, foc_runtime_ctx_t *runtime);

#endif /* FOC_SERVICE_HANDLER_H */