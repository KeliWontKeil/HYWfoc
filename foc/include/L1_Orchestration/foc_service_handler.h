#ifndef FOC_SERVICE_HANDLER_H
#define FOC_SERVICE_HANDLER_H

#include "LS_Config/foc_motor_types.h"

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

#endif /* FOC_SERVICE_HANDLER_H */
