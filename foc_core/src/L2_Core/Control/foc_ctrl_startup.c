#include "L2_Core/Control/foc_ctrl_startup.h"

#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)

void FOC_Startup_Init(foc_motor_t *motor)
{
    if (motor == 0) return;
    motor->startup_openloop_state.phase = FOC_STARTUP_PHASE_IDLE;
}

void FOC_Startup_RunStep(foc_motor_t *motor, float dt_sec)
{
    (void)motor;
    (void)dt_sec;
}

uint8_t FOC_Startup_IsComplete(const foc_motor_t *motor)
{
    (void)motor;
    return 0U;
}

void FOC_Startup_Abort(foc_motor_t *motor)
{
    (void)motor;
}

#else

void FOC_Startup_Init(foc_motor_t *motor)
{
    (void)motor;
}

void FOC_Startup_RunStep(foc_motor_t *motor, float dt_sec)
{
    (void)motor;
    (void)dt_sec;
}

uint8_t FOC_Startup_IsComplete(const foc_motor_t *motor)
{
    (void)motor;
    return 0U;
}

void FOC_Startup_Abort(foc_motor_t *motor)
{
    (void)motor;
}

#endif