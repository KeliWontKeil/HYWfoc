#include "L2_Core/Control/foc_ctrl_startup.h"

#include <math.h>

#include "L2_Core/Control/foc_ctrl_estim.h"
#include "L2_Core/Control/foc_ctrl_startup_openloop.h"
#include "LS_Config/foc_config.h"

#if (FOC_STARTUP_OPENLOOP_ENABLE == FOC_CFG_ENABLE)

static void FOC_StartupOpenLoop_InitParams(foc_motor_t *motor)
{
    motor->startup_openloop_state.phase              = FOC_STARTUP_PHASE_RUNNING;
    motor->startup_openloop_state.virtual_angle_rad   = 0.0f;
    motor->startup_openloop_state.virtual_speed_rad_s = 0.0f;
    motor->startup_openloop_state.current_ref_a       = FOC_STARTUP_OPENLOOP_CURRENT_A;
    motor->startup_openloop_state.ramp_rate_rad_s2    = FOC_STARTUP_OPENLOOP_RAMP_RATE_RAD_S2;
    motor->startup_openloop_state.target_speed_rad_s  = FOC_STARTUP_OPENLOOP_TARGET_SPEED_RAD_S;
}

void FOC_Startup_Init(foc_motor_t *motor)
{
    if (motor == 0) return;
    FOC_StartupOpenLoop_InitParams(motor);
    motor->state.current_loop_ready = 1U;
}

void FOC_Startup_RunStep(foc_motor_t *motor, float dt_sec)
{
    if (motor == 0) return;
    FOC_StartupOpenLoop_RunStep(motor, dt_sec);
}

uint8_t FOC_Startup_IsComplete(const foc_motor_t *motor)
{
    if (motor == 0) return 0U;
    return (motor->startup_openloop_state.phase == FOC_STARTUP_PHASE_DONE) ? 1U : 0U;
}

void FOC_Startup_Abort(foc_motor_t *motor)
{
    if (motor == 0) return;
    motor->startup_openloop_state.phase = FOC_STARTUP_PHASE_FAILED;
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
