#include "L2_Core/Control/foc_ctrl_transition.h"

#if (FOC_TRANSITION_ENABLE == FOC_CFG_ENABLE)

void FOC_Transition_Init(foc_motor_t *motor)
{
    (void)motor;
}

void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec)
{
    (void)motor;
    (void)dt_sec;
}

uint8_t FOC_Transition_Request(foc_motor_t *motor, uint8_t target_estimator_type)
{
    (void)motor;
    (void)target_estimator_type;
    return 0U;
}

#else

void FOC_Transition_Init(foc_motor_t *motor)
{
    (void)motor;
}

void FOC_Transition_RunStep(foc_motor_t *motor, float dt_sec)
{
    (void)motor;
    (void)dt_sec;
}

uint8_t FOC_Transition_Request(foc_motor_t *motor, uint8_t target_estimator_type)
{
    (void)motor;
    (void)target_estimator_type;
    return 0U;
}

#endif