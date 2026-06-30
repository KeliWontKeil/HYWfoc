#include "L1_Orchestration/foc_init_check.h"

#include "L2_Core/Runtime/foc_runtime_types.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void FOC_InitCheck_Verify(foc_motor_t *motor, const sensor_data_t *sensor)
{
    uint16_t missing;

    if ((motor == 0) || (sensor == 0)) return;

    motor->state.init_check_mask = RUNTIME_INIT_CHECK_COMMAND |
                                   RUNTIME_INIT_CHECK_COMM |
                                   RUNTIME_INIT_CHECK_PROTOCOL |
                                   RUNTIME_INIT_CHECK_DEBUG |
                                   RUNTIME_INIT_CHECK_PWM;

    if ((sensor->adc_valid != 0U) && (sensor->encoder_valid != 0U))
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_SENSOR;
    }

    if ((motor->direction != FOC_DIR_UNDEFINED) &&
        (motor->mech_angle_at_elec_zero_rad != FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_MOTOR;
    }

#if (FOC_FEATURE_UNDERVOLTAGE_PROTECTION == FOC_CFG_ENABLE)
    if (sensor->vbus_voltage_filtered > FOC_UNDERVOLTAGE_TRIP_VBUS_DEFAULT)
    {
        motor->state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
    else
    {
        motor->state.init_fail_mask |= RUNTIME_INIT_CHECK_VBUS;
    }
#else
    motor->state.init_check_mask |= RUNTIME_INIT_CHECK_VBUS;
#endif

    missing = (uint16_t)(RUNTIME_INIT_CHECK_VBUS |
               RUNTIME_INIT_CHECK_MOTOR |
               RUNTIME_INIT_CHECK_PWM |
               RUNTIME_INIT_CHECK_SENSOR |
               RUNTIME_INIT_CHECK_DEBUG |
               RUNTIME_INIT_CHECK_COMMAND |
               RUNTIME_INIT_CHECK_PROTOCOL |
               RUNTIME_INIT_CHECK_COMM) & (~motor->state.init_check_mask);

    if ((motor->state.init_check_mask != 0U) &&
        (motor->state.init_fail_mask == 0U) &&
        (missing == 0U))
    {
        motor->state.system_running = 1U;
        motor->state.system_fault = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;
        FOC_Platform_WriteDebugText("init: all checks passed\r\n");
    }
    else
    {
        motor->state.system_running = 0U;
        motor->state.system_fault = 1U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_INIT_FAILED;
        FOC_Platform_WriteDebugText("init: checks failed or missing\r\n");
    }
}