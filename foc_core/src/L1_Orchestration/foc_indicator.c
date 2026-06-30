#include "L1_Orchestration/foc_indicator.h"

#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void FOC_Indicator_Update(foc_motor_t *motor, foc_runtime_ctx_t *runtime)
{
    uint8_t led_run_on = 0U;
    uint8_t led_fault_on = 0U;

    if (motor->state.system_fault != 0U)
    {
        led_fault_on = 1U;
        runtime->indicator.led_run_blink_counter = 0U;
    }
    else if (motor->state.system_running != 0U)
    {
        runtime->indicator.led_run_blink_counter++;
        if (runtime->indicator.led_run_blink_counter >= (2U * FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS))
        {
            runtime->indicator.led_run_blink_counter = 0U;
        }
        led_run_on = (runtime->indicator.led_run_blink_counter < FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS) ? 1U : 0U;
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_FAULT_INDEX, led_fault_on);

    if (runtime->indicator.comm_pulse_counter > 0U)
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 1U);
        runtime->indicator.comm_pulse_counter--;
    }
    else
    {
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, 0U);
    }
}