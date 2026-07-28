#include "L1_Orchestration/foc_indicator.h"

#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

void FOC_Indicator_Update(foc_motor_t *motor, foc_runtime_ctx_t *runtime)
{
    uint8_t led_run_on = 0U;
    uint8_t led_error_on = 0U;

    if (motor->state.system_fault != 0U)
    {
        led_error_on = 1U;
        runtime->indicator.run_blink_counter = 0U;
    }
    else if (motor->state.system_running != 0U)
    {
        runtime->indicator.run_blink_counter++;
        if (runtime->indicator.run_blink_counter >= (2U * FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS))
        {
            runtime->indicator.run_blink_counter = 0U;
        }
        uint8_t blink_on = (runtime->indicator.run_blink_counter < FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS) ? 1U : 0U;

        uint8_t region = motor->source_mgr_state.control_region;
        if (region == FOC_CONTROL_REGION_LOW)
        {
            /* 低速：蓝灯闪烁，红灯灭 */
            led_run_on = blink_on;
            led_error_on = 0U;
        }
        else
        {
            /* 高速(HIGH/FULL)：蓝灯红灯同相同步闪烁 */
            led_run_on = blink_on;
            led_error_on = blink_on;
        }
    }

    FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_run_on);
    FOC_Platform_SetIndicator(FOC_LED_ERROR_INDEX, led_error_on);

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
