#include "L2_Core/foc_motor_aggregate.h"
#include "L1_Orchestration/foc_indicator.h"

#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* 显示态（互斥，优先级自上而下）：判定与灯效输出解耦，新增状态只需扩本枚举与判定函数 */
typedef enum {
    FOC_IND_STATE_FAULT = 0U,   /* 故障：红常亮，蓝灭（最高优先） */
    FOC_IND_STATE_CALIB,        /* 对齐/标定：蓝常亮 */
    FOC_IND_STATE_PLAY,         /* 声学播放：蓝按曲目节拍闪烁 */
    FOC_IND_STATE_RUN,          /* 正常控制：区域指示（既有规则不变） */
    FOC_IND_STATE_IDLE          /* 未运行：全灭 */
} foc_indicator_state_t;

/* 判定显示态：control_phase 各态天然互斥，故 fault 之下为同一优先级 */
static foc_indicator_state_t Indicator_ResolveState(const foc_motor_t *motor)
{
    if (motor->state.system_fault != 0U)
    {
        return FOC_IND_STATE_FAULT;
    }

    if ((motor->state.control_phase == FOC_CONTROL_PHASE_COGGING_CALIB) ||
        (motor->state.control_phase == FOC_CONTROL_PHASE_REINIT) ||
        (motor->state.control_phase == FOC_CONTROL_PHASE_STARTUP))
    {
        return FOC_IND_STATE_CALIB;
    }

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
    if (motor->state.control_phase == FOC_CONTROL_PHASE_ACOUSTIC)
    {
        return FOC_IND_STATE_PLAY;
    }
#endif

    if (motor->state.system_running != 0U)
    {
        return FOC_IND_STATE_RUN;
    }

    return FOC_IND_STATE_IDLE;
}

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
/* 播放态闪烁半周期（控制率拍）= 半个四分音符 = 30 · 控制率 / bpm，收敛到可读区间；
 * 无有效节拍（bpm = 0）时退化为最慢档。 */
static uint16_t Indicator_PlayBlinkHalfTicks(uint16_t bpm)
{
    uint32_t half;

    if (bpm == 0U)
    {
        return (uint16_t)FOC_LED_PLAY_BLINK_MAX_HALF_TICKS;
    }

    half = ((uint32_t)30U * (uint32_t)FOC_SCHEDULER_CONTROL_HZ) / (uint32_t)bpm;
    if (half < (uint32_t)FOC_LED_PLAY_BLINK_MIN_HALF_TICKS)
    {
        half = (uint32_t)FOC_LED_PLAY_BLINK_MIN_HALF_TICKS;
    }
    else if (half > (uint32_t)FOC_LED_PLAY_BLINK_MAX_HALF_TICKS)
    {
        half = (uint32_t)FOC_LED_PLAY_BLINK_MAX_HALF_TICKS;
    }

    return (uint16_t)half;
}
#endif /* FOC_ACOUSTIC_ENABLE */

/* 推进闪烁计数并返回本拍电平（period_ticks = 半周期，控制率拍） */
static uint8_t Indicator_BlinkStep(uint16_t *counter, uint16_t period_ticks)
{
    *counter += 1U;
    if (*counter >= (uint16_t)(2U * period_ticks))
    {
        *counter = 0U;
    }

    return (*counter < period_ticks) ? 1U : 0U;
}

void FOC_Indicator_Update(foc_motor_t *motor, foc_runtime_ctx_t *runtime)
{
    foc_indicator_runtime_t *ind = &runtime->indicator;
    uint8_t led_blue = 0U;
    uint8_t led_red = 0U;
    uint8_t led_green;

    switch (Indicator_ResolveState(motor))
    {
    case FOC_IND_STATE_FAULT:
        ind->run_blink_counter = 0U;
        led_red = 1U;
        break;

    case FOC_IND_STATE_CALIB:
        ind->run_blink_counter = 0U;
        led_blue = 1U;
        break;

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
    case FOC_IND_STATE_PLAY:
        led_blue = Indicator_BlinkStep(&ind->run_blink_counter,
                                       Indicator_PlayBlinkHalfTicks(motor->acoustic_state.bpm));
        break;
#endif

    case FOC_IND_STATE_RUN:
    {
        uint8_t blink_on = Indicator_BlinkStep(&ind->run_blink_counter,
                                               (uint16_t)FOC_LED_RUN_BLINK_HALF_PERIOD_TICKS);

        led_blue = blink_on;
        /* 低速区：仅蓝灯；高速区(HIGH/FULL)：蓝红同相同步（既有区域指示） */
        led_red = (motor->source_mgr_state.control_region == FOC_CONTROL_REGION_LOW) ? 0U : blink_on;
        break;
    }

    default:
        ind->run_blink_counter = 0U;
        break;
    }

    /* 电平缓存：仅在实际变化时写 GPIO（本函数运行在控制率 ISR 内） */
    if (ind->last_blue != led_blue)
    {
        ind->last_blue = led_blue;
        FOC_Platform_SetIndicator(FOC_LED_RUN_INDEX, led_blue);
    }

    if (ind->last_red != led_red)
    {
        ind->last_red = led_red;
        FOC_Platform_SetIndicator(FOC_LED_ERROR_INDEX, led_red);
    }

    led_green = (ind->comm_pulse_counter > 0U) ? 1U : 0U;
    if (ind->comm_pulse_counter > 0U)
    {
        ind->comm_pulse_counter--;
    }

    if (ind->last_green != led_green)
    {
        ind->last_green = led_green;
        FOC_Platform_SetIndicator(FOC_LED_COMM_INDEX, led_green);
    }
}
