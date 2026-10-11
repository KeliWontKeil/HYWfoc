#include "L1_Orchestration/foc_app_event.h"

#include "L1_Orchestration/foc_output_mgr.h"
#include "L2_Core/Control/foc_ctrl_align.h"
#include "L2_Core/Control/foc_ctrl_acoustic.h"
#include "LS_Config/foc_ringtone_table.h"

/* fault 0→1 跃迁当轮补发完整详情（慢路径可靠输出；无故障音——
 * fault 保护功率输出，声学需 NORMAL/ACOUSTIC 相位） */
static void AppEvent_ReportFaultTransition(foc_app_event_state_t *ev, foc_motor_t *motor)
{
    if ((motor->state.system_fault != 0U) && (ev->prev_system_fault == 0U))
    {
        FOC_OutputMgr_WriteFaultReport(motor);
    }
    ev->prev_system_fault = motor->state.system_fault;
}

/* 对齐/重新对齐完成后补发启动信息（完成报告由对齐状态机在 ISR 内只置位） */
static void AppEvent_ReportAlignment(foc_motor_t *motor)
{
    if (FOC_Align_TakeReport(motor) != 0U)
    {
        FOC_OutputMgr_WriteStartupInfo(motor);
    }
}

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
/* 就绪播报：上电（从无到有）= 长鸣；其他进入正常态（恢复 / 正常推进）= 两短鸣。
 * ACOUSTIC 视为"仍就绪"（它是通知，不改变控制态），避免曲终回转被误判为进入正常态。 */
static void AppEvent_AnnounceReadySound(foc_app_event_state_t *ev, foc_motor_t *motor)
{
    uint8_t ready;

    ready = ((motor->state.system_running != 0U) &&
             (motor->state.system_fault == 0U) &&
             (motor->state.motor_enabled != 0U) &&
             ((motor->state.control_phase == FOC_CONTROL_PHASE_NORMAL) ||
              (motor->state.control_phase == FOC_CONTROL_PHASE_ACOUSTIC))) ? 1U : 0U;

    if ((ready != 0U) && (ev->ready_prev == 0U))
    {
        uint8_t tune_id = (ev->ready_announced == 0U) ? (uint8_t)FOC_RINGTONE_ID_LONG_BEEP
                                                      : (uint8_t)FOC_RINGTONE_ID_TWO_SHORT_BEEPS;

        ev->ready_announced = 1U;
        (void)FOC_Acoustic_RequestTune(motor, tune_id);
    }

    ev->ready_prev = ready;
}
#endif /* FOC_ACOUSTIC_ENABLE */

void FOC_AppEvent_Run(foc_system_t *sys, foc_motor_t *motor)
{
    AppEvent_ReportFaultTransition(&sys->events, motor);
    AppEvent_ReportAlignment(motor);

#if (FOC_ACOUSTIC_ENABLE == FOC_CFG_ENABLE)
    AppEvent_AnnounceReadySound(&sys->events, motor);
#endif
}
