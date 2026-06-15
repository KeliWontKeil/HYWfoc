#include "L2_Service/runtime_c1_entry.h"

#include "L2_Service/runtime_c2_frame_source.h"

/* L1层运行时入口：初始化运行时管线 */
void Runtime_Init(void)
{
    RuntimeC2_Init();
}

/* L1层：处理一帧通讯（最多处理frame_budget帧），返回是否有通讯活动 */
uint8_t Runtime_FrameRunStep(uint8_t frame_budget)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    while (consumed < frame_budget)
    {
        if (RuntimeC2_ProcessOneFrame() != 0U)
        {
            has_comm_activity = 1U;
        }
        consumed++;
    }

    return has_comm_activity;
}

/* L1层：将步进信号（传感器状态/初始化结果/故障等）下传到L2管线 */
void Runtime_UpdateSignals(const runtime_step_signal_t *signals)
{
    RuntimeC2_UpdateSignals(signals);
}

/* L1层：获取当前运行时快照（参数+状态+遥测） */
void Runtime_GetSnapshot(runtime_snapshot_t *snapshot)
{
    RuntimeC2_BuildSnapshot(snapshot);
}

/* L1层：提交参数变更（清除dirty标志） */
void Runtime_Commit(void)
{
    RuntimeC2_Commit();
}

/* L1层：清除重初始化请求标志 */
void Runtime_ClearReinit(void)
{
    RuntimeC2_ClearReinit();
}