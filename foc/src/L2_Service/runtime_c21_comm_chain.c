#include "L2_Service/runtime_chain_internal.h"

#include <string.h>

#include "L2_Service/protocol_parser.h"
#include "L2_Service/command_manager.h"

uint8_t RuntimeChain_ProcessCommStep(uint8_t max_frames)
{
    uint8_t consumed = 0U;
    uint8_t has_comm_activity = 0U;

    if (max_frames == 0U)
    {
        return 0U;
    }

    while ((ProtocolParser_IsParsePending() != 0U) && (consumed < max_frames))
    {
        ProtocolParser_Process();

        if (CommandManager_Process() != 0U)
        {
            has_comm_activity = 1U;
        }

        consumed++;
    }

    return has_comm_activity;
}

void RuntimeChain_BuildSnapshot(runtime_snapshot_t *snapshot)
{
    const command_manager_runtime_state_t *runtime;

    if (snapshot == 0)
    {
        return;
    }

    (void)memset(snapshot, 0, sizeof(*snapshot));

    runtime = CommandManager_GetRuntimeState();
    if (runtime != 0)
    {
        snapshot->runtime.system_running = (runtime->system_state == COMMAND_MANAGER_SYSTEM_RUNNING) ? 1U : 0U;
        snapshot->runtime.system_fault = (runtime->system_state == COMMAND_MANAGER_SYSTEM_FAULT) ? 1U : 0U;
        snapshot->runtime.params_dirty = runtime->params_dirty;
        snapshot->runtime.last_exec_ok = runtime->last_exec_ok;
    }

    snapshot->control_cfg.control_mode = CommandManager_GetControlMode();
    snapshot->control_cfg.target_angle_rad = CommandManager_GetTargetAngleRad();
    snapshot->control_cfg.angle_position_speed_rad_s = CommandManager_GetAngleSpeedRadS();
    snapshot->control_cfg.speed_only_rad_s = CommandManager_GetSpeedOnlyRadS();
    snapshot->control_cfg.sensor_sample_offset_percent = CommandManager_GetSensorSampleOffsetPercent();
    snapshot->control_cfg.pid_current_kp = CommandManager_GetCurrentPidKp();
    snapshot->control_cfg.pid_current_ki = CommandManager_GetCurrentPidKi();
    snapshot->control_cfg.pid_current_kd = CommandManager_GetCurrentPidKd();
    snapshot->control_cfg.pid_angle_kp = CommandManager_GetAnglePidKp();
    snapshot->control_cfg.pid_angle_ki = CommandManager_GetAnglePidKi();
    snapshot->control_cfg.pid_angle_kd = CommandManager_GetAnglePidKd();
    snapshot->control_cfg.pid_speed_kp = CommandManager_GetSpeedPidKp();
    snapshot->control_cfg.pid_speed_ki = CommandManager_GetSpeedPidKi();
    snapshot->control_cfg.pid_speed_kd = CommandManager_GetSpeedPidKd();
    snapshot->control_cfg.cfg_min_mech_angle_accum_delta_rad = CommandManager_GetControlMinMechAngleAccumDeltaRad();
    snapshot->control_cfg.cfg_angle_hold_integral_limit = CommandManager_GetControlAngleHoldIntegralLimit();
    snapshot->control_cfg.cfg_angle_hold_pid_deadband_rad = CommandManager_GetControlAngleHoldPidDeadbandRad();
    snapshot->control_cfg.cfg_speed_angle_transition_start_rad = CommandManager_GetControlSpeedAngleTransitionStartRad();
    snapshot->control_cfg.cfg_speed_angle_transition_end_rad = CommandManager_GetControlSpeedAngleTransitionEndRad();
    snapshot->control_cfg.motor_enabled = CommandManager_IsMotorEnabled();
    snapshot->control_cfg.current_soft_switch_enable = CommandManager_IsCurrentSoftSwitchEnabled();
    snapshot->control_cfg.current_soft_switch_mode = CommandManager_GetCurrentSoftSwitchMode();
    snapshot->control_cfg.current_soft_switch_auto_open_iq_a = CommandManager_GetCurrentSoftSwitchAutoOpenIqA();
    snapshot->control_cfg.current_soft_switch_auto_closed_iq_a = CommandManager_GetCurrentSoftSwitchAutoClosedIqA();

    snapshot->telemetry.semantic_report_enabled = CommandManager_IsSemanticReportEnabled();
    snapshot->telemetry.osc_report_enabled = CommandManager_IsOscilloscopeReportEnabled();
    snapshot->telemetry.semantic_report_freq_hz = CommandManager_GetSemanticReportFrequencyHz();
    snapshot->telemetry.osc_report_freq_hz = CommandManager_GetOscilloscopeReportFrequencyHz();
    snapshot->telemetry.osc_parameter_mask = CommandManager_GetOscilloscopeParameterMask();
}

void RuntimeChain_CommitAppliedConfig(void)
{
    CommandManager_ClearDirtyFlag();
}
