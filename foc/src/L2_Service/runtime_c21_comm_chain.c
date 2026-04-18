#include "L2_Service/runtime_chain_internal.h"

#include <string.h>

#include "L2_Service/protocol_parser.h"
#include "L2_Service/command_manager.h"
#include "L3_Algorithm/protocol_core.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"

#define PROTOCOL_PARSER_COMM_SOURCE_1 1U
#define PROTOCOL_PARSER_COMM_SOURCE_2 2U
#define PROTOCOL_PARSER_COMM_SOURCE_3 3U
#define PROTOCOL_PARSER_COMM_SOURCE_4 4U

static protocol_command_t g_latest_command;
static protocol_parser_result_t g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
static uint8_t g_preferred_comm_source = 0U;

static uint8_t ProtocolParser_IsFrameReadyFromSource(uint8_t source);
static uint16_t ProtocolParser_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len);
static uint16_t ProtocolParser_TryReadReadySources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);
static uint16_t ProtocolParser_TryReadAnySource(uint8_t *buffer, uint16_t max_len, uint8_t *source_used);

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

void ProtocolParser_Init(void)
{
    memset(&g_latest_command, 0, sizeof(g_latest_command));
    g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    g_preferred_comm_source = 0U;
}

uint8_t ProtocolParser_IsParsePending(void)
{
    uint8_t source;

    if ((g_preferred_comm_source != 0U) &&
        (ProtocolParser_IsFrameReadyFromSource(g_preferred_comm_source) != 0U))
    {
        return 1U;
    }

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_comm_source)
        {
            continue;
        }

        if (ProtocolParser_IsFrameReadyFromSource(source) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

void ProtocolParser_Process(void)
{
    uint8_t frame[PROTOCOL_PARSER_RX_MAX_LEN];
    const uint8_t *payload = 0;
    uint16_t payload_len = 0U;
    protocol_core_frame_parse_result_t parse_result = PROTOCOL_CORE_FRAME_PARSE_INVALID;
    uint16_t len;
    uint8_t source_used = 0U;

    len = ProtocolParser_TryReadReadySources(frame,
                                             PROTOCOL_PARSER_RX_MAX_LEN,
                                             &source_used);
    if (len == 0U)
    {
        len = ProtocolParser_TryReadAnySource(frame,
                                              PROTOCOL_PARSER_RX_MAX_LEN,
                                              &source_used);
    }

    if (len == 0U)
    {
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
        return;
    }

    if (source_used != 0U)
    {
        g_preferred_comm_source = source_used;
    }

    if (ProtocolCore_ExtractFrame(frame, len, &payload, &payload_len) == 0U)
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
        return;
    }

    parse_result = ProtocolCore_ParseFrame(payload, payload_len, &g_latest_command);
    if (parse_result == PROTOCOL_CORE_FRAME_PARSE_OK)
    {
        g_latest_command.updated = 1U;
        g_last_result = PROTOCOL_PARSER_RESULT_OK;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_OK_CHAR);
    }
    else if (parse_result == PROTOCOL_CORE_FRAME_PARSE_ADDRESS_MISMATCH)
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_NO_FRAME;
    }
    else
    {
        g_latest_command.updated = 0U;
        g_latest_command.frame_valid = 0U;
        g_last_result = PROTOCOL_PARSER_RESULT_FRAME_ERROR;
        FOC_Platform_WriteStatusByte((uint8_t)PROTOCOL_PARSER_STATUS_FRAME_ERROR_CHAR);
    }
}

const protocol_command_t *ProtocolParser_GetLatestCommand(void)
{
    return &g_latest_command;
}

void ProtocolParser_ClearUpdatedFlag(void)
{
    g_latest_command.updated = 0U;
}

protocol_parser_result_t ProtocolParser_GetLastResult(void)
{
    return g_last_result;
}

static uint8_t ProtocolParser_IsFrameReadyFromSource(uint8_t source)
{
    if (source == PROTOCOL_PARSER_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_IsFrameReady();
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_IsFrameReady();
    }

    return 0U;
}

static uint16_t ProtocolParser_ReadFrameFromSource(uint8_t source, uint8_t *buffer, uint16_t max_len)
{
    if (source == PROTOCOL_PARSER_COMM_SOURCE_1)
    {
        return FOC_Platform_CommSource1_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_2)
    {
        return FOC_Platform_CommSource2_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_3)
    {
        return FOC_Platform_CommSource3_ReadFrame(buffer, max_len);
    }

    if (source == PROTOCOL_PARSER_COMM_SOURCE_4)
    {
        return FOC_Platform_CommSource4_ReadFrame(buffer, max_len);
    }

    return 0U;
}

static uint16_t ProtocolParser_TryReadReadySources(uint8_t *buffer, uint16_t max_len, uint8_t *source_used)
{
    uint16_t len;
    uint8_t source;

    if ((buffer == 0) || (source_used == 0))
    {
        return 0U;
    }

    *source_used = 0U;

    if ((g_preferred_comm_source != 0U) &&
        (ProtocolParser_IsFrameReadyFromSource(g_preferred_comm_source) != 0U))
    {
        len = ProtocolParser_ReadFrameFromSource(g_preferred_comm_source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = g_preferred_comm_source;
            return len;
        }
    }

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        if (source == g_preferred_comm_source)
        {
            continue;
        }

        if (ProtocolParser_IsFrameReadyFromSource(source) == 0U)
        {
            continue;
        }

        len = ProtocolParser_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = source;
            return len;
        }
    }

    return 0U;
}

static uint16_t ProtocolParser_TryReadAnySource(uint8_t *buffer, uint16_t max_len, uint8_t *source_used)
{
    uint16_t len;
    uint8_t source;

    if ((buffer == 0) || (source_used == 0))
    {
        return 0U;
    }

    *source_used = 0U;

    for (source = PROTOCOL_PARSER_COMM_SOURCE_1; source <= PROTOCOL_PARSER_COMM_SOURCE_4; source++)
    {
        len = ProtocolParser_ReadFrameFromSource(source, buffer, max_len);
        if (len > 0U)
        {
            *source_used = source;
            return len;
        }
    }

    return 0U;
}
