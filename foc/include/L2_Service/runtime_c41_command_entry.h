#ifndef RUNTIME_C41_COMMAND_ENTRY_H
#define RUNTIME_C41_COMMAND_ENTRY_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L2_Service/runtime_snapshot.h"

typedef enum {
    COMMAND_MANAGER_SYSTEM_INIT = 0,
    COMMAND_MANAGER_SYSTEM_RUNNING,
    COMMAND_MANAGER_SYSTEM_FAULT
} command_manager_system_state_t;

typedef enum {
    COMMAND_MANAGER_COMM_IDLE = 0,
    COMMAND_MANAGER_COMM_ACTIVE,
    COMMAND_MANAGER_COMM_ERROR
} command_manager_comm_state_t;

typedef enum {
    COMMAND_MANAGER_REPORT_OFF = 0,
    COMMAND_MANAGER_REPORT_SEMANTIC_ONLY,
    COMMAND_MANAGER_REPORT_OSC_ONLY,
    COMMAND_MANAGER_REPORT_BOTH
} command_manager_report_mode_t;

typedef enum {
    COMMAND_MANAGER_DIAG_NOT_EXECUTED = 0,
    COMMAND_MANAGER_DIAG_SUCCESS,
    COMMAND_MANAGER_DIAG_FAILED
} command_manager_diag_state_t;

typedef enum {
    COMMAND_MANAGER_FAULT_NONE = 0,
    COMMAND_MANAGER_FAULT_SENSOR_ADC_INVALID,
    COMMAND_MANAGER_FAULT_SENSOR_ENCODER_INVALID,
    COMMAND_MANAGER_FAULT_UNDERVOLTAGE,
    COMMAND_MANAGER_FAULT_PROTOCOL_FRAME,
    COMMAND_MANAGER_FAULT_PARAM_INVALID,
    COMMAND_MANAGER_FAULT_INIT_FAILED
} command_manager_fault_code_t;

#define COMMAND_MANAGER_INIT_CHECK_COMM        (1U << 0)
#define COMMAND_MANAGER_INIT_CHECK_PROTOCOL    (1U << 1)
#define COMMAND_MANAGER_INIT_CHECK_COMMAND     (1U << 2)
#define COMMAND_MANAGER_INIT_CHECK_DEBUG       (1U << 3)
#define COMMAND_MANAGER_INIT_CHECK_SENSOR      (1U << 4)
#define COMMAND_MANAGER_INIT_CHECK_PWM         (1U << 5)
#define COMMAND_MANAGER_INIT_CHECK_MOTOR       (1U << 6)

#ifndef COMMAND_MANAGER_INIT_REQUIRED_MASK
#define COMMAND_MANAGER_INIT_REQUIRED_MASK (COMMAND_MANAGER_INIT_CHECK_COMM     | \
                                           COMMAND_MANAGER_INIT_CHECK_PROTOCOL | \
                                           COMMAND_MANAGER_INIT_CHECK_COMMAND  | \
                                           COMMAND_MANAGER_INIT_CHECK_DEBUG    | \
                                           COMMAND_MANAGER_INIT_CHECK_SENSOR   | \
                                           COMMAND_MANAGER_INIT_CHECK_PWM      | \
                                           COMMAND_MANAGER_INIT_CHECK_MOTOR)
#endif

void CommandManager_Init(void);
uint8_t CommandManager_ProcessCommStep(uint8_t max_frames);
void CommandManager_ReportInitCheck(uint16_t check_bit, uint8_t success);
void CommandManager_FinalizeInitDiagnostics(void);
void CommandManager_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid);
void CommandManager_ReportUndervoltageFault(float vbus_voltage);
void CommandManager_ReportControlLoopSkip(void);
void CommandManager_CaptureSnapshot(runtime_snapshot_t *snapshot);

void CommandManager_ClearDirtyFlag(void);

#endif /* RUNTIME_C41_COMMAND_ENTRY_H */
