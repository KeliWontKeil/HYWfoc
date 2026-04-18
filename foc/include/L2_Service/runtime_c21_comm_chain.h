#ifndef RUNTIME_C21_COMM_CHAIN_H
#define RUNTIME_C21_COMM_CHAIN_H

#include <stdint.h>

#include "L2_Service/runtime_snapshot.h"

typedef enum {
    RUNTIME_INIT_CHECK_COMM = (1U << 0),
    RUNTIME_INIT_CHECK_PROTOCOL = (1U << 1),
    RUNTIME_INIT_CHECK_COMMAND = (1U << 2),
    RUNTIME_INIT_CHECK_DEBUG = (1U << 3),
    RUNTIME_INIT_CHECK_SENSOR = (1U << 4),
    RUNTIME_INIT_CHECK_PWM = (1U << 5),
    RUNTIME_INIT_CHECK_MOTOR = (1U << 6)
} runtime_init_check_t;

void RuntimeService_Init(void);
void RuntimeService_ReportInitCheck(uint16_t check_bit, uint8_t success);
void RuntimeService_FinalizeInitDiagnostics(void);
void RuntimeService_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid);
void RuntimeService_ReportUndervoltageFault(float vbus_voltage);
void RuntimeService_ReportControlLoopSkip(void);

uint8_t RuntimeService_ProcessCommStep(uint8_t max_frames, runtime_snapshot_t *snapshot);
void RuntimeService_RefreshSnapshot(runtime_snapshot_t *snapshot);
void RuntimeService_CommitAppliedConfig(void);

#endif /* RUNTIME_C21_COMM_CHAIN_H */