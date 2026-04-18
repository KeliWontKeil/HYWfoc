#ifndef L2_SERVICE_C11_ENTRY_H
#define L2_SERVICE_C11_ENTRY_H

#include <stdint.h>

#include "L2_Service/l2_service_contract.h"

typedef enum {
	L2_SERVICE_C11_INIT_CHECK_COMM = (1U << 0),
	L2_SERVICE_C11_INIT_CHECK_PROTOCOL = (1U << 1),
	L2_SERVICE_C11_INIT_CHECK_COMMAND = (1U << 2),
	L2_SERVICE_C11_INIT_CHECK_DEBUG = (1U << 3),
	L2_SERVICE_C11_INIT_CHECK_SENSOR = (1U << 4),
	L2_SERVICE_C11_INIT_CHECK_PWM = (1U << 5),
	L2_SERVICE_C11_INIT_CHECK_MOTOR = (1U << 6)
} l2_service_c11_init_check_t;

void L2_ServiceC11_Init(void);
void L2_ServiceC11_ReportInitCheck(uint16_t check_bit, uint8_t success);
void L2_ServiceC11_FinalizeInitDiagnostics(void);
void L2_ServiceC11_ReportRuntimeSensorState(uint8_t adc_valid, uint8_t encoder_valid);
void L2_ServiceC11_ReportUndervoltageFault(float vbus_voltage);
void L2_ServiceC11_ReportControlLoopSkip(void);

uint8_t L2_ServiceC11_ProcessCommStep(uint8_t max_frames, l2_service_snapshot_t *snapshot);
void L2_ServiceC11_RefreshSnapshot(l2_service_snapshot_t *snapshot);
void L2_ServiceC11_CommitAppliedConfig(void);

#endif /* L2_SERVICE_C11_ENTRY_H */