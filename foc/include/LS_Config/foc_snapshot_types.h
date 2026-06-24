#ifndef FOC_SNAPSHOT_TYPES_H
#define FOC_SNAPSHOT_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_cfg_feature_switches.h"

/* ========== 遥测策略（系统级配置，非 per-motor） ========== */
typedef struct {
    uint8_t semantic_report_enabled;
    uint8_t osc_report_enabled;
    uint16_t semantic_report_freq_hz;
    uint16_t osc_report_freq_hz;
    uint16_t osc_parameter_mask;
} telemetry_policy_snapshot_t;

#endif /* FOC_SNAPSHOT_TYPES_H */
