#ifndef FOC_CTRL_TYPES_H
#define FOC_CTRL_TYPES_H

#include <stdint.h>

#include "LS_Config/foc_config.h"
#include "L3_Hal/foc_math_types.h"

/* 过渡期（阶段 1b 前）：保持 sensor_data_t / svpwm 类型可见性
 * 阶段 1b 完成后，L2 共享类型层不再依赖 L3 类型头。 */
#include "L3_Hal/foc_sensor.h"
#include "L3_Hal/foc_svpwm.h"

/* ========== Alpha-beta / three-phase voltage state ========== */
typedef struct {
    float alpha;
    float beta;
    float phase_a;
    float phase_b;
    float phase_c;
} foc_alpha_beta_phase_t;

/* ========== Source identifiers and published source view ========== */
typedef enum {
    FOC_SOURCE_TYPE_NONE     = FOC_CONTROL_SRC_NONE,
    FOC_SOURCE_TYPE_ENCODER  = FOC_CONTROL_SRC_ENCODER,
    FOC_SOURCE_TYPE_SMO      = FOC_CONTROL_SRC_SMO,
    FOC_SOURCE_TYPE_HFI      = FOC_CONTROL_SRC_HFI,
    FOC_SOURCE_TYPE_OPENLOOP = FOC_CONTROL_SRC_OPENLOOP,
    FOC_SOURCE_TYPE_FLUX     = FOC_CONTROL_SRC_FLUX
} foc_source_type_t;

typedef enum {
    FOC_SOURCE_STATE_INIT = 0U,
    FOC_SOURCE_STATE_CONVERGING = 1U,
    FOC_SOURCE_STATE_LOCKED = 2U,
    FOC_SOURCE_STATE_DIVERGED = 3U
} foc_source_state_t;

typedef struct {
    uint8_t  source;
    uint8_t  state;
    uint8_t  valid;
    float    confidence;

    float    elec_angle_rad;
    float    mech_angle_rad;
    float    mech_speed_rad_s;
} foc_active_source_state_t;

/* ========== Source/control region switch hint（跨域枚举） ========== */
typedef enum {
    FOC_CONTROL_REGION_LOW = 0U,
    FOC_CONTROL_REGION_HIGH = 1U,
    FOC_CONTROL_REGION_FULL = 2U
} foc_control_region_t;

/* ========== Encoder services availability ========== */
typedef struct {
    uint8_t comp_available;
    uint8_t comp_active;
    uint8_t calib_available;
    uint8_t reinit_available;
} foc_encoder_services_state_t;

/* ========== Phase output (special-phase state machines) ========== */
typedef enum {
    FOC_PHASE_OUTPUT_IDLE = 0U,
    FOC_PHASE_OUTPUT_ZERO,
    FOC_PHASE_OUTPUT_DQ_VOLTAGE_ANGLE,
    FOC_PHASE_OUTPUT_DIRECT_DUTY
} foc_phase_output_type_t;

typedef struct {
    uint8_t phase;
    uint8_t type;
    uint8_t valid;
    uint8_t state_id;

    float duty_a;
    float duty_b;
    float duty_c;
    uint8_t sector;
} foc_phase_output_state_t;

typedef struct {
    uint8_t valid;
    float ud;
    float uq;
    float electrical_angle_rad;
} foc_applied_output_state_t;

/* ========== 故障码枚举（per-motor） ========== */
typedef enum {
    FOC_FAULT_NONE = 0U,
    FOC_FAULT_SENSOR_ADC_INVALID = 1U,
    FOC_FAULT_SENSOR_ENCODER_INVALID = 2U,
    FOC_FAULT_UNDERVOLTAGE = 3U,
    FOC_FAULT_PROTOCOL_FRAME = 4U,
    FOC_FAULT_PARAM_INVALID = 5U,
    FOC_FAULT_INIT_FAILED = 6U,
    FOC_FAULT_ESTIMATOR_INVALID = 7U
} foc_fault_code_t;

/* ========== 控制阶段枚举 ========== */
typedef enum {
    FOC_CONTROL_PHASE_NORMAL        = 0U,
    FOC_CONTROL_PHASE_COGGING_CALIB = 1U,
    FOC_CONTROL_PHASE_REINIT        = 2U
} foc_control_phase_t;

/* ========== 运行时状态（per-motor） ========== */
typedef struct {
    uint8_t system_running;
    uint8_t system_fault;
    uint8_t last_fault_code;
    uint8_t motor_enabled;
    uint8_t control_mode;
    uint8_t control_phase;
    uint8_t current_loop_ready;
    uint16_t init_check_mask;
    uint16_t init_fail_mask;
    uint16_t sensor_invalid_consecutive;
    uint32_t protocol_error_count;
    uint32_t param_error_count;
    uint32_t control_skip_count;
} foc_motor_state_t;

/* ========== D/Q 控制运行时（控制链总线） ========== */
typedef struct {
    float electrical_angle_rad;
    float ud;
    float uq;
    float max_phase_voltage;
    float iq_target;
    float iq_measured;
} foc_control_runtime_t;

/* ========== 控制参考（控制 ISR → 电流环 ISR 单点原子发布块） ========== */
/* 由控制 ISR 在过程末尾一次性写入并提交（写块 → MemoryBarrier → ctrl_ref_ready=1），
 * 电流环 ISR 在过程开头原子获取为本地快照。编码器字段仅当控制 ISR 负责采样时
 * （FOC_SENSOR_ANGLE_FAST_ENABLE == DISABLE）纳入；FAST 模式下编码器由电流环自持。 */
typedef struct {
    float iq_target;
    float ramped_speed_rad_s;
#if (FOC_SENSOR_ANGLE_FAST_ENABLE == FOC_CFG_DISABLE)
    float mech_angle_rad;
    float mech_speed_rad_s;
    uint8_t encoder_valid;
#endif
#if (FOC_OPENLOOP_SOURCE_ENABLE == FOC_CFG_ENABLE)
    uint8_t openloop_phase;
    float openloop_virtual_angle_rad;
    float openloop_virtual_speed_rad_s;
    float openloop_mech_speed_rad_s;
#endif
} foc_control_ref_t;

/* ========== 电机物理参数 ========== */
typedef struct {
    float phase_resistance;
    float stator_inductance;
    uint8_t pole_pairs;
    float mech_angle_at_elec_zero_rad;
    int8_t direction;
    float vbus_voltage;
} foc_motor_params_t;

/* ========== 控制目标与调优配置 ========== */
typedef struct {
    float target_angle_rad;
    float angle_position_speed_rad_s;
    float speed_only_rad_s;
    float sensor_sample_offset_percent;
    float min_mech_angle_accum_delta_rad;
    float angle_hold_integral_limit;
    float angle_hold_pid_deadband_rad;
    float speed_angle_transition_start_rad;
    float speed_angle_transition_end_rad;
} foc_control_cfg_t;

/* ========== Torque mode ========== */
typedef enum {
    FOC_TORQUE_MODE_OPEN_LOOP = 0,
    FOC_TORQUE_MODE_CURRENT_PID = 1
} foc_torque_mode_t;

#endif /* FOC_CTRL_TYPES_H */
