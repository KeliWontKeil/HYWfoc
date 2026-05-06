#include "L3_Algorithm/foc_control_c12_init.h"

#include <stdio.h>
#include <math.h>

#include "L3_Algorithm/foc_control_c22_current_loop.h"
#include "L3_Algorithm/foc_control_c23_motor_param_learn.h"
#include "L3_Algorithm/foc_control_c24_compensation.h"
#include "L3_Algorithm/foc_control_c13_cfg_state.h"
#include "L41_Math/foc_math_lut.h"
#include "L41_Math/math_transforms.h"
#include "L42_PAL/foc_platform_api.h"
#include "LS_Config/foc_config.h"
#include "LS_Config/foc_cogging_table.h"

float Math_WrapRad(float angle);
float Math_WrapRadDelta(float angle);
float Math_ClampFloat(float value, float min_val, float max_val);

void FOC_CalibrateElectricalAngleAndDirection(foc_motor_t *motor)
{
    float calib_uq;
    float backup_ud;
    float backup_uq;
    int8_t direction_est;
    uint8_t pole_pairs_est;
    float mech_zero_rad_est;
    uint8_t need_zero;
    uint8_t need_direction;
    uint8_t need_pole_pairs;

    if (motor == 0)
    {
        return;
    }

    need_zero = (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED) ? 1U : 0U;
    need_direction = (motor->direction == FOC_DIR_UNDEFINED) ? 1U : 0U;
    need_pole_pairs = (motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ? 1U : 0U;

    if ((need_zero == 0U) && (need_direction == 0U) && (need_pole_pairs == 0U))
    {
        return;
    }

    backup_ud = motor->ud;
    backup_uq = motor->uq;

    calib_uq = motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO;
    calib_uq = Math_ClampFloat(calib_uq, 0.0f, motor->set_voltage);

    motor->uq = 0.0f;
    motor->ud = calib_uq;

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_ZERO_LOCK_SETTLE_MS,
                                            FOC_CALIB_ZERO_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->mech_angle_at_elec_zero_rad = mech_zero_rad_est;
            motor->mech_angle_accum_rad = mech_zero_rad_est;
            motor->mech_angle_prev_rad = mech_zero_rad_est;
            motor->mech_angle_prev_valid = 1U;
        }
        else
        {
            motor->mech_angle_at_elec_zero_rad = FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED;
            motor->mech_angle_accum_rad = 0.0f;
            motor->mech_angle_prev_rad = 0.0f;
            motor->mech_angle_prev_valid = 0U;
            FOC_Platform_WriteDebugText("init.calib: zero-lock sampling failed, keep zero as undefined\r\n");
        }
    }
    else
    {
        FOC_CurrentControlApplyElectricalAngleDirect(motor, 0.0f);
        FOC_Platform_WaitMs(FOC_CALIB_ZERO_LOCK_SETTLE_MS);
    }

    if ((need_direction != 0U) || (need_pole_pairs != 0U))
    {
        if (FOC_EstimateDirectionAndPolePairs(motor, &direction_est, &pole_pairs_est) != 0U)
        {
            if (need_direction != 0U)
            {
                motor->direction = direction_est;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = pole_pairs_est;
            }
        }
        else
        {
            if (need_direction != 0U)
            {
                motor->direction = FOC_DIR_UNDEFINED;
            }
            if (need_pole_pairs != 0U)
            {
                motor->pole_pairs = FOC_POLE_PAIRS_UNDEFINED;
            }
            FOC_Platform_WriteDebugText("init.calib: direction/pole-pairs estimation failed, keep as undefined\r\n");
        }
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_CurrentControlApplyElectricalAngleDirect(motor, 0.0f);
}

void FOC_MotorInit(foc_motor_t *motor,
                   float vbus_voltage,
                   float set_voltage,
                   float phase_resistance,
                   uint8_t pole_pairs,
                   float mech_angle_at_elec_zero_rad,
                   int8_t direction)
{
    if (motor == 0)
    {
        return;
    }

    if (vbus_voltage < 0.0f)
    {
        vbus_voltage = 0.0f;
    }
    set_voltage = Math_ClampFloat(set_voltage, 0.0f, vbus_voltage);

    motor->electrical_phase_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 0.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->iq_target = 0.0f;

    motor->iq_measured = 0.0f;
    motor->cogging_speed_ref_rad_s = 0.0f;
    motor->mech_angle_accum_rad = 0.0f;
    motor->mech_angle_prev_rad = 0.0f;
    motor->mech_angle_prev_valid = 0U;
    motor->phase_resistance = phase_resistance;
    motor->pole_pairs = pole_pairs;
    motor->mech_angle_at_elec_zero_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_accum_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_rad = mech_angle_at_elec_zero_rad;
    motor->mech_angle_prev_valid = 1U;
    motor->direction = direction;

    motor->alpha = 0.0f;
    motor->beta = 0.0f;
    motor->phase_a = 0.0f;
    motor->phase_b = 0.0f;
    motor->phase_c = 0.0f;
    motor->duty_a = 0.0f;
    motor->duty_b = 0.0f;
    motor->duty_c = 0.0f;
    motor->sector = 0U;

    FOC_ControlConfigResetDefault(motor);

#if (FOC_INIT_CALIBRATION_ENABLE == FOC_CFG_ENABLE)
    FOC_CalibrateElectricalAngleAndDirection(motor);
#endif
#if (FOC_COGGING_COMP_ENABLE == FOC_CFG_ENABLE)
    {
        uint8_t table_defined = FOC_CFG_DISABLE;

#if (FOC_COGGING_STATIC_TABLE_DEFINED == FOC_CFG_ENABLE)
        table_defined = FOC_CFG_ENABLE;
#endif
        motor->cogging_comp_status.available = table_defined;
        motor->cogging_comp_status.enabled = table_defined;
        motor->cogging_comp_status.source = table_defined ? FOC_COGGING_COMP_SOURCE_STATIC : FOC_COGGING_COMP_SOURCE_NONE;
        motor->cogging_comp_status.point_count = FOC_COGGING_LUT_POINT_COUNT;
        motor->cogging_comp_status.iq_lsb_a = FOC_COGGING_LUT_IQ_LSB_A;
        motor->cogging_comp_status.speed_gate_rad_s = FOC_COGGING_COMP_SPEED_GATE_RAD_S;
        motor->cogging_comp_status.iq_limit_a = FOC_COGGING_COMP_IQ_LIMIT_A;

#if (FOC_COGGING_STATIC_TABLE_DEFINED == FOC_CFG_ENABLE)
        (void)FOC_ControlLoadCoggingCompTableQ15(motor,
                                                  foc_cogging_default_table_q15,
                                                  FOC_COGGING_LUT_POINT_COUNT,
                                                  FOC_COGGING_LUT_IQ_LSB_A,
                                                  FOC_COGGING_COMP_SOURCE_STATIC);
#endif

        if (table_defined != 0U)
        {
            FOC_Platform_WriteDebugText("init.cogging: static table defined, compensation ready\r\n");
        }
        else
        {
            FOC_Platform_WriteDebugText("init.cogging: no table defined, use Y:G to calibrate or set static table\r\n");
        }
    }
#endif
}
