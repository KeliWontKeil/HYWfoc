#include "L2_Core/Control/foc_ctrl_reinit.h"

#if (FOC_REINIT_ENABLE == FOC_CFG_ENABLE)

#include <math.h>
#include <stdio.h>

#include "L2_Core/Control/foc_ctrl_actuation.h"
#include "L2_Core/Control/foc_ctrl_cfg.h"
#include "L2_Core/Control/foc_ctrl_init.h"
#include "L2_Core/Control/foc_ctrl_current_loop.h"
#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "L3_Hal/foc_svpwm.h"
#include "L3_Hal/foc_sensor.h"
#include "LS_Config/foc_config.h"

/* ========== 内部工具 ========== */

/* 根据 dt_sec 和毫秒数计算等待周期数 */
static uint16_t ReInit_MsToCycles(float dt_sec, uint16_t ms)
{
    float cycles_f;
    if (dt_sec <= 0.0f)
    {
        dt_sec = FOC_CONTROL_DT_SEC;
    }
    cycles_f = (float)ms / (dt_sec * 1000.0f);
    if (cycles_f < 1.0f) cycles_f = 1.0f;
    return (uint16_t)(cycles_f + 0.5f);
}

/* 写日志 */
static void ReInit_WriteLog(const char *msg)
{
    FOC_Platform_WriteDebugText(msg);
}

/* D 轴对齐电压：与阻塞标定 FOC_CalibrateElectricalAngleAndDirection 一致 */
static void ReInit_ApplyDAlign(foc_motor_t *motor, float calib_uq)
{
    motor->uq = 0.0f;
    motor->ud = calib_uq;
    FOC_ControlApplyElectricalAngleDirect(motor, 0.0f);
}

/* 归零输出 */
static void ReInit_ZeroOutput(foc_motor_t *motor, float dt_sec)
{
    FOC_CurrentControlOpenLoopStep(motor, 0.0f, 0.0f, dt_sec);
    SVPWM_ApplyDirectDuty(motor, 0U, 0.0f, 0.0f, 0.0f);
}

/* sin/cos 矢量平均采样机械角度 */
static float ReInit_SampleMechanicalAngle(foc_reinit_state_t *rs)
{
    float sample_rad;
    if (FOC_Platform_ReadMechanicalAngleRad(&sample_rad) == 0U)
    {
        return -999.0f; /* 无效 */
    }
    rs->sin_sum += FOC_MathLut_Sin(sample_rad);
    rs->cos_sum += FOC_MathLut_Sin(sample_rad + FOC_MATH_PI * 0.5f);
    rs->sample_count++;
    return sample_rad;
}

/* 从 sin_sum/cos_sum 计算机械角度 */
static float ReInit_AngleFromSum(const foc_reinit_state_t *rs)
{
    if ((fabsf(rs->sin_sum) < 1e-6f) && (fabsf(rs->cos_sum) < 1e-6f))
    {
        return -999.0f;
    }
    return Math_WrapRad(FOC_MathLut_Atan2(rs->sin_sum, rs->cos_sum));
}

/* ========== 重初始化内部状态机步进 ========== */

uint8_t FOC_ReInit_RunStep(foc_motor_t *motor, float dt_sec)
{
    foc_reinit_state_t *rs;

    if (motor == 0)
    {
        return 0U;
    }

    rs = &motor->reinit_state;
    if (dt_sec <= 0.0f)
    {
        dt_sec = FOC_CONTROL_DT_SEC;
    }

    /* IDLE -> 直接进入 STOP */
    if (rs->phase == FOC_REINIT_PHASE_IDLE)
    {
        /* 停止电机 */
        ReInit_ZeroOutput(motor, dt_sec);
        motor->state.current_loop_ready = 0U;

        rs->phase = FOC_REINIT_PHASE_STOP;
        rs->settle_cycles = 0U;
        rs->sample_count = 0U;
        rs->sample_target = 0U;
        rs->sin_sum = 0.0f;
        rs->cos_sum = 0.0f;
        rs->calib_uq = 0.0f;
        rs->elec_angle_rad = 0.0f;
        rs->has_prev = 0U;
        rs->step_index = 0U;
        rs->step_count = 0U;
        rs->reverse_pass = 0U;
        rs->prev_mech_rad = 0.0f;
        rs->prev_elec_rad = 0.0f;
        rs->sum_d_mech = 0.0f;
        rs->sum_d_elec = 0.0f;

        ReInit_WriteLog("\r\n=== Non-blocking ReInit started ===\r\n");
        return 1U;
    }

    switch (rs->phase)
    {
    /* ========== STOP: 已完成停止，进入零漂采样 ========== */
    case FOC_REINIT_PHASE_STOP:
    {
        rs->phase = FOC_REINIT_PHASE_ZERO_SAMPLE;
        rs->sample_count = 0U;
        rs->sample_target = SENSOR_ZERO_CALIB_SAMPLES;
        motor->sensor_zero_offset_a = 0.0f;
        motor->sensor_zero_offset_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        motor->sensor_zero_offset_c = 0.0f;
#endif
        ReInit_WriteLog("ReInit: zero offset sampling\r\n");
        return 1U;
    }

    /* ========== ZERO_SAMPLE: 非阻塞零漂采样 ========== */
    case FOC_REINIT_PHASE_ZERO_SAMPLE:
    {
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
        float cur_a = 0.0f;
        float cur_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        float cur_c = 0.0f;
        if (FOC_Platform_ReadPhaseCurrent(&cur_a, &cur_b, &cur_c) != 0U)
#else
        if (FOC_Platform_ReadPhaseCurrent(&cur_a, &cur_b, 0) != 0U)
#endif
        {
            motor->sensor_zero_offset_a += cur_a;
            motor->sensor_zero_offset_b += cur_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
            motor->sensor_zero_offset_c += cur_c;
#endif
            rs->sample_count++;
        }
#else
        rs->sample_count = rs->sample_target;
#endif
        if (rs->sample_count >= rs->sample_target)
        {
            /* 计算均值 */
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
            float count_f = (float)rs->sample_count;
            if (count_f > 1.0f)
            {
                motor->sensor_zero_offset_a /= count_f;
                motor->sensor_zero_offset_b /= count_f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
                motor->sensor_zero_offset_c /= count_f;
#endif
                if (fabsf(motor->sensor_zero_offset_a) > SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
                    motor->sensor_zero_offset_a = 0.0f;
                if (fabsf(motor->sensor_zero_offset_b) > SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
                    motor->sensor_zero_offset_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
                if (fabsf(motor->sensor_zero_offset_c) > SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
                    motor->sensor_zero_offset_c = 0.0f;
#endif
            }
#endif
            ReInit_WriteLog("ReInit: zero offset done\r\n");

            /* 计算对齐电压，与阻塞标定一致：ud = calib_uq, uq = 0 */
            rs->calib_uq = motor->set_voltage * FOC_CALIB_ALIGN_VOLTAGE_RATIO;
            rs->calib_uq = Math_ClampFloat(rs->calib_uq, 0.0f, motor->set_voltage);

            /* 进入对齐 */
            rs->phase = FOC_REINIT_PHASE_ALIGN_SETTLE;
            rs->settle_cycles = ReInit_MsToCycles(dt_sec, FOC_CALIB_ZERO_LOCK_SETTLE_MS);
            rs->sin_sum = 0.0f;
            rs->cos_sum = 0.0f;
            rs->sample_count = 0U;
            rs->sample_target = FOC_CALIB_ZERO_LOCK_SAMPLE_COUNT;

            ReInit_ApplyDAlign(motor, rs->calib_uq);
        }
        return 1U;
    }

    /* ========== ALIGN_SETTLE: D 轴对齐等待 ========== */
    case FOC_REINIT_PHASE_ALIGN_SETTLE:
    {
        ReInit_ApplyDAlign(motor, rs->calib_uq);

        if (rs->settle_cycles > 0U)
        {
            rs->settle_cycles--;
        }
        if (rs->settle_cycles == 0U)
        {
            rs->phase = FOC_REINIT_PHASE_ALIGN_SAMPLE;
            rs->sin_sum = 0.0f;
            rs->cos_sum = 0.0f;
            rs->sample_count = 0U;
        }
        return 1U;
    }

    /* ========== ALIGN_SAMPLE: D 轴对齐采样 ========== */
    case FOC_REINIT_PHASE_ALIGN_SAMPLE:
    {
        ReInit_ApplyDAlign(motor, rs->calib_uq);
        (void)ReInit_SampleMechanicalAngle(rs);

        if (rs->sample_count >= rs->sample_target)
        {
            float mech_zero = ReInit_AngleFromSum(rs);
            if (mech_zero >= -998.0f)
            {
                motor->mech_angle_at_elec_zero_rad = mech_zero;
                motor->mech_angle_accum_rad = mech_zero;
                motor->mech_angle_prev_rad = mech_zero;
                motor->mech_angle_prev_valid = 1U;
            }
            else
            {
                motor->mech_angle_at_elec_zero_rad = FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED;
                motor->mech_angle_accum_rad = 0.0f;
                motor->mech_angle_prev_rad = 0.0f;
                motor->mech_angle_prev_valid = 0U;
            }
            ReInit_WriteLog("ReInit: zero align done\r\n");

            /* 退磁 */
            motor->uq = 0.0f;
            motor->ud = 0.0f;
            FOC_ControlApplyElectricalAngleDirect(motor, 0.0f);

            /* 进入方向/极对数粗步进 */
            rs->phase = FOC_REINIT_PHASE_DIR_STEP;
            rs->step_index = 0U;
            rs->step_count = (uint8_t)FOC_CALIB_COARSE_STEP_COUNT;
            rs->has_prev = 0U;
            rs->sum_d_mech = 0.0f;
            rs->sum_d_elec = 0.0f;
            rs->prev_mech_rad = 0.0f;
            rs->prev_elec_rad = 0.0f;
            rs->reverse_pass = 0U;
        }
        return 1U;
    }

    /* ========== DIR_STEP: 步进到下一个电角度位置（开环步进） ========== */
    case FOC_REINIT_PHASE_DIR_STEP:
    {
        float step_elec = FOC_CALIB_COARSE_STEP_ELEC_RAD;
        rs->elec_angle_rad = step_elec * (float)rs->step_index;

        /* 应用 D 轴对齐电压在目标电角度 */
        motor->uq = 0.0f;
        motor->ud = rs->calib_uq;
        FOC_ControlApplyElectricalAngleDirect(motor, rs->elec_angle_rad);

        rs->settle_cycles = ReInit_MsToCycles(dt_sec, FOC_CALIB_COARSE_STEP_SETTLE_MS);
        rs->sin_sum = 0.0f;
        rs->cos_sum = 0.0f;
        rs->sample_count = 0U;
        rs->sample_target = FOC_CALIB_COARSE_STEP_SAMPLE_COUNT;

        rs->phase = FOC_REINIT_PHASE_DIR_SAMPLE;
        return 1U;
    }

    /* ========== DIR_SAMPLE: 在当前电角度位置采样机械角度 ========== */
    case FOC_REINIT_PHASE_DIR_SAMPLE:
    {
        motor->uq = 0.0f;
        motor->ud = rs->calib_uq;
        FOC_ControlApplyElectricalAngleDirect(motor, rs->elec_angle_rad);

        if (rs->settle_cycles > 0U)
        {
            rs->settle_cycles--;
        }
        else
        {
            (void)ReInit_SampleMechanicalAngle(rs);
        }

        if (rs->sample_count >= rs->sample_target)
        {
            float mech_rad_est = ReInit_AngleFromSum(rs);

            if (rs->has_prev != 0U)
            {
                float d_mech = Math_WrapRadDelta(mech_rad_est - rs->prev_mech_rad);
                float d_elec = rs->elec_angle_rad - rs->prev_elec_rad;

                if (fabsf(d_mech) >= FOC_CALIB_MIN_MECH_STEP_RAD)
                {
                    rs->sum_d_mech += d_mech;
                    rs->sum_d_elec += d_elec;
                }
            }

            rs->prev_mech_rad = mech_rad_est;
            rs->prev_elec_rad = rs->elec_angle_rad;
            rs->has_prev = 1U;

            rs->step_index++;
            if (rs->step_index > rs->step_count)
            {
                rs->phase = FOC_REINIT_PHASE_DIR_CALC;
            }
            else
            {
                rs->phase = FOC_REINIT_PHASE_DIR_STEP;
            }
        }
        return 1U;
    }

    /* ========== DIR_CALC: 计算方向 + 极对数 ========== */
    case FOC_REINIT_PHASE_DIR_CALC:
    {
        if ((fabsf(rs->sum_d_mech) >= FOC_CALIB_MIN_MECH_STEP_RAD) &&
            (fabsf(rs->sum_d_elec) >= 1e-6f))
        {
            int8_t dir = (rs->sum_d_mech >= 0.0f) ? FOC_DIR_NORMAL : FOC_DIR_REVERSED;
            uint8_t poles = (uint8_t)((uint32_t)(fabsf(rs->sum_d_elec / rs->sum_d_mech) + 0.5f));
            if (poles < 1U) poles = 1U;
            if (poles > 32U) poles = 32U;

            motor->direction = dir;
            motor->pole_pairs = poles;
        }

        ReInit_WriteLog("ReInit: dir/pole estimated\r\n");

        /* 反向扫描确认（与阻塞标定一致） */
        rs->step_index = rs->step_count;
        rs->reverse_pass = 0U;
        rs->phase = FOC_REINIT_PHASE_DIR_REV_STEP;
        return 1U;
    }

    /* ========== DIR_REV_STEP: 反向扫描步进 ========== */
    case FOC_REINIT_PHASE_DIR_REV_STEP:
    {
        if ((rs->step_index == 0U) && (rs->reverse_pass != 0U))
        {
            /* 反向扫描完成 */
            rs->phase = FOC_REINIT_PHASE_FINALIZE;
            return 1U;
        }

        if (rs->reverse_pass == 0U)
        {
            rs->reverse_pass = 1U;
        }
        rs->step_index--;

        rs->elec_angle_rad = FOC_CALIB_COARSE_STEP_ELEC_RAD * (float)rs->step_index;

        motor->uq = 0.0f;
        motor->ud = rs->calib_uq;
        FOC_ControlApplyElectricalAngleDirect(motor, rs->elec_angle_rad);

        rs->settle_cycles = ReInit_MsToCycles(dt_sec, FOC_CALIB_COARSE_STEP_SETTLE_MS);
        rs->sin_sum = 0.0f;
        rs->cos_sum = 0.0f;
        rs->sample_count = 0U;
        rs->sample_target = FOC_CALIB_COARSE_STEP_SAMPLE_COUNT;

        rs->phase = FOC_REINIT_PHASE_DIR_REV_SAMPLE;
        return 1U;
    }

    /* ========== DIR_REV_SAMPLE: 反向扫描采样 ========== */
    case FOC_REINIT_PHASE_DIR_REV_SAMPLE:
    {
        motor->uq = 0.0f;
        motor->ud = rs->calib_uq;
        FOC_ControlApplyElectricalAngleDirect(motor, rs->elec_angle_rad);

        if (rs->settle_cycles > 0U)
        {
            rs->settle_cycles--;
        }
        else
        {
            (void)ReInit_SampleMechanicalAngle(rs);
        }

        if (rs->sample_count >= rs->sample_target)
        {
            rs->phase = FOC_REINIT_PHASE_DIR_REV_STEP;
        }
        return 1U;
    }

    /* ========== FINALIZE: 完成 ========== */
    case FOC_REINIT_PHASE_FINALIZE:
    {
        /* 归零输出 */
        ReInit_ZeroOutput(motor, dt_sec);

        /* 应用配置 */
        FOC_Control_ApplyConfig(motor);

        motor->state.system_running = 1U;
        motor->state.current_loop_ready = 0U;
        motor->state.system_fault = 0U;
        motor->state.last_fault_code = (uint8_t)FOC_FAULT_NONE;

        {
            char info[120];
            snprintf(info, sizeof(info),
                     "reinit done: mech_zero=%.4f rad, dir=%d, poles=%d, vbus=%.2fV\r\n",
                     (double)motor->mech_angle_at_elec_zero_rad,
                     (int)motor->direction,
                     (int)motor->pole_pairs,
                     (double)motor->vbus_voltage);
            FOC_Platform_WriteDebugText(info);
        }

        rs->phase = FOC_REINIT_PHASE_DONE;
        return 1U;
    }

    /* ========== DONE: 切回 NORMAL ========== */
    case FOC_REINIT_PHASE_DONE:
    {
        rs->phase = FOC_REINIT_PHASE_IDLE;
        motor->state.control_phase = FOC_CONTROL_PHASE_NORMAL;
        return 0U;
    }

    default:
        rs->phase = FOC_REINIT_PHASE_IDLE;
        motor->state.control_phase = FOC_CONTROL_PHASE_NORMAL;
        return 0U;
    }
}

#endif /* FOC_REINIT_ENABLE */
