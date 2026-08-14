#include "L2_Core/Control/foc_ctrl_current_loop.h"

#include <math.h>

#include "L3_Hal/foc_math_lut.h"
#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
static float FOC_CurrentSoftSwitchUpdateBlend(float current_blend,
                                              uint8_t *blend_initialized,
                                              float target_blend,
                                              float dt_sec)
{
    float alpha;

    target_blend = Math_ClampFloat(target_blend, 0.0f, 1.0f);
    if ((blend_initialized != 0) && (*blend_initialized == 0U))
    {
        *blend_initialized = 1U;
        return target_blend;
    }

    alpha = dt_sec / (FOC_CURRENT_SOFT_SWITCH_BLEND_TAU_DEFAULT_SEC + dt_sec);
    alpha = Math_ClampFloat(alpha, 0.0f, 1.0f);

    current_blend += (target_blend - current_blend) * alpha;
    return Math_ClampFloat(current_blend, 0.0f, 1.0f);
}
#endif

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static float FOC_CurrentLoopPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    dt_sec = Math_NormalizeDt(dt_sec, FOC_CONTROL_DT_SEC);

    error = target - measurement;

    pid->integral += error * dt_sec;

    derivative = (error - pid->prev_error) / dt_sec;
    pid->prev_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_limit = (pid->out_max - pid->out_min) / pid->ki;
        pid->integral = Math_ClampFloat(pid->integral, -i_limit, i_limit);
    }
    else
    {
        pid->integral = 0.0f;
    }

    return output;
}
#endif

static void FOC_CurrentLoopEstimateOpenLoopResistanceModel(const foc_control_runtime_t *ctrl,
                                                           const foc_motor_params_t *params,
                                                           float iq_ref,
                                                           float id_ref,
                                                           float *ud_out,
                                                           float *uq_out,
                                                           float *iq_estimated_out)
{
    float voltage_limit;
    float phase_resistance;
    float current_limit;
    float iq_estimated;
    float id_estimated;

    if ((ctrl == 0) || (params == 0) || (ud_out == 0) || (uq_out == 0) || (iq_estimated_out == 0))
    {
        return;
    }

    voltage_limit = Math_ClampFloat(ctrl->max_phase_voltage, 0.0f, params->vbus_voltage);
    phase_resistance = fabsf(params->phase_resistance);

    if (phase_resistance < 1e-6f)
    {
        phase_resistance = 1e-6f;
    }

    current_limit = voltage_limit / phase_resistance;
    iq_estimated = Math_ClampFloat(iq_ref, -current_limit, current_limit);
    id_estimated = Math_ClampFloat(id_ref, -current_limit, current_limit);

    *uq_out = iq_estimated * phase_resistance;
    *ud_out = id_estimated * phase_resistance;
    *iq_estimated_out = iq_estimated;
}

static void FOC_CurrentLoopApplyOpenLoopResistanceModel(foc_control_runtime_t *ctrl,
                                                        const foc_motor_params_t *params,
                                                        float iq_ref,
                                                        float id_ref)
{
    float ud = 0.0f;
    float uq = 0.0f;
    float iq_estimated = 0.0f;

    FOC_CurrentLoopEstimateOpenLoopResistanceModel(ctrl,
                                                   params,
                                                   iq_ref,
                                                   id_ref,
                                                   &ud,
                                                   &uq,
                                                   &iq_estimated);

    ctrl->uq = uq;
    ctrl->ud = ud;
    ctrl->iq_measured = iq_estimated;
}

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)

static void FOC_CurrentLoopComputeIqMeasured(const foc_control_runtime_t *ctrl,
                                             float electrical_angle,
                                             float *iq_out)
{
    float id_measured;
    float sin_theta;
    float cos_theta;

    if ((ctrl == 0) || (iq_out == 0))
    {
        return;
    }

    /* Park 变换复用 executor 阶段1b 单点化的 αβ 结果（消除重复 Clarke）；
     * sin/cos 联合查表一次取得（消除重复三角函数查表）。 */
    FOC_MathLut_SinCos(electrical_angle, &sin_theta, &cos_theta);
    Math_ParkTransformSC(ctrl->ialpha,
                         ctrl->ibeta,
                         sin_theta,
                         cos_theta,
                         &id_measured,
                         iq_out);
    (void)id_measured;
}

static void FOC_CurrentControlClosedLoopStep(foc_control_runtime_t *ctrl,
                                              foc_pid_t *torque_pid,
                                              const sensor_data_t *sensor,
                                              float electrical_angle,
                                              float dt_sec)
{
    float iq_measured;
    float uq_cmd;

    (void)sensor;

    FOC_CurrentLoopComputeIqMeasured(ctrl,
                                     electrical_angle,
                                     &iq_measured);

    uq_cmd = FOC_CurrentLoopPIDRun(torque_pid, ctrl->iq_target, iq_measured, dt_sec);

    ctrl->iq_measured = iq_measured;

    ctrl->ud = 0.0f;
    ctrl->uq = uq_cmd;
}

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
static uint8_t FOC_CurrentSoftSwitchResolveActiveMode(foc_current_soft_switch_status_t *soft_switch_status,
                                                       float iq_ref_abs)
{
    float open_threshold;
    float closed_threshold;

    if (soft_switch_status->configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN)
    {
        soft_switch_status->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        return soft_switch_status->active_mode;
    }

    if (soft_switch_status->configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        soft_switch_status->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
        return soft_switch_status->active_mode;
    }

    open_threshold = Math_ClampFloat(soft_switch_status->auto_open_iq_a, 0.0f, 1e6f);
    closed_threshold = Math_ClampFloat(soft_switch_status->auto_closed_iq_a, 0.0f, 1e6f);

    if ((soft_switch_status->active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (soft_switch_status->active_mode != FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
    {
        soft_switch_status->active_mode =
            (iq_ref_abs >= closed_threshold) ? FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED : FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
        return soft_switch_status->active_mode;
    }

    if ((soft_switch_status->active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) &&
        (iq_ref_abs <= open_threshold))
    {
        soft_switch_status->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
    }
    else if ((soft_switch_status->active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
             (iq_ref_abs >= closed_threshold))
    {
        soft_switch_status->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
    }

    return soft_switch_status->active_mode;
}

static void FOC_CurrentControlSoftSwitchStep(foc_control_runtime_t *ctrl,
                                             foc_pid_t *torque_pid,
                                             foc_current_soft_switch_status_t *soft_switch_status,
                                             const sensor_data_t *sensor,
                                             const foc_motor_params_t *params,
                                             float electrical_angle,
                                             float dt_sec)
{
    uint8_t *blend_initialized;
    float open_ud = 0.0f;
    float open_uq = 0.0f;
    float open_iq = 0.0f;
    float closed_ud;
    float closed_uq;
    float closed_iq;
    float blend_factor;
    float target_blend;
    uint8_t active_mode;

    blend_initialized = &soft_switch_status->blend_initialized;

    /* Resolve active mode first, since the branch below depends on it. */
    active_mode = FOC_CurrentSoftSwitchResolveActiveMode(soft_switch_status,
                                                          fabsf(ctrl->iq_target));

    if (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        /* CLOSED: run PID closed-loop current control. */
        FOC_CurrentControlClosedLoopStep(ctrl,
                                         torque_pid,
                                         sensor,
                                         electrical_angle,
                                         dt_sec);
        closed_ud = ctrl->ud;
        closed_uq = ctrl->uq;
        closed_iq = ctrl->iq_measured;
    }
    else
    {
        /* OPEN: skip PID, only sample ADC for iq_measured reporting. */
        closed_ud = 0.0f;
        closed_uq = 0.0f;
        closed_iq = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
        if (sensor->adc_valid != 0U)
        {
            FOC_CurrentLoopComputeIqMeasured(ctrl,
                                             electrical_angle,
                                             &closed_iq);
        }
#else
        (void)sensor;
#endif
    }

    /* OPEN→CLOSED transition: reset PID integral to avoid stale-state voltage bump. */
    if ((soft_switch_status->prev_active_mode != 0xFFU) &&
        (soft_switch_status->prev_active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
    {
        torque_pid->integral = 0.0f;
        torque_pid->prev_error = 0.0f;
    }
    soft_switch_status->prev_active_mode = active_mode;

    /* Open-loop resistance model: always computed (OPEN mode uses it for uq output). */
    FOC_CurrentLoopEstimateOpenLoopResistanceModel(ctrl,
                                                   params,
                                                   ctrl->iq_target,
                                                   0.0f,
                                                   &open_ud,
                                                   &open_uq,
                                                   &open_iq);

    /* Update blender state. */
    target_blend = (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) ? 1.0f : 0.0f;
    blend_factor = FOC_CurrentSoftSwitchUpdateBlend(soft_switch_status->blend_factor,
                                                    blend_initialized,
                                                    target_blend,
                                                    dt_sec);
    soft_switch_status->blend_factor = blend_factor;

    /* Mix ud/uq for smooth OPEN↔CLOSED transition. */
    ctrl->ud = open_ud + (closed_ud - open_ud) * blend_factor;
    ctrl->uq = open_uq + (closed_uq - open_uq) * blend_factor;
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    ctrl->iq_measured = closed_iq;
#else
    ctrl->iq_measured = open_iq + (closed_iq - open_iq) * blend_factor;
#endif
}
#endif
#endif

void FOC_CurrentControlStep(foc_control_runtime_t *ctrl,
                            foc_pid_t *torque_pid,
                            foc_current_soft_switch_status_t *soft_switch,
                            const sensor_data_t *sensor,
                            const foc_motor_params_t *params,
                            float dt_sec)
{
    float local_angle;

    if ((ctrl == 0) || (params == 0))
    {
        return;
    }

    local_angle = Math_WrapRad(ctrl->electrical_angle_rad);
    dt_sec = Math_NormalizeDt(dt_sec, FOC_CONTROL_DT_SEC);

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    if (sensor == 0)
    {
        return;
    }

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (soft_switch->enabled != 0U)
    {
        FOC_CurrentControlSoftSwitchStep(ctrl,
                                         torque_pid,
                                         soft_switch,
                                         sensor,
                                         params,
                                         local_angle,
                                         dt_sec);
    }
    else
    {
        if (sensor->adc_valid == 0U) return;
        soft_switch->blend_initialized = 0U;
        if (soft_switch->configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
        {
            soft_switch->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
            soft_switch->blend_factor = 1.0f;
            FOC_CurrentControlClosedLoopStep(ctrl, torque_pid, sensor, local_angle, dt_sec);
        }
        else
        {
            soft_switch->active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
            soft_switch->blend_factor = 0.0f;
            FOC_CurrentLoopApplyOpenLoopResistanceModel(ctrl, params, ctrl->iq_target, 0.0f);
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
            FOC_CurrentLoopComputeIqMeasured(ctrl,
                                             local_angle,
                                             &ctrl->iq_measured);
#endif
        }
    }
#else
    /* FOC_CURRENT_SOFT_SWITCH_ENABLE == DISABLE: always run closed-loop PID */
    if (sensor->adc_valid == 0U) return;
    FOC_CurrentControlClosedLoopStep(ctrl, torque_pid, sensor, local_angle, dt_sec);
#endif
#else
    (void)sensor;
    (void)torque_pid;
    (void)soft_switch;
    FOC_CurrentLoopApplyOpenLoopResistanceModel(ctrl, params, ctrl->iq_target, 0.0f);
#endif
}

uint8_t FOC_ControlRequiresCurrentSample(void)
{
#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    return 1U;
#else
    return 0U;
#endif
}
