#include "L2_Core/Control/foc_ctrl_current_loop.h"

#include <math.h>

#include "L3_Hal/foc_math_transforms.h"
#include "LS_Config/foc_config.h"

static float FOC_NormalizeDt(float dt_sec)
{
    return (dt_sec > 0.0f) ? dt_sec : FOC_CONTROL_DT_SEC;
}

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

    if (pid == 0)
    {
        return 0.0f;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);

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

static void FOC_CurrentLoopEstimateOpenLoopResistanceModel(const foc_motor_t *motor,
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

    if ((motor == 0) || (ud_out == 0) || (uq_out == 0) || (iq_estimated_out == 0))
    {
        return;
    }

    voltage_limit = Math_ClampFloat(motor->ctrl.max_phase_voltage, 0.0f, motor->params.vbus_voltage);
    phase_resistance = fabsf(motor->params.phase_resistance);

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

static void FOC_CurrentLoopApplyOpenLoopResistanceModel(foc_motor_t *motor,
                                                        float iq_ref,
                                                        float id_ref)
{
    float ud = 0.0f;
    float uq = 0.0f;
    float iq_estimated = 0.0f;

    if (motor == 0)
    {
        return;
    }

    FOC_CurrentLoopEstimateOpenLoopResistanceModel(motor,
                                                   iq_ref,
                                                   id_ref,
                                                   &ud,
                                                   &uq,
                                                   &iq_estimated);

    motor->ctrl.uq = uq;
    motor->ctrl.ud = ud;
    motor->ctrl.iq_measured = iq_estimated;
}

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)

static void FOC_CurrentLoopComputeIqMeasured(const sensor_data_t *sensor,
                                             float electrical_angle,
                                             float *iq_out)
{
    float ia_comp;
    float ib_comp;
    float ic_comp;
    float i_alpha;
    float i_beta;
    float id_measured;

    if ((sensor == 0) || (iq_out == 0))
    {
        return;
    }

    /* Apply Clarke/Park transform. Static zero offsets already applied in sensor layer. */
#if (FOC_CURRENT_SENSE_PHASES == 2U)
    {
        ia_comp = sensor->current_a.output_value;
        ib_comp = sensor->current_b.output_value;
        ic_comp = -(ia_comp + ib_comp);
    }
#else
    {
        ia_comp = sensor->current_a.output_value;
        ib_comp = sensor->current_b.output_value;
        ic_comp = sensor->current_c.output_value;
    }
#endif

    Math_ClarkeTransform(ia_comp, ib_comp, ic_comp, &i_alpha, &i_beta);

    Math_ParkTransform(i_alpha,
                       i_beta,
                       electrical_angle,
                       &id_measured,
                       iq_out);
    (void)id_measured;

}

static void FOC_CurrentControlClosedLoopStep(foc_motor_t *motor,
                                              const sensor_data_t *sensor,
                                              float electrical_angle,
                                              float dt_sec)
{
    float iq_measured;
    float uq_cmd;

    FOC_CurrentLoopComputeIqMeasured(sensor,
                                     electrical_angle,
                                     &iq_measured);

    uq_cmd = FOC_CurrentLoopPIDRun(&motor->torque_current_pid, motor->ctrl.iq_target, iq_measured, dt_sec);

    motor->ctrl.iq_measured = iq_measured;

    motor->ctrl.ud = 0.0f;
    motor->ctrl.uq = uq_cmd;
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

static void FOC_CurrentControlSoftSwitchStep(foc_motor_t *motor,
                                             const sensor_data_t *sensor,
                                             float electrical_angle,
                                             float dt_sec)
{
    foc_current_soft_switch_status_t *soft_switch_status;
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

    if ((motor == 0) || (sensor == 0))
    {
        return;
    }

    soft_switch_status = &motor->current_soft_switch_status;
    blend_initialized = &motor->current_soft_switch_status.blend_initialized;

    /* Resolve active mode first, since the branch below depends on it. */
    active_mode = FOC_CurrentSoftSwitchResolveActiveMode(soft_switch_status,
                                                          fabsf(motor->ctrl.iq_target));

    if (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
    {
        /* CLOSED: run PID closed-loop current control. */
        FOC_CurrentControlClosedLoopStep(motor,
                                         sensor,
                                         electrical_angle,
                                         dt_sec);
        closed_ud = motor->ctrl.ud;
        closed_uq = motor->ctrl.uq;
        closed_iq = motor->ctrl.iq_measured;
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
            FOC_CurrentLoopComputeIqMeasured(sensor,
                                             electrical_angle,
                                             &closed_iq);
        }
#else
        (void)sensor;
#endif
    }

    /* OPEN→CLOSED transition: reset PID integral to avoid stale-state voltage bump. */
    if ((motor->current_soft_switch_status.prev_active_mode != 0xFFU) &&
        (motor->current_soft_switch_status.prev_active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_OPEN) &&
        (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED))
    {
        motor->torque_current_pid.integral = 0.0f;
        motor->torque_current_pid.prev_error = 0.0f;
    }
    motor->current_soft_switch_status.prev_active_mode = active_mode;

    /* Open-loop resistance model: always computed (OPEN mode uses it for uq output). */
    FOC_CurrentLoopEstimateOpenLoopResistanceModel(motor,
                                                   motor->ctrl.iq_target,
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
    motor->ctrl.ud = open_ud + (closed_ud - open_ud) * blend_factor;
    motor->ctrl.uq = open_uq + (closed_uq - open_uq) * blend_factor;
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    motor->ctrl.iq_measured = closed_iq;
#else
    motor->ctrl.iq_measured = open_iq + (closed_iq - open_iq) * blend_factor;
#endif
}
#endif
#endif

void FOC_CurrentControlStep(foc_motor_t *motor,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec)
{
    float local_angle;

    if (motor == 0)
    {
        return;
    }

    local_angle = Math_WrapRad(electrical_angle);
    dt_sec = FOC_NormalizeDt(dt_sec);

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    if (sensor == 0)
    {
        return;
    }

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (motor->current_soft_switch_status.enabled != 0U)
    {
        FOC_CurrentControlSoftSwitchStep(motor,
                                         sensor,
                                         local_angle,
                                         dt_sec);
    }
    else
    {
        if (sensor->adc_valid == 0U) return;
        motor->current_soft_switch_status.blend_initialized = 0U;
        if (motor->current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
        {
            motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
            motor->current_soft_switch_status.blend_factor = 1.0f;
            FOC_CurrentControlClosedLoopStep(motor, sensor, local_angle, dt_sec);
        }
        else
        {
            motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
            motor->current_soft_switch_status.blend_factor = 0.0f;
            FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->ctrl.iq_target, 0.0f);
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
            FOC_CurrentLoopComputeIqMeasured(sensor,
                                             local_angle,
                                             &motor->ctrl.iq_measured);
#endif
        }
    }
#else
    /* FOC_CURRENT_SOFT_SWITCH_ENABLE == DISABLE: always run closed-loop PID */
    if (sensor->adc_valid == 0U) return;
    FOC_CurrentControlClosedLoopStep(motor, sensor, local_angle, dt_sec);
#endif
#else
    (void)sensor;
    FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->ctrl.iq_target, 0.0f);
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
