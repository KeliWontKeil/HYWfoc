#include "L3_Algorithm/foc_control_c22_current_loop.h"

#include <math.h>

#include "L3_Algorithm/foc_control_c31_actuation.h"
#include "L41_Math/math_transforms.h"
#include "LS_Config/foc_config.h"

#if ((FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE) && (FOC_CURRENT_LOOP_IQ_LPF_ENABLE == FOC_CFG_ENABLE))
static uint8_t g_current_iq_lpf_state_valid = 0U;
static float g_current_iq_lpf_state = 0.0f;
#endif

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
    if ((blend_initialized != 0U) && (*blend_initialized == 0U))
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

    /* 标准积分累加 */
    pid->integral += error * dt_sec;

    /* 导数直接基于实际误差 */
    derivative = (error - pid->prev_error) / dt_sec;
    pid->prev_error = error;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = Math_ClampFloat(output, pid->out_min, pid->out_max);

    /* 独立积分抗饱和：限制积分累积量不超过输出量程，
       避免 Kp*error 单独已饱和时 back-calculation 强行削零积分导致的阶梯现象 */
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

    voltage_limit = Math_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    phase_resistance = fabsf(motor->phase_resistance);

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

    motor->uq = uq;
    motor->ud = ud;
    motor->iq_measured = iq_estimated;
}

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)

#if (FOC_SENSOR_PHASE_COUNT == 2U)
/* Aggregate accumulated samples and update drift offsets for two-phase sampling.
 * Called when one electrical cycle has elapsed. Does NOT include per-frame
 * sample accumulation — that is done by the caller each control cycle. */
static void FOC_EcycleUpdateTwoPhase(foc_motor_t *motor,
                                     const sensor_data_t *sensor)
{
    float avg_a;
    float avg_b;
    float alpha;
    uint16_t count;

    (void)sensor;
    count = motor->ecycle_sample_count;

    if (count > 0U)
    {
        avg_a = motor->ecycle_accum_a / (float)count;
        avg_b = motor->ecycle_accum_b / (float)count;

        alpha = Math_ClampFloat(FOC_ELEC_CYCLE_OFFSET_LPF_ALPHA, 0.0f, 1.0f);
        motor->ecycle_offset_dyn_a += alpha * (avg_a - motor->ecycle_offset_dyn_a);
        motor->ecycle_offset_dyn_b += alpha * (avg_b - motor->ecycle_offset_dyn_b);
        motor->ecycle_offset_valid = 1U;
    }

    motor->ecycle_accum_a = 0.0f;
    motor->ecycle_accum_b = 0.0f;
    motor->ecycle_sample_count = 0U;
    motor->ecycle_accu_mech_delta = 0.0f;
}

#elif (FOC_SENSOR_PHASE_COUNT == 3U)
/* Aggregate accumulated samples and update drift offsets for three-phase sampling. */
static void FOC_EcycleUpdateThreePhase(foc_motor_t *motor,
                                       const sensor_data_t *sensor)
{
    //reserve for future implementation if needed; currently the same as two-phase since all three phases are sampled directly
    return;
}

#endif /* FOC_SENSOR_PHASE_COUNT == 3U */

#endif /* FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE */

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
static void FOC_CurrentLoopComputeIqMeasured(const sensor_data_t *sensor,
                                             float electrical_angle,
                                             float *iq_out,
                                             foc_motor_t *motor)
{
    float ia_comp;
    float ib_comp;
    float ic_comp;
    float i_alpha;
    float i_beta;
    float id_measured;

    if ((sensor == 0) || (iq_out == 0) || (motor == 0))
    {
        return;
    }

    /* Apply electrical-cycle dynamic drift offset before Clarke/Park. */
#if (FOC_SENSOR_PHASE_COUNT == 2U)
    {
#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
        if (motor->ecycle_offset_valid != 0U)
        {
            ia_comp = sensor->current_a.output_value - motor->ecycle_offset_dyn_a;
            ib_comp = sensor->current_b.output_value - motor->ecycle_offset_dyn_b;
        }
        else
#endif
        {
            ia_comp = sensor->current_a.output_value;
            ib_comp = sensor->current_b.output_value;
        }
        ic_comp = -(ia_comp + ib_comp);
    }
#else
    {
#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
        if (motor->ecycle_offset_valid != 0U)
        {
            ia_comp = sensor->current_a.output_value - motor->ecycle_offset_dyn_a;
            ib_comp = sensor->current_b.output_value - motor->ecycle_offset_dyn_b;
            ic_comp = sensor->current_c.output_value - motor->ecycle_offset_dyn_c;
        }
        else
#endif
        {
            ia_comp = sensor->current_a.output_value;
            ib_comp = sensor->current_b.output_value;
            ic_comp = sensor->current_c.output_value;
        }
    }
#endif

    Math_ClarkeTransform(ia_comp, ib_comp, ic_comp, &i_alpha, &i_beta);

    Math_ParkTransform(i_alpha,
                       i_beta,
                       electrical_angle,
                       &id_measured,
                       iq_out);
    (void)id_measured;

#if (FOC_CURRENT_LOOP_IQ_LPF_ENABLE == FOC_CFG_ENABLE)
    *iq_out = Math_FirstOrderLpf(*iq_out,
                                  &g_current_iq_lpf_state,
                                  FOC_CURRENT_LOOP_IQ_LPF_ALPHA,
                                  &g_current_iq_lpf_state_valid);
#endif
}

static void FOC_CurrentControlClosedLoopStep(foc_motor_t *motor,
                                             foc_pid_t *current_pid,
                                             const sensor_data_t *sensor,
                                             float dt_sec)
{
    float iq_measured;
    float uq_cmd;
    float local_angle;

    local_angle = motor->electrical_phase_angle;
    FOC_CurrentLoopComputeIqMeasured(sensor,
                                     local_angle,
                                     &iq_measured,
                                     motor);

    uq_cmd = FOC_CurrentLoopPIDRun(current_pid, motor->iq_target, iq_measured, dt_sec);

    motor->iq_measured = iq_measured;

    motor->ud = 0.0f;
    motor->uq = uq_cmd;
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
                                             foc_pid_t *current_pid,
                                             const sensor_data_t *sensor,
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

    if ((motor == 0) || (current_pid == 0) || (sensor == 0))
    {
        return;
    }

    soft_switch_status = &motor->current_soft_switch_status;
    blend_initialized = &motor->current_soft_switch_blend_initialized;

    FOC_CurrentControlClosedLoopStep(motor,
                                     current_pid,
                                     sensor,
                                     dt_sec);
    closed_ud = motor->ud;
    closed_uq = motor->uq;
    closed_iq = motor->iq_measured;

    FOC_CurrentLoopEstimateOpenLoopResistanceModel(motor,
                                                   motor->iq_target,
                                                   0.0f,
                                                   &open_ud,
                                                   &open_uq,
                                                   &open_iq);

    active_mode = FOC_CurrentSoftSwitchResolveActiveMode(soft_switch_status, fabsf(motor->iq_target));
    target_blend = (active_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED) ? 1.0f : 0.0f;
    blend_factor = FOC_CurrentSoftSwitchUpdateBlend(soft_switch_status->blend_factor,
                                                    blend_initialized,
                                                    target_blend,
                                                    dt_sec);
    soft_switch_status->blend_factor = blend_factor;

    motor->ud = open_ud + (closed_ud - open_ud) * blend_factor;
    motor->uq = open_uq + (closed_uq - open_uq) * blend_factor;
    motor->iq_measured = open_iq + (closed_iq - open_iq) * blend_factor;
}
#endif
#endif

void FOC_CurrentControlStep(foc_motor_t *motor,
                            foc_pid_t *current_pid,
                            const sensor_data_t *sensor,
                            float electrical_angle,
                            float dt_sec)
{
    uint8_t *blend_initialized;
    float    local_angle;

    if (motor == 0)
    {
        return;
    }

    blend_initialized = &motor->current_soft_switch_blend_initialized;

    local_angle = Math_WrapRad(electrical_angle);
    motor->electrical_phase_angle = local_angle;
    dt_sec = FOC_NormalizeDt(dt_sec);

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)
    if ((sensor != 0) && (sensor->encoder_valid != 0U))
    {
        float mech_now = sensor->mech_angle_rad.output_value;
        float raw_delta;
        float delta_mech;

        raw_delta = mech_now - motor->ecycle_prev_mech_angle;
        if (raw_delta > FOC_MATH_PI)
        {
            raw_delta -= FOC_MATH_TWO_PI;
        }
        else if (raw_delta < -FOC_MATH_PI)
        {
            raw_delta += FOC_MATH_TWO_PI;
        }
        delta_mech = (raw_delta >= 0.0f) ? raw_delta : (-raw_delta);

        motor->ecycle_accu_mech_delta += delta_mech;

        if (motor->ecycle_accu_mech_delta * (float)motor->pole_pairs >= FOC_MATH_TWO_PI)
        {
#if (FOC_SENSOR_PHASE_COUNT == 2U)
            FOC_EcycleUpdateTwoPhase(motor, sensor);
#else
            FOC_EcycleUpdateThreePhase(motor, sensor);
#endif
        }

#if (FOC_SENSOR_PHASE_COUNT == 2U)
        motor->ecycle_accum_a += sensor->current_a.filtered_value;
        motor->ecycle_accum_b += sensor->current_b.filtered_value;
        motor->ecycle_sample_count++;
#else
        motor->ecycle_accum_a += sensor->current_a.filtered_value;
        motor->ecycle_accum_b += sensor->current_b.filtered_value;
        motor->ecycle_accum_c += sensor->current_c.filtered_value;
        motor->ecycle_sample_count++;
#endif
        motor->ecycle_prev_mech_angle = mech_now;
    }
#endif

#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    if ((sensor == 0) || (current_pid == 0))
    {
        return;
    }

#if (FOC_CURRENT_SOFT_SWITCH_ENABLE == FOC_CFG_ENABLE)
    if (motor->current_soft_switch_status.enabled != 0U)
    {
        FOC_CurrentControlSoftSwitchStep(motor,
                                         current_pid,
                                         sensor,
                                         dt_sec);
    }
    else
#endif
    {
        if (blend_initialized != 0)
        {
            *blend_initialized = 0U;
        }

        if (motor->current_soft_switch_status.configured_mode == FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED)
        {
            motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_CLOSED;
            motor->current_soft_switch_status.blend_factor = 1.0f;
            FOC_CurrentControlClosedLoopStep(motor,
                                             current_pid,
                                             sensor,
                                             dt_sec);
        }
        else
        {
            motor->current_soft_switch_status.active_mode = FOC_CURRENT_SOFT_SWITCH_MODE_OPEN;
            motor->current_soft_switch_status.blend_factor = 0.0f;
            FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->iq_target, 0.0f);
            FOC_CurrentLoopComputeIqMeasured(sensor,
                                             local_angle,
                                             &motor->iq_measured,
                                             motor);
        }
    }
#else
    (void)current_pid;
    (void)sensor;
    (void)blend_initialized;
    FOC_CurrentLoopApplyOpenLoopResistanceModel(motor, motor->iq_target, 0.0f);
#endif

    FOC_ControlApplyElectricalAngleRuntime(motor, local_angle);
}

void FOC_CurrentControlApplyElectricalAngleDirect(foc_motor_t *motor, float electrical_angle)
{
    if (motor == 0)
    {
        return;
    }

    FOC_ControlApplyElectricalAngleDirect(motor, electrical_angle);
}

void FOC_CurrentControlOpenLoopStep(foc_motor_t *motor,
                                    float voltage,
                                    float turn_speed,
                                    float dt_sec)
{
    if (motor == 0)
    {
        return;
    }

    dt_sec = FOC_NormalizeDt(dt_sec);
    motor->electrical_phase_angle = Math_WrapRad(
        motor->electrical_phase_angle +
        FOC_MATH_TWO_PI * turn_speed * motor->pole_pairs * dt_sec * motor->direction);

    motor->ud = 0.0f;
    motor->uq = Math_ClampFloat(voltage, -motor->set_voltage, motor->set_voltage);
    FOC_ControlApplyElectricalAngleRuntime(motor, motor->electrical_phase_angle);
}

uint8_t FOC_ControlRequiresCurrentSample(void)
{
#if (FOC_CURRENT_LOOP_PID_ENABLE == FOC_CFG_ENABLE)
    return 1U;
#else
    return 0U;
#endif
}