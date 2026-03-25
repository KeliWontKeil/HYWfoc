#include "foc_control.h"

#define FOC_SPEED_ERR_ACCUM_LIMIT_RAD (FOC_TWO_PI * 4.0f)
#define FOC_SPEED_MECH_REBASE_RAD 2048.0f

static float g_speed_err_accum_rad = 0.0f;
static float g_prev_mech_signed_rad = 0.0f;
static uint8_t g_speed_state_valid = 0U;

static float FOC_WrapRad(float angle)
{
    while (angle >= FOC_TWO_PI)
    {
        angle -= FOC_TWO_PI;
    }
    while (angle < 0.0f)
    {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static float FOC_WrapRadDelta(float angle)
{
    while (angle > 3.1415926f)
    {
        angle -= FOC_TWO_PI;
    }
    while (angle < -3.1415926f)
    {
        angle += FOC_TWO_PI;
    }
    return angle;
}

static float FOC_ClampFloat(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }
    if (value > max_val)
    {
        return max_val;
    }
    return value;
}

static float FOC_PIDRunCore(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    error = target - measurement;
    pid->integral += error * dt_sec;
    derivative = (error - pid->prev_error) / dt_sec;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = FOC_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = FOC_ClampFloat(pid->integral, i_min, i_max);
    }

    pid->prev_error = error;
    return output;
}

static uint8_t FOC_ClampPolePairs(int32_t pole_pairs)
{
    if (pole_pairs < 1)
    {
        return 1U;
    }
    if (pole_pairs > 32)
    {
        return 32U;
    }
    return (uint8_t)pole_pairs;
}

static uint8_t FOC_ReadMechanicalAngleRad(float *angle_rad)
{
    return FOC_Platform_ReadMechanicalAngleRad(angle_rad);
}

static float FOC_MechanicalToElectricalAngle(foc_motor_t *motor, float mech_angle_rad)
{
    float mech_delta;
    float mech_delta_mod;

    if (motor == 0)
    {
        return 0.0f;
    }

    if ((motor->pole_pairs == FOC_POLE_PAIRS_UNDEFINED) ||
        (motor->mech_angle_at_elec_zero_rad == FOC_MECH_ANGLE_AT_ELEC_ZERO_UNDEFINED))
    {
        return motor->electrical_phase_angle;
    }

    mech_delta = FOC_WrapRadDelta(mech_angle_rad - motor->mech_angle_at_elec_zero_rad);

    mech_delta_mod = fmodf(mech_delta, FOC_TWO_PI / (float)motor->pole_pairs);
    if (mech_delta_mod < 0.0f)
    {
        mech_delta_mod += FOC_TWO_PI / (float)motor->pole_pairs;
    }

    return FOC_WrapRad( motor->direction * mech_delta_mod * (float)motor->pole_pairs);
}

static void FOC_UpdateAccumulatedMechanicalAngle(foc_motor_t *motor, float mech_angle_rad)
{
    float delta;

    if (motor == 0)
    {
        return;
    }

    if (motor->mech_angle_prev_valid == 0U)
    {
        motor->mech_angle_prev_rad = mech_angle_rad;
        motor->mech_angle_accum_rad = mech_angle_rad;
        motor->mech_angle_prev_valid = 1U;
        return;
    }

    delta = FOC_WrapRadDelta(mech_angle_rad - motor->mech_angle_prev_rad);
    if (fabsf(delta) >= FOC_MIN_MECH_ANGLE_ACCUM_DELTA_RAD)
    {
        motor->mech_angle_accum_rad += delta;
    }

    motor->mech_angle_prev_rad = mech_angle_rad;
}

static void FOC_ResetPIDState(foc_pid_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static void FOC_ResetSpeedState(void)
{
    g_speed_err_accum_rad = 0.0f;
    g_prev_mech_signed_rad = 0.0f;
    g_speed_state_valid = 0U;
}

static float FOC_UpdateSpeedAngleError(foc_motor_t *motor,
                                       float speed_ref_rad_s,
                                       float dt_sec)
{
    float mech_signed_rad;
    float mech_delta_rad;
    float speed_cmd_delta_rad;

    if (motor == 0)
    {
        return 0.0f;
    }

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    mech_signed_rad = motor->direction * motor->mech_angle_accum_rad;

    if (g_speed_state_valid == 0U)
    {
        g_prev_mech_signed_rad = mech_signed_rad;
        g_speed_err_accum_rad = 0.0f;
        g_speed_state_valid = 1U;
        return 0.0f;
    }

    if ((mech_signed_rad > FOC_SPEED_MECH_REBASE_RAD) ||
        (mech_signed_rad < -FOC_SPEED_MECH_REBASE_RAD))
    {
        g_prev_mech_signed_rad -= mech_signed_rad;
        mech_signed_rad = 0.0f;
    }

    mech_delta_rad = mech_signed_rad - g_prev_mech_signed_rad;
    g_prev_mech_signed_rad = mech_signed_rad;

    /* 1kHz loop: rad/s * dt(s) => rad increment per control cycle */
    speed_cmd_delta_rad = speed_ref_rad_s * dt_sec;
    g_speed_err_accum_rad += speed_cmd_delta_rad - mech_delta_rad;

    g_speed_err_accum_rad = FOC_ClampFloat(g_speed_err_accum_rad,
                                           -FOC_SPEED_ERR_ACCUM_LIMIT_RAD,
                                           FOC_SPEED_ERR_ACCUM_LIMIT_RAD);

    return g_speed_err_accum_rad;
}

void FOC_PIDInit(foc_pid_t *pid,
                 float kp,
                 float ki,
                 float kd,
                 float out_min,
                 float out_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float FOC_PIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    return FOC_PIDRunCore(pid, target, measurement, dt_sec);
}

float FOC_CurrentLoopPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    return FOC_PIDRunCore(pid, target, measurement, dt_sec);
}

static float FOC_AngleHoldPIDRun(foc_pid_t *pid, float target, float measurement, float dt_sec)
{
    float error;
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    error = target - measurement;
    if (fabsf(error) <= FOC_ANGLE_HOLD_PID_DEADBAND_RAD)
    {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        return 0.0f;
    }

    pid->integral += error * dt_sec;
    pid->integral = FOC_ClampFloat(pid->integral,
                                   -FOC_ANGLE_HOLD_INTEGRAL_LIMIT,
                                   FOC_ANGLE_HOLD_INTEGRAL_LIMIT);

    derivative = (error - pid->prev_error) / dt_sec;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = FOC_ClampFloat(output, pid->out_min, pid->out_max);

    if (pid->ki > 1e-6f)
    {
        float i_min = (pid->out_min - pid->kp * error - pid->kd * derivative) / pid->ki;
        float i_max = (pid->out_max - pid->kp * error - pid->kd * derivative) / pid->ki;
        pid->integral = FOC_ClampFloat(pid->integral, i_min, i_max);
        pid->integral = FOC_ClampFloat(pid->integral,
                                       -FOC_ANGLE_HOLD_INTEGRAL_LIMIT,
                                       FOC_ANGLE_HOLD_INTEGRAL_LIMIT);
    }

    pid->prev_error = error;
    return output;
}

void FOC_CurrentLoopStep(foc_motor_t *motor,
                         foc_pid_t *current_pid,
                         float iq_ref,
                         const sensor_data_t *sensor,
                         float electrical_angle,
                         float dt_sec)
{
    float i_alpha;
    float i_beta;
    float id_measured;
    float iq_measured;
    float voltage_limit;
    float uq_ff;
    float uq_pid;
    float uq_cmd;

    if ((motor == 0) || (current_pid == 0) || (sensor == 0))
    {
        return;
    }

    Math_ClarkeTransform(sensor->current_a.output_value,
                         sensor->current_b.output_value,
                         sensor->current_c.output_value,
                         &i_alpha,
                         &i_beta);

    Math_ParkTransform(i_alpha,
                       i_beta,
                       electrical_angle,
                       &id_measured,
                       &iq_measured);

    voltage_limit = FOC_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    uq_ff = iq_ref * motor->phase_resistance;
    uq_pid = 0.0f;

    uq_pid = FOC_CurrentLoopPIDRun(current_pid, iq_ref, iq_measured, dt_sec);

    uq_cmd = uq_ff + uq_pid;

    motor->electrical_phase_angle = FOC_WrapRad(electrical_angle);
    motor->ud = 0.0f;
    motor->uq = FOC_ClampFloat(uq_cmd, -voltage_limit, voltage_limit);
    motor->iq_target = iq_ref;
    motor->iq_measured = iq_measured;
}

static void FOC_ApplyElectricalAngle(foc_motor_t *motor, float electrical_angle)
{
    electrical_angle = FOC_WrapRad(electrical_angle);
    float dq_magnitude;
    float voltage_limit;
    float voltage_command;
    float ud_applied;
    float uq_applied;

    motor->electrical_phase_angle = electrical_angle;

    voltage_limit = FOC_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);
    dq_magnitude = sqrtf(motor->ud * motor->ud + motor->uq * motor->uq);
    ud_applied = motor->ud;
    uq_applied = motor->uq;

    if ((dq_magnitude > voltage_limit) && (dq_magnitude > 1e-6f))
    {
        float scale = voltage_limit / dq_magnitude;
        ud_applied *= scale;
        uq_applied *= scale;
        dq_magnitude = voltage_limit;
    }

    Math_InverseParkTransform(ud_applied,
                              uq_applied,
                              electrical_angle,
                              &motor->alpha,
                              &motor->beta);

    Math_InverseClarkeTransform(motor->alpha,
                                motor->beta,
                                &motor->phase_a,
                                &motor->phase_b,
                                &motor->phase_c);

    voltage_command = FOC_ClampFloat(dq_magnitude, 0.0f, voltage_limit);

    SVPWM_Update(motor->phase_a,
                 motor->phase_b,
                 motor->phase_c,
                 voltage_command,
                 motor->vbus_voltage,
                 &motor->sector,
                 &motor->duty_a,
                 &motor->duty_b,
                 &motor->duty_c);

    /* Duty update is executed by the TIMER2 callback via SVPWM linear interpolation. */
}

static uint8_t FOC_SampleLockedMechanicalAngle(foc_motor_t *motor,
                                               float electrical_angle,
                                               uint16_t settle_ms,
                                               uint16_t sample_count,
                                               float *mech_angle_rad)
{
    float sin_sum = 0.0f;
    float cos_sum = 0.0f;
    uint16_t i;

    if ((motor == 0) || (mech_angle_rad == 0) || (sample_count == 0U))
    {
        return 0U;
    }

    FOC_ApplyElectricalAngle(motor, electrical_angle);
    FOC_Platform_WaitMs(settle_ms);

    for (i = 0U; i < sample_count; i++)
    {
        float sample_rad;

        if (FOC_ReadMechanicalAngleRad(&sample_rad) == 0U)
        {
            continue;
        }

        sin_sum += sinf(sample_rad);
        cos_sum += cosf(sample_rad);
        FOC_Platform_WaitMs(FOC_CALIB_SETTLE_MS);
    }

    if ((fabsf(sin_sum) < 1e-6f) && (fabsf(cos_sum) < 1e-6f))
    {
        return 0U;
    }

    *mech_angle_rad = FOC_WrapRad(atan2f(sin_sum, cos_sum));
    return 1U;
}

static uint8_t FOC_EstimateDirectionAndPolePairs(foc_motor_t *motor,
                                                 int8_t *direction_est,
                                                 uint8_t *pole_pairs_est)
{
    float prev_mech_rad = 0.0f;
    float prev_elec_rad = 0.0f;
    float sum_d_mech = 0.0f;
    float sum_d_elec = 0.0f;
    uint8_t has_prev = 0U;
    uint16_t i;

    if ((motor == 0) || (direction_est == 0) || (pole_pairs_est == 0))
    {
        return 0U;
    }

    for (i = 0U; i <= FOC_CALIB_STEP_COUNT; i++)
    {
        float elec_target = FOC_CALIB_STEP_ELEC_RAD * (float)i;
        float mech_rad;

        if (FOC_SampleLockedMechanicalAngle(motor,
                                            elec_target,
                                            FOC_CALIB_STEP_SETTLE_MS,
                                            FOC_CALIB_STEP_SAMPLE_COUNT,
                                            &mech_rad) == 0U)
        {
            continue;
        }

        if (has_prev == 0U)
        {
            prev_mech_rad = mech_rad;
            prev_elec_rad = elec_target;
            has_prev = 1U;
            continue;
        }

        {
            float d_mech = FOC_WrapRadDelta(mech_rad - prev_mech_rad);
            float d_elec = elec_target - prev_elec_rad;

            if (fabsf(d_mech) >= FOC_CALIB_MIN_MECH_STEP_RAD)
            {
                sum_d_mech += d_mech;
                sum_d_elec += d_elec;
            }
        }

        prev_mech_rad = mech_rad;
        prev_elec_rad = elec_target;
    }

    if ((fabsf(sum_d_mech) < FOC_CALIB_MIN_MECH_STEP_RAD) || (fabsf(sum_d_elec) < 1e-6f))
    {
        return 0U;
    }

    *direction_est = (sum_d_mech >= 0.0f) ? FOC_DIR_NORMAL: FOC_DIR_REVERSED;
    *pole_pairs_est = FOC_ClampPolePairs((int32_t)(fabsf(sum_d_elec / sum_d_mech) + 0.5f));

    FOC_ApplyElectricalAngle(motor, 0.0f);
    FOC_Platform_WaitMs(FOC_CALIB_STEP_SETTLE_MS);

    for (i = FOC_CALIB_STEP_COUNT; i > 0; i--)
    {
        float elec_target = FOC_CALIB_STEP_ELEC_RAD * (float)i;
        float mech_rad;

        FOC_SampleLockedMechanicalAngle(motor,
                                        elec_target,
                                        FOC_CALIB_STEP_SETTLE_MS,
                                        FOC_CALIB_STEP_SAMPLE_COUNT,
                                         &mech_rad);
    }

    return 1U;
}

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

    calib_uq = motor->set_voltage * 0.20f;
    if (calib_uq < 0.5f)
    {
        calib_uq = 0.5f;
    }
    if (calib_uq > 2.5f)
    {
        calib_uq = 2.5f;
    }

    motor->uq = 0.0f;
    motor->ud = calib_uq;

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
            motor->direction = FOC_DIR_NORMAL;
         }
         if (need_pole_pairs != 0U)
         {
            motor->pole_pairs = 1U;
         }
    }

    if (need_zero != 0U)
    {
        if (FOC_SampleLockedMechanicalAngle(motor,
                                            0.0f,
                                            FOC_CALIB_LOCK_SETTLE_MS,
                                            FOC_CALIB_LOCK_SAMPLE_COUNT,
                                            &mech_zero_rad_est) != 0U)
        {
            motor->mech_angle_at_elec_zero_rad = mech_zero_rad_est;
            motor->mech_angle_accum_rad = mech_zero_rad_est;
            motor->mech_angle_prev_rad = mech_zero_rad_est;
            motor->mech_angle_prev_valid = 1U;
        }
        else
        {
            motor->mech_angle_at_elec_zero_rad = 0.0f;
            motor->mech_angle_accum_rad = 0.0f;
            motor->mech_angle_prev_rad = 0.0f;
            motor->mech_angle_prev_valid = 1U;
        }
    }

    motor->ud = backup_ud;
    motor->uq = backup_uq;
    FOC_ApplyElectricalAngle(motor, 0.0f);
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

    motor->electrical_phase_angle = 0.0f;
    motor->ud = 0.0f;
    motor->uq = 1.0f;
    motor->set_voltage = set_voltage;
    motor->vbus_voltage = vbus_voltage;
    motor->iq_target = 0.0f;
    motor->iq_measured = 0.0f;
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
    motor->duty_a = 0.5f;
    motor->duty_b = 0.5f;
    motor->duty_c = 0.5f;
    motor->sector = 0U;

    FOC_CalibrateElectricalAngleAndDirection(motor);
}

void FOC_OpenLoopStep(foc_motor_t *motor,float voltage, float turn_speed)
{
    float delta_theta;

    delta_theta = FOC_TWO_PI * turn_speed * motor->pole_pairs * 0.001f * motor->direction;
    motor->electrical_phase_angle = FOC_WrapRad(motor->electrical_phase_angle + delta_theta);

    motor->ud = 0.0f;
    motor->uq = FOC_ClampFloat(voltage, 0.0f, motor->set_voltage);
    
    FOC_ApplyElectricalAngle(motor, motor->electrical_phase_angle);
}

void FOC_TorqueControlStep(foc_motor_t *motor,
                           foc_pid_t *current_pid,
                           float torque_ref_current,
                           const sensor_data_t *sensor,
                           float dt_sec,
                           foc_torque_mode_t mode)
{
    float voltage_limit;
    float mech_angle_rad;

    if ((motor == 0) || (sensor == 0))
    {
        return;
    }

    mech_angle_rad = sensor->mech_angle_rad.output_value;

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    FOC_UpdateAccumulatedMechanicalAngle(motor, mech_angle_rad);

    motor->electrical_phase_angle = FOC_MechanicalToElectricalAngle(motor, mech_angle_rad);
    voltage_limit = FOC_ClampFloat(motor->set_voltage, 0.0f, motor->vbus_voltage);

    if ((mode == FOC_TORQUE_MODE_CURRENT_PID) && (current_pid != 0))
    {
        FOC_CurrentLoopStep(motor,
                            current_pid,
                            torque_ref_current,
                            sensor,
                            motor->electrical_phase_angle,
                            dt_sec);
    }
    else
    {
        float open_loop_uq = torque_ref_current * motor->phase_resistance;

        motor->uq = FOC_ClampFloat(open_loop_uq, -voltage_limit, voltage_limit);
        motor->ud = 0.0f;
    }

    FOC_ApplyElectricalAngle(motor, motor->electrical_phase_angle);
}

void FOC_SpeedControlStep(foc_motor_t *motor,
                          foc_pid_t *speed_pid,
                          foc_pid_t *current_pid,
                          float speed_ref_rad_s,
                          const sensor_data_t *sensor,
                          float dt_sec,
                          foc_torque_mode_t torque_mode)
{
    float torque_ref_current;
    float speed_angle_error_rad;
    float mech_angle_rad;

    if ((motor == 0) || (speed_pid == 0) || (sensor == 0))
    {
        return;
    }

    mech_angle_rad = sensor->mech_angle_rad.output_value;

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    FOC_UpdateAccumulatedMechanicalAngle(motor, mech_angle_rad);
    speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor, speed_ref_rad_s, dt_sec);

    torque_ref_current = FOC_PIDRunCore(speed_pid,
                                        speed_angle_error_rad,
                                        0.0f,
                                        dt_sec);

    FOC_TorqueControlStep(motor,
                          current_pid,
                          torque_ref_current,
                          sensor,
                          dt_sec,
                          torque_mode);
}

void FOC_SpeedAngleControlStep(foc_motor_t *motor,
                               foc_pid_t *speed_pid,
                               foc_pid_t *angle_hold_pid,
                               foc_pid_t *current_pid,
                               float angle_ref_rad,
                               float angle_position_speed_rad_s,
                               const sensor_data_t *sensor,
                               float dt_sec,
                               foc_torque_mode_t torque_mode)
{
    float torque_ref_current;
    float torque_ref_speed;
    float torque_ref_hold;
    float mech_angle_rad;
    float mech_signed_total_rad;
    float angle_error_rad;
    float abs_angle_error_rad;
    float transition_span_rad;
    float speed_blend;
    float speed_ref_rad_s;
    float speed_angle_error_rad;

    if ((motor == 0) || (speed_pid == 0) || (angle_hold_pid == 0) || (sensor == 0))
    {
        return;
    }

    mech_angle_rad = sensor->mech_angle_rad.output_value;

    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    FOC_UpdateAccumulatedMechanicalAngle(motor, mech_angle_rad);
    mech_signed_total_rad = motor->direction * motor->mech_angle_accum_rad;
    angle_error_rad = angle_ref_rad - mech_signed_total_rad;
    abs_angle_error_rad = fabsf(angle_error_rad);

    transition_span_rad = FOC_SPEED_ANGLE_TRANSITION_END_RAD - FOC_SPEED_ANGLE_TRANSITION_START_RAD;
    if (transition_span_rad < 1e-6f)
    {
        transition_span_rad = 1e-6f;
    }

    if (abs_angle_error_rad <= FOC_SPEED_ANGLE_TRANSITION_START_RAD)
    {
        speed_blend = 0.0f;
    }
    else if (abs_angle_error_rad >= FOC_SPEED_ANGLE_TRANSITION_END_RAD)
    {
        speed_blend = 1.0f;
    }
    else
    {
        speed_blend = (abs_angle_error_rad - FOC_SPEED_ANGLE_TRANSITION_START_RAD) / transition_span_rad;
    }

    speed_ref_rad_s = (angle_error_rad >= 0.0f) ? fabsf(angle_position_speed_rad_s) : -fabsf(angle_position_speed_rad_s);
    speed_ref_rad_s *= speed_blend;

    if (speed_blend < 1e-4f)
    {
        FOC_ResetPIDState(speed_pid);
        FOC_ResetSpeedState();
        torque_ref_speed = 0.0f;
    }
    else
    {
        speed_angle_error_rad = FOC_UpdateSpeedAngleError(motor, speed_ref_rad_s, dt_sec);
        torque_ref_speed = FOC_PIDRunCore(speed_pid,
                                          speed_angle_error_rad,
                                          0.0f,
                                          dt_sec);
    }

    torque_ref_hold = FOC_AngleHoldPIDRun(angle_hold_pid,
                                          angle_ref_rad,
                                          mech_signed_total_rad,
                                          dt_sec);

    torque_ref_current = (1.0f - speed_blend) * torque_ref_hold + speed_blend * torque_ref_speed;

    FOC_TorqueControlStep(motor,
                          current_pid,
                          torque_ref_current,
                          sensor,
                          dt_sec,
                          torque_mode);
}
