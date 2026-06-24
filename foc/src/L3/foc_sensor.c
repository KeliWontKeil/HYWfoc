#include "L3/foc_sensor.h"

#include <math.h>

#include "L3/foc_math_transforms.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"

/* Forward declarations for internal helpers. */
static void Sensor_ReadEncoder(foc_motor_t *motor, sensor_data_t *out);
static void Sensor_ReadVBUS(sensor_data_t *out);
static void Kalman_Init(kalman_filter_t* filter,
                        float measurement_error,
                        float estimate_error,
                        float process_noise,
                        float initial_value);
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update(kalman_filter_t* filter, float measurement);
#endif
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update_Angle(kalman_filter_t* filter, float measurement);
#endif
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static float Sensor_UpdateAngleLpf(foc_motor_t *motor, float measurement);
#endif
static void ApplyZeroOffsets(sensor_data_t *out, const foc_motor_t *motor);

/*
 * Sensor_InitSnapshot: initialize Kalman/state fields in a caller-provided sensor_data_t.
 * Must be called once per snapshot buffer before first use.
 */
void Sensor_InitSnapshot(sensor_data_t *out)
{
    if (out == 0) return;

#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    Kalman_Init(&out->current_a,
                FOC_SENSOR_KALMAN_CURRENT_A_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_A_INIT);
    Kalman_Init(&out->current_b,
                FOC_SENSOR_KALMAN_CURRENT_B_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_B_INIT);
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    Kalman_Init(&out->current_c,
                FOC_SENSOR_KALMAN_CURRENT_A_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_A_INIT);
#endif
#else
    out->current_a.raw_value = 0.0f;
    out->current_a.filtered_value = 0.0f;
    out->current_a.output_value = 0.0f;
    out->current_b.raw_value = 0.0f;
    out->current_b.filtered_value = 0.0f;
    out->current_b.output_value = 0.0f;
    out->current_c.raw_value = 0.0f;
    out->current_c.filtered_value = 0.0f;
    out->current_c.output_value = 0.0f;
#endif

    Kalman_Init(&out->mech_angle_rad,
                FOC_SENSOR_KALMAN_ANGLE_MEAS_ERR,
                FOC_SENSOR_KALMAN_ANGLE_EST_ERR,
                FOC_SENSOR_KALMAN_ANGLE_PROC_NOISE,
                FOC_SENSOR_KALMAN_ANGLE_INIT);

    out->adc_valid = 0;
    out->encoder_valid = 0;
    out->vbus_valid = 0;
    out->vbus_voltage_raw = 0.0f;
    out->vbus_voltage_filtered = 0.0f;
}

/*
 * Sensor_Init: configure PWM frequency and sample timing.
 * Zero offset and angle LPF state live in the motor struct, not here.
 */
void Sensor_Init(uint8_t pwm_freq_kHz, float adc_sample_offset_percent)
{
    FOC_Platform_SensorInputInit(pwm_freq_kHz);
    Sensor_ADCSampleTimeOffset(adc_sample_offset_percent);
}

/*
 * Sensor_SetZeroOffset: sample stationary phase currents to obtain DC offset.
 * Result stored in motor->sensor_zero_offset_*.
 * When FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE, this is a no-op.
 */
void Sensor_SetZeroOffset(foc_motor_t *motor)
{
    uint16_t i;
    uint16_t valid_samples = 0U;
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float current_c = 0.0f;
#endif
    float sum_a = 0.0f;
    float sum_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float sum_c = 0.0f;
#endif
    float avg_a;
    float avg_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float avg_c;
#endif

    if (motor == 0) return;

#if (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
    motor->sensor_zero_offset_a = 0.0f;
    motor->sensor_zero_offset_b = 0.0f;
    return;
#endif

    for (i = 0U; i < SENSOR_ZERO_CALIB_SAMPLES; i++)
    {
#if (FOC_CURRENT_SENSE_PHASES == 2U)
        if (FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, 0) != 0U)
        {
            sum_a += current_a;
            sum_b += current_b;
            valid_samples++;
        }
#else
        if (FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, &current_c) != 0U)
        {
            sum_a += current_a;
            sum_b += current_b;
            sum_c += current_c;
            valid_samples++;
        }
#endif
        FOC_Platform_WaitMs(1U);
    }

    if (valid_samples < SENSOR_ZERO_CALIB_MIN_VALID_SAMPLES)
    {
        motor->sensor_zero_offset_a = 0.0f;
        motor->sensor_zero_offset_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        motor->sensor_zero_offset_c = 0.0f;
#endif
        return;
    }

    avg_a = sum_a / (float)valid_samples;
    avg_b = sum_b / (float)valid_samples;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    avg_c = sum_c / (float)valid_samples;
#endif

    if ((fabsf(avg_a) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT) &&
        (fabsf(avg_b) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT))
    {
        motor->sensor_zero_offset_a = avg_a;
        motor->sensor_zero_offset_b = avg_b;
    }
    else
    {
        motor->sensor_zero_offset_a = 0.0f;
        motor->sensor_zero_offset_b = 0.0f;
    }

#if (FOC_CURRENT_SENSE_PHASES == 3U)
    if (fabsf(avg_c) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
    {
        motor->sensor_zero_offset_c = avg_c;
    }
    else
    {
        motor->sensor_zero_offset_c = 0.0f;
    }
#endif
}

/* Apply zero offsets from motor to a snapshot's current output_value fields. */
static void ApplyZeroOffsets(sensor_data_t *out, const foc_motor_t *motor)
{
    out->current_a.output_value -= motor->sensor_zero_offset_a;
    out->current_b.output_value -= motor->sensor_zero_offset_b;
#if (FOC_CURRENT_SENSE_PHASES == 2U)
    out->current_c.output_value = -(out->current_a.output_value + out->current_b.output_value);
#else
    out->current_c.output_value -= motor->sensor_zero_offset_c;
#endif
}

/*
 * Sensor_ReadCurrentSlow: read phase currents with slow (control-cycle) averaging window.
 * Written to motor->sensor with zero-offset correction from motor.
 * When FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE, sets adc_valid = 1 but leaves
 * currents at zero (no hardware read).
 */
void Sensor_ReadCurrentSlow(foc_motor_t *motor)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float current_c = 0.0f;
#endif
    uint8_t read_ok;

    if (motor == 0) return;

#if (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
    motor->sensor.adc_valid = 1;
    return;
#endif

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, 0);
#else
    read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, &current_c);
#endif

    if (read_ok != 0U)
    {
        sensor_data_t *out = &motor->sensor;

#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
        Kalman_Update(&out->current_a, current_a);
        Kalman_Update(&out->current_b, current_b);
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        Kalman_Update(&out->current_c, current_c);
#endif
#else
        out->current_a.raw_value = current_a;
        out->current_a.filtered_value = current_a;
        out->current_b.raw_value = current_b;
        out->current_b.filtered_value = current_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c.raw_value = current_c;
        out->current_c.filtered_value = current_c;
#endif
#endif

        out->current_a.output_value = out->current_a.filtered_value;
        out->current_b.output_value = out->current_b.filtered_value;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c.output_value = out->current_c.filtered_value;
#endif
        ApplyZeroOffsets(out, motor);

        out->adc_valid = 1;
    }
    else
    {
        motor->sensor.adc_valid = 0;
    }
}

/*
 * Sensor_ReadCurrentFast: read phase currents with fast (current-loop ISR) averaging window.
 * Written to motor->sensor_fast with zero-offset correction from motor.
 * When FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE, sets adc_valid = 1 but leaves
 * currents at zero (no hardware read).
 */
void Sensor_ReadCurrentFast(foc_motor_t *motor)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float current_c = 0.0f;
#endif
    uint8_t read_ok;

    if (motor == 0) return;

#if (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
    motor->sensor_fast.adc_valid = 1;
    return;
#endif

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    read_ok = FOC_Platform_ReadPhaseCurrentFast(&current_a, &current_b, 0);
#else
    read_ok = FOC_Platform_ReadPhaseCurrentFast(&current_a, &current_b, &current_c);
#endif

    if (read_ok != 0U)
    {
        sensor_data_t *out = &motor->sensor_fast;

#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
        Kalman_Update(&out->current_a, current_a);
        Kalman_Update(&out->current_b, current_b);
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        Kalman_Update(&out->current_c, current_c);
#endif
#else
        out->current_a.raw_value = current_a;
        out->current_a.filtered_value = current_a;
        out->current_b.raw_value = current_b;
        out->current_b.filtered_value = current_b;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c.raw_value = current_c;
        out->current_c.filtered_value = current_c;
#endif
#endif

        out->current_a.output_value = out->current_a.filtered_value;
        out->current_b.output_value = out->current_b.filtered_value;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c.output_value = out->current_c.filtered_value;
#endif
        ApplyZeroOffsets(out, motor);

        out->adc_valid = 1;
    }
    else
    {
        motor->sensor_fast.adc_valid = 0;
    }
}

/* Read encoder mechanical angle, apply Kalman and LPF. */
static void Sensor_ReadEncoder(foc_motor_t *motor, sensor_data_t *out)
{
    float angle_rad;
    float angle_for_output;

    if (out == 0) return;

    if (FOC_Platform_ReadMechanicalAngleRad(&angle_rad) != 0U)
    {
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
        Kalman_Update_Angle(&out->mech_angle_rad, angle_rad);
        angle_for_output = out->mech_angle_rad.filtered_value;
#else
        out->mech_angle_rad.raw_value = angle_rad;
        out->mech_angle_rad.filtered_value = Math_WrapRad(angle_rad);
        angle_for_output = out->mech_angle_rad.filtered_value;
#endif

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
        angle_for_output = Sensor_UpdateAngleLpf(motor, angle_for_output);
#endif

        out->mech_angle_rad.output_value = angle_for_output;
        out->encoder_valid = 1;
    }
    else
    {
        out->encoder_valid = 0;
    }
}

/* Read VBUS voltage, first-order LPF. */
static void Sensor_ReadVBUS(sensor_data_t *out)
{
    float vbus_raw;

    if (out == 0) return;

    if (FOC_Platform_ReadVbusVoltage(&vbus_raw) != 0U)
    {
        out->vbus_voltage_raw = vbus_raw;
        if (out->vbus_valid == 0U)
        {
            out->vbus_voltage_filtered = vbus_raw;
            out->vbus_valid = 1U;
        }

        out->vbus_voltage_filtered = Math_FirstOrderLpf(vbus_raw,
                                                         &out->vbus_voltage_filtered,
                                                         0.1F,
                                                         &out->vbus_valid);
    }
}

/*
 * Sensor_ReadAll: read all sensors (current slow + encoder + VBUS) into motor->sensor.
 */
void Sensor_ReadAll(foc_motor_t *motor)
{
    if (motor == 0) return;

    Sensor_ReadCurrentSlow(motor);
    Sensor_ReadEncoder(motor, &motor->sensor);
    Sensor_ReadVBUS(&motor->sensor);
}

/* Set ADC sample time offset percentage. */
void Sensor_ADCSampleTimeOffset(float percent)
{
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    FOC_Platform_SetSensorSampleOffsetPercent(percent);
#else
    (void)percent;
#endif
}

/* Kalman filter initialization. */
static void Kalman_Init(kalman_filter_t* filter,
                        float measurement_error,
                        float estimate_error,
                        float process_noise,
                        float initial_value)
{
    filter->raw_value = initial_value;
    filter->filtered_value = initial_value;
    filter->kalman_gain = 0.9f;
    filter->estimate_error = estimate_error;
    filter->measurement_error = measurement_error;
    filter->process_noise = process_noise;
    filter->zero_offset = 0.0f;
    filter->output_value = initial_value;
}

/* Kalman filter update (standard 1-D Kalman). */
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update(kalman_filter_t* filter, float measurement)
{
    float denominator;

    filter->raw_value = measurement;
    filter->estimate_error += filter->process_noise;

    denominator = filter->estimate_error + filter->measurement_error;
    if (denominator < 1e-6f)
    {
        filter->filtered_value = measurement;
        filter->kalman_gain = 1.0f;
        filter->estimate_error = 0.0f;
        filter->output_value = filter->filtered_value;
        return;
    }

    filter->kalman_gain = filter->estimate_error / denominator;
    filter->filtered_value = filter->filtered_value + filter->kalman_gain * (measurement - filter->filtered_value);
    filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;
    filter->output_value = filter->filtered_value;
}
#endif

/* Kalman filter update for angle (handles 0/2pi wrap). */
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
static void Kalman_Update_Angle(kalman_filter_t* filter, float measurement)
{
    float denominator;
    float err_direct;
    float err_plus_turn;
    float err_minus_turn;
    float selected_measurement;

    if (filter == 0)
    {
        return;
    }

    err_direct = fabsf(measurement - filter->filtered_value);
    err_plus_turn = fabsf((measurement + FOC_MATH_TWO_PI) - filter->filtered_value);
    err_minus_turn = fabsf((measurement - FOC_MATH_TWO_PI) - filter->filtered_value);

    selected_measurement = measurement;
    if ((err_plus_turn < err_direct) && (err_plus_turn <= err_minus_turn))
    {
        selected_measurement = measurement + FOC_MATH_TWO_PI;
    }
    else if ((err_minus_turn < err_direct) && (err_minus_turn < err_plus_turn))
    {
        selected_measurement = measurement - FOC_MATH_TWO_PI;
    }

    filter->estimate_error += filter->process_noise;
    denominator = filter->estimate_error + filter->measurement_error;

    if (denominator < 1e-6f)
    {
        filter->filtered_value = selected_measurement;
        filter->kalman_gain = 1.0f;
        filter->estimate_error = 0.0f;
    }
    else
    {
        filter->kalman_gain = filter->estimate_error / denominator;
        filter->filtered_value = filter->filtered_value +
                                 filter->kalman_gain * (selected_measurement - filter->filtered_value);
        filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;
    }

    filter->filtered_value = Math_WrapRad(filter->filtered_value);
    filter->output_value = filter->filtered_value;
}
#endif

/* Angle first-order LPF, state in motor struct (per-motor). */
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static float Sensor_UpdateAngleLpf(foc_motor_t *motor, float measurement)
{
    float alpha;
    float wrapped_measurement;
    float delta;

    alpha = Math_ClampFloat(FOC_SENSOR_ANGLE_LPF_ALPHA, 0.0f, 1.0f);
    wrapped_measurement = Math_WrapRad(measurement);

    if (motor->sensor_angle_lpf_valid == 0U)
    {
        motor->sensor_angle_lpf_state = wrapped_measurement;
        motor->sensor_angle_lpf_valid = 1U;
        return motor->sensor_angle_lpf_state;
    }

    delta = Math_WrapRadDelta(wrapped_measurement - motor->sensor_angle_lpf_state);
    motor->sensor_angle_lpf_state = Math_WrapRad(motor->sensor_angle_lpf_state + alpha * delta);

    return motor->sensor_angle_lpf_state;
}
#endif

/* ========== Electrical-cycle drift offset accumulation ========== */

#if (FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_ENABLE)

#if (FOC_CURRENT_SENSE_PHASES == 2U)
static void Sensor_EcycleUpdateTwoPhase(foc_motor_t *motor,
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

#elif (FOC_CURRENT_SENSE_PHASES == 3U)
static void Sensor_EcycleUpdateThreePhase(foc_motor_t *motor,
                                          const sensor_data_t *sensor)
{
    (void)motor;
    (void)sensor;
}
#endif /* FOC_CURRENT_SENSE_PHASES == 3U */

void Sensor_AccumulateEcycle(foc_motor_t *motor, const sensor_data_t *current_snapshot)
{
    float mech_now;
    float raw_delta;
    float delta_mech;

    if (motor == 0) return;
    if (current_snapshot == 0) return;

    /* Use motor->sensor.encoder_valid (updated by control-cycle Sensor_ReadAll)
     * to determine whether the motor is rotating.  The current snapshot (typically
     * motor->sensor_fast) does not carry encoder state. */
    if (motor->sensor.encoder_valid == 0U) return;

    mech_now = motor->sensor.mech_angle_rad.output_value;

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
#if (FOC_CURRENT_SENSE_PHASES == 2U)
        Sensor_EcycleUpdateTwoPhase(motor, current_snapshot);
#else
        Sensor_EcycleUpdateThreePhase(motor, current_snapshot);
#endif
    }

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    motor->ecycle_accum_a += current_snapshot->current_a.filtered_value;
    motor->ecycle_accum_b += current_snapshot->current_b.filtered_value;
    motor->ecycle_sample_count++;
#else
    motor->ecycle_accum_a += current_snapshot->current_a.filtered_value;
    motor->ecycle_accum_b += current_snapshot->current_b.filtered_value;
    motor->ecycle_accum_c += current_snapshot->current_c.filtered_value;
    motor->ecycle_sample_count++;
#endif
    motor->ecycle_prev_mech_angle = mech_now;
}

#else /* FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE == FOC_CFG_DISABLE */

void Sensor_AccumulateEcycle(foc_motor_t *motor, const sensor_data_t *current_snapshot)
{
    (void)motor;
    (void)current_snapshot;
}

#endif /* FOC_SENSOR_ELEC_CYCLE_OFFSET_ENABLE */

/* Get filtered VBUS voltage from a snapshot. */
float Sensor_GetVBUSVoltage(const sensor_data_t *snapshot)
{
    if (snapshot == 0) return 0.0f;
    return snapshot->vbus_voltage_filtered;
}

uint8_t Sensor_IsVBUSValid(const sensor_data_t *snapshot)
{
    if (snapshot == 0) return 0U;
    return snapshot->vbus_valid;
}
