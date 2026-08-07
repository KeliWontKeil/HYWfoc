#include "L3_Hal/foc_sensor.h"

#include <math.h>

#include "L3_Hal/foc_filter_gate.h"
#include "L3_Hal/foc_math_transforms.h"
#include "L3_Hal/foc_platform_api.h"
#include "LS_Config/foc_config.h"

static void ApplyZeroOffsets(sensor_data_t *out)
{
    out->current_a.output_value -= out->current_a_zero_offset;
    out->current_b.output_value -= out->current_b_zero_offset;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    out->current_c.output_value -= out->current_c_zero_offset;
#endif
}

void Sensor_InitSnapshot(sensor_data_t *out)
{
    if (out == 0) return;

#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
#if (FOC_FILTER_SENSOR_CURRENT_A == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&out->current_a,
                              FOC_FILTER_SENSOR_CURRENT_A_KALMAN_MEAS_ERR,
                              FOC_FILTER_SENSOR_CURRENT_A_KALMAN_EST_ERR,
                              FOC_FILTER_SENSOR_CURRENT_A_KALMAN_PROC_NOISE,
                              FOC_FILTER_SENSOR_CURRENT_A_KALMAN_INIT);
#elif (FOC_FILTER_SENSOR_CURRENT_A == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&out->current_a, 0.0f);
#else
    out->current_a.output_value = 0.0f;
#endif

#if (FOC_FILTER_SENSOR_CURRENT_B == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&out->current_b,
                              FOC_FILTER_SENSOR_CURRENT_B_KALMAN_MEAS_ERR,
                              FOC_FILTER_SENSOR_CURRENT_B_KALMAN_EST_ERR,
                              FOC_FILTER_SENSOR_CURRENT_B_KALMAN_PROC_NOISE,
                              FOC_FILTER_SENSOR_CURRENT_B_KALMAN_INIT);
#elif (FOC_FILTER_SENSOR_CURRENT_B == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&out->current_b, 0.0f);
#else
    out->current_b.output_value = 0.0f;
#endif

#if (FOC_CURRENT_SENSE_PHASES == 3U)
#if (FOC_FILTER_SENSOR_CURRENT_C == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&out->current_c,
                              FOC_FILTER_SENSOR_CURRENT_C_KALMAN_MEAS_ERR,
                              FOC_FILTER_SENSOR_CURRENT_C_KALMAN_EST_ERR,
                              FOC_FILTER_SENSOR_CURRENT_C_KALMAN_PROC_NOISE,
                              FOC_FILTER_SENSOR_CURRENT_C_KALMAN_INIT);
#elif (FOC_FILTER_SENSOR_CURRENT_C == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&out->current_c, 0.0f);
#else
    out->current_c.output_value = 0.0f;
#endif
#endif
#else
    out->current_a.output_value = 0.0f;
    out->current_b.output_value = 0.0f;
#endif

#if (FOC_FILTER_SENSOR_ANGLE == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanAngleInit(&out->mech_angle_rad,
                                   FOC_FILTER_SENSOR_ANGLE_KALMAN_MEAS_ERR,
                                   FOC_FILTER_SENSOR_ANGLE_KALMAN_EST_ERR,
                                   FOC_FILTER_SENSOR_ANGLE_KALMAN_PROC_NOISE,
                                   FOC_FILTER_SENSOR_ANGLE_KALMAN_INIT);
#elif (FOC_FILTER_SENSOR_ANGLE == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&out->mech_angle_rad, 0.0f);
#else
    out->mech_angle_rad.output_value = 0.0f;
#endif

#if (FOC_FILTER_ENCODER_SPEED == FOC_FILTER_TYPE_KALMAN)
    FOC_FilterMath_KalmanInit(&out->encoder_speed_filter,
                              FOC_FILTER_ENCODER_SPEED_KALMAN_MEAS_ERR,
                              FOC_FILTER_ENCODER_SPEED_KALMAN_EST_ERR,
                              FOC_FILTER_ENCODER_SPEED_KALMAN_PROC_NOISE,
                              FOC_FILTER_ENCODER_SPEED_KALMAN_INIT);
#elif (FOC_FILTER_ENCODER_SPEED == FOC_FILTER_TYPE_LPF1)
    FOC_FilterMath_Lpf1Init(&out->encoder_speed_filter, 0.0f);
#else
    out->encoder_speed_filter.output_value = 0.0f;
#endif

    out->adc_valid = 0;
    out->encoder_valid = 0;
    out->vbus_valid = 0;
    out->prev_mech_angle_rad = 0.0f;
    out->mech_speed_rad_s = 0.0f;
    out->mech_speed_valid = 0U;
    out->speed_window_pos = 0U;
    out->speed_window_count = 0U;
    out->vbus.raw = 0.0f;
    out->vbus.filtered = 0.0f;
    out->current_a_zero_offset = 0.0f;
    out->current_b_zero_offset = 0.0f;
}

void Sensor_Init(void)
{
    FOC_Platform_SensorInputInit();
    Sensor_ADCSampleTimeOffset(FOC_SENSOR_SAMPLE_OFFSET_PERCENT_DEFAULT);
}

void Sensor_SetZeroOffset(sensor_data_t *out)
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

    if (out == 0) return;

#if (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
    out->current_a_zero_offset = 0.0f;
    out->current_b_zero_offset = 0.0f;
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
        out->current_a_zero_offset = 0.0f;
        out->current_b_zero_offset = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c_zero_offset = 0.0f;
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
        out->current_a_zero_offset = avg_a;
        out->current_b_zero_offset = avg_b;
    }
    else
    {
        out->current_a_zero_offset = 0.0f;
        out->current_b_zero_offset = 0.0f;
    }

#if (FOC_CURRENT_SENSE_PHASES == 3U)
    if (fabsf(avg_c) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
    {
        out->current_c_zero_offset = avg_c;
    }
    else
    {
        out->current_c_zero_offset = 0.0f;
    }
#endif
}

void Sensor_ReadCurrent(sensor_data_t *out)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_CURRENT_SENSE_PHASES == 3U)
    float current_c = 0.0f;
#endif
    uint8_t read_ok;

    if (out == 0) return;

#if (FOC_CURRENT_SENSE_PHASES == FOC_CURRENT_SENSE_NONE)
    out->adc_valid = 1;
    return;
#endif

#if (FOC_CURRENT_SENSE_PHASES == 2U)
    read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, 0);
#else
    read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, &current_c);
#endif

    if (read_ok != 0U)
    {
        out->current_a.output_value = FOC_FilterGate_CurrentA(&out->current_a, current_a);
        out->current_b.output_value = FOC_FilterGate_CurrentB(&out->current_b, current_b);
#if (FOC_CURRENT_SENSE_PHASES == 3U)
        out->current_c.output_value = FOC_FilterGate_CurrentC(&out->current_c, current_c);
#endif

        ApplyZeroOffsets(out);

        out->adc_valid = 1;
    }
    else
    {
        out->adc_valid = 0;
    }
}

void Sensor_ReadEncoder(sensor_data_t *out, float dt_sec)
{
    float angle_rad;
    float angle_for_output;
    float delta;

    if (out == 0) return;

    if (FOC_Platform_ReadMechanicalAngleRad(&angle_rad) != 0U)
    {
        angle_for_output = FOC_FilterGate_Angle(&out->mech_angle_rad, angle_rad);
        if ((out->mech_speed_valid != 0U) && (dt_sec > 0.0f))
        {
            delta = Math_WrapRadDelta(angle_for_output - out->prev_mech_angle_rad);
            out->speed_window[out->speed_window_pos] = delta;
            out->speed_window_pos++;
            if (out->speed_window_pos >= FOC_ENCODER_SPEED_WINDOW_SIZE)
            {
                out->speed_window_pos = 0U;
            }
            if (out->speed_window_count < FOC_ENCODER_SPEED_WINDOW_SIZE)
            {
                out->speed_window_count++;
            }

            if (out->speed_window_count > 0U)
            {
                uint8_t i;
                float sum = 0.0f;
                for (i = 0U; i < out->speed_window_count; i++)
                {
                    sum += out->speed_window[i];
                }
                out->mech_speed_rad_s = sum / ((float)out->speed_window_count * dt_sec);
                out->mech_speed_rad_s = FOC_FilterGate_EncoderSpeed(&out->encoder_speed_filter, out->mech_speed_rad_s);
            }
        }
        else
        {
            out->speed_window_count = 0U;
            out->speed_window_pos = 0U;
        }
        out->prev_mech_angle_rad = angle_for_output;
        out->mech_speed_valid = 1U;
        out->mech_angle_rad.output_value = angle_for_output;
        out->encoder_valid = 1;
    }
    else
    {
        out->encoder_valid = 0;
    }
}

void Sensor_ReadVBUS(sensor_data_t *out)
{
    float vbus_raw;

    if (out == 0) return;

    if (FOC_Platform_ReadVbusVoltage(&vbus_raw) != 0U)
    {
        out->vbus.raw = vbus_raw;
        if (out->vbus_valid == 0U)
        {
            out->vbus.filtered = vbus_raw;
            out->vbus_valid = 1U;
        }

        out->vbus.filtered = Math_FirstOrderLpf(vbus_raw,
                                                  &out->vbus.filtered,
                                                  0.1F,
                                                  &out->vbus_valid);
    }
}

void Sensor_ADCSampleTimeOffset(float percent)
{
#if (FOC_CURRENT_SENSE_PHASES != FOC_CURRENT_SENSE_NONE)
    FOC_Platform_SetSensorSampleOffsetPercent(percent);
#else
    (void)percent;
#endif
}