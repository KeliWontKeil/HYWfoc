#include "L2/Control/foc_sensor.h"

#include <math.h>

#include "L3/foc_math_transforms.h"
#include "L3/foc_platform_api.h"
#include "LS_Config/foc_config.h"

static sensor_data_t sensor_data;
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static uint8_t g_sensor_angle_lpf_state_valid = 0U;
static float g_sensor_angle_lpf_state = 0.0f;
#endif

static void Sensor_ReadADC(uint8_t use_fast_window);

static void Sensor_ReadEncoder(void);
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
static float Sensor_UpdateAngleLpf(float measurement);
#endif

/* 浼犳劅鍣ㄥ垵濮嬪寲锛氶厤缃甈WM棰戠巼銆佸崱灏旀浖婊ゆ尝鍣ㄥ弬鏁般€侀浂鐐规牎鍑?*/
void Sensor_Init(uint8_t pwm_freq_kHz,float adc_sample_offset_percent)
{
    FOC_Platform_SensorInputInit(pwm_freq_kHz);

    Kalman_Init(&sensor_data.current_a,
                FOC_SENSOR_KALMAN_CURRENT_A_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_A_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_A_INIT);
    Kalman_Init(&sensor_data.current_b,
                FOC_SENSOR_KALMAN_CURRENT_B_MEAS_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_EST_ERR,
                FOC_SENSOR_KALMAN_CURRENT_B_PROC_NOISE,
                FOC_SENSOR_KALMAN_CURRENT_B_INIT);
    Kalman_Init(&sensor_data.mech_angle_rad,
                FOC_SENSOR_KALMAN_ANGLE_MEAS_ERR,
                FOC_SENSOR_KALMAN_ANGLE_EST_ERR,
                FOC_SENSOR_KALMAN_ANGLE_PROC_NOISE,
                FOC_SENSOR_KALMAN_ANGLE_INIT);

    sensor_data.adc_valid = 0;
    sensor_data.encoder_valid = 0;

    Sensor_SetZeroOffset();
    Sensor_ADCSampleTimeOffset(adc_sample_offset_percent);

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
    g_sensor_angle_lpf_state = FOC_SENSOR_KALMAN_ANGLE_INIT;
    g_sensor_angle_lpf_state_valid = 0U;
#endif

}

/* 璁剧疆鐢垫祦闆剁偣鍋忕Щ锛氬闈欐鐘舵€佷笅鐨凙DC閲囨牱鍙栧钩鍧囧€间綔涓哄亸绉婚噺 */
void Sensor_SetZeroOffset(void)
{
    uint16_t i;
    uint16_t valid_samples = 0U;
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    float current_c = 0.0f;
#endif
    float sum_a = 0.0f;
    float sum_b = 0.0f;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    float sum_c = 0.0f;
#endif
    float avg_a;
    float avg_b;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    float avg_c;
#endif

    for (i = 0U; i < SENSOR_ZERO_CALIB_SAMPLES; i++)
    {
#if (FOC_SENSOR_PHASE_COUNT == 2U)
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
        sensor_data.current_a.zero_offset = 0.0f;
        sensor_data.current_b.zero_offset = 0.0f;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
        sensor_data.current_c.zero_offset = 0.0f;
#endif
        return;
    }

    avg_a = sum_a / (float)valid_samples;
    avg_b = sum_b / (float)valid_samples;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    avg_c = sum_c / (float)valid_samples;
#endif

    if ((fabsf(avg_a) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT) &&
        (fabsf(avg_b) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT))
    {
        sensor_data.current_a.zero_offset = avg_a;
        sensor_data.current_b.zero_offset = avg_b;
    }
    else
    {
        sensor_data.current_a.zero_offset = 0.0f;
        sensor_data.current_b.zero_offset = 0.0f;
    }

#if (FOC_SENSOR_PHASE_COUNT == 3U)
    if (fabsf(avg_c) <= SENSOR_ZERO_CALIB_MAX_ABS_CURRENT)
    {
        sensor_data.current_c.zero_offset = avg_c;
    }
    else
    {
        sensor_data.current_c.zero_offset = 0.0f;
    }
#endif

}

/* 璇诲彇鎵€鏈変紶鎰熷櫒鏁版嵁锛圓DC+缂栫爜鍣?VBUS锛?*/
void Sensor_ReadAll(void)
{
    Sensor_ReadADC(0U);
    Sensor_ReadEncoder();
    Sensor_ReadVBUS();
}

/* 浠呰鍙栫數娴佷紶鎰熷櫒锛堢敤浜庡揩閫熺數娴佺幆ISR璺緞锛?*/
void Sensor_ReadCurrentOnly(void)
{
    Sensor_ReadADC(1U);
    //sensor_data.encoder_valid = 0;
}

/* 璇诲彇ADC鐢垫祦閲囨牱锛屼娇鐢ㄥ揩閫熺獥鍙ｆ垨鏅€氱獥鍙?*/
static void Sensor_ReadADC(uint8_t use_fast_window)
{
    float current_a = 0.0f;
    float current_b = 0.0f;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
    float current_c = 0.0f;
#endif
    uint8_t read_ok;

    if (use_fast_window != 0U)
    {
#if (FOC_SENSOR_PHASE_COUNT == 2U)
        read_ok = FOC_Platform_ReadPhaseCurrentFast(&current_a, &current_b, 0);
#else
        read_ok = FOC_Platform_ReadPhaseCurrentFast(&current_a, &current_b, &current_c);
#endif
    }
    else
    {
#if (FOC_SENSOR_PHASE_COUNT == 2U)
        read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, 0);
#else
        read_ok = FOC_Platform_ReadPhaseCurrent(&current_a, &current_b, &current_c);
#endif
    }

    if (read_ok != 0U)
    {
#if (FOC_SENSOR_KALMAN_CURRENT_ENABLE == FOC_CFG_ENABLE)
        Kalman_Update(&sensor_data.current_a, current_a);
        Kalman_Update(&sensor_data.current_b, current_b);
#if (FOC_SENSOR_PHASE_COUNT == 3U)
        Kalman_Update(&sensor_data.current_c, current_c);
#endif
#else
        sensor_data.current_a.raw_value = current_a;
        sensor_data.current_a.filtered_value = current_a;
        sensor_data.current_b.raw_value = current_b;
        sensor_data.current_b.filtered_value = current_b;
#if (FOC_SENSOR_PHASE_COUNT == 3U)
        sensor_data.current_c.raw_value = current_c;
        sensor_data.current_c.filtered_value = current_c;
#endif
#endif

        /* 搴旂敤闈欐€侀浂鐐瑰亸绉讳慨姝ｃ€?
         * 涓ょ浉閲囨牱鏃讹紝C鐩哥敱 -(Ia+Ib) 閲嶆瀯锛?
         * 涓夌浉閲囨牱鏃讹紝鍚勭浉鐩存帴璇诲彇銆?*/
#if (FOC_SENSOR_PHASE_COUNT == 2U)
        sensor_data.current_a.output_value = sensor_data.current_a.filtered_value - sensor_data.current_a.zero_offset;
        sensor_data.current_b.output_value = sensor_data.current_b.filtered_value - sensor_data.current_b.zero_offset;
        sensor_data.current_c.output_value = -(sensor_data.current_a.output_value + sensor_data.current_b.output_value);
#else
        sensor_data.current_a.output_value = sensor_data.current_a.filtered_value - sensor_data.current_a.zero_offset;
        sensor_data.current_b.output_value = sensor_data.current_b.filtered_value - sensor_data.current_b.zero_offset;
        sensor_data.current_c.output_value = sensor_data.current_c.filtered_value - sensor_data.current_c.zero_offset;
#endif

        sensor_data.adc_valid = 1;
    }
    else
    {
        sensor_data.adc_valid = 0;
    }
}

/* 璇诲彇缂栫爜鍣ㄦ満姊拌搴︼紝搴旂敤鍗″皵鏇兼护娉㈠拰浣庨€氭护娉?*/
static void Sensor_ReadEncoder(void)
{
    float angle_rad;
    float angle_for_output;

    if (FOC_Platform_ReadMechanicalAngleRad(&angle_rad) != 0U)
    {
#if (FOC_SENSOR_KALMAN_ANGLE_ENABLE == FOC_CFG_ENABLE)
        Kalman_Update_Angle(&sensor_data.mech_angle_rad, angle_rad);
        angle_for_output = sensor_data.mech_angle_rad.filtered_value;
#else
        sensor_data.mech_angle_rad.raw_value = angle_rad;
        sensor_data.mech_angle_rad.filtered_value = Math_WrapRad(angle_rad);
        angle_for_output = sensor_data.mech_angle_rad.filtered_value;
#endif

#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
        angle_for_output = Sensor_UpdateAngleLpf(angle_for_output);
#endif

        sensor_data.mech_angle_rad.output_value = angle_for_output;
        sensor_data.encoder_valid = 1;
    }
    else
    {
        sensor_data.encoder_valid = 0;
    }
}

/* 璇诲彇VBUS鐢靛帇锛屼竴闃朵綆閫氭护娉?*/
void Sensor_ReadVBUS(void)
{
    float vbus_raw;

    if (FOC_Platform_ReadVbusVoltage(&vbus_raw) != 0U)
    {
        sensor_data.vbus_voltage_raw = vbus_raw;
        if(sensor_data.vbus_valid == 0U)
        {
            sensor_data.vbus_voltage_filtered = vbus_raw;
            sensor_data.vbus_valid = 1U;
        }        

        sensor_data.vbus_voltage_filtered = Math_FirstOrderLpf(vbus_raw,
                                                                &sensor_data.vbus_voltage_filtered,
                                                                0.1F,
                                                                &sensor_data.vbus_valid);
    }
}

/* 鎷疯礉浼犳劅鍣ㄦ暟鎹埌澶栭儴缂撳啿鍖猴紙鐢ㄤ簬蹇収锛?*/
void Sensor_CopyData(sensor_data_t *out_data)
{
    *out_data = sensor_data;
}

/* 璁剧疆ADC閲囨牱鏃堕棿鍋忕Щ鐧惧垎姣?*/
void Sensor_ADCSampleTimeOffset(float percent)
{
    FOC_Platform_SetSensorSampleOffsetPercent(percent);
}

/* 鍗″皵鏇兼护娉㈠櫒鍒濆鍖?*/
static void Kalman_Init(kalman_filter_t* filter, float measurement_error, float estimate_error, float process_noise, float initial_value)
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

/* 鍗″皵鏇兼护娉㈠櫒鏇存柊锛堟爣鍑嗕竴缁村崱灏旀浖锛?*/
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
        filter->output_value = filter->filtered_value - filter->zero_offset;
        return;
    }

    filter->kalman_gain = filter->estimate_error / denominator;
    filter->filtered_value = filter->filtered_value + filter->kalman_gain * (measurement - filter->filtered_value);
    filter->estimate_error = (1.0f - filter->kalman_gain) * filter->estimate_error;
    filter->output_value = filter->filtered_value - filter->zero_offset;
}
#endif

/* 鍗″皵鏇兼护娉㈠櫒鏇存柊锛堣搴︾増鏈紝澶勭悊0/2pi缂犵粫锛?*/
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

    /* 閫夋嫨鏈€杩戠瓑鏁堣搴︼紝閬垮厤0/2pi缂犵粫涓嶈繛缁?*/
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

/* 瑙掑害涓€闃朵綆閫氭护娉?*/
#if (FOC_SENSOR_ANGLE_LPF_ENABLE == FOC_CFG_ENABLE)
static float Sensor_UpdateAngleLpf(float measurement)
{
    float alpha;
    float wrapped_measurement;
    float delta;

    alpha = Math_ClampFloat(FOC_SENSOR_ANGLE_LPF_ALPHA, 0.0f, 1.0f);
    wrapped_measurement = Math_WrapRad(measurement);

    if (g_sensor_angle_lpf_state_valid == 0U)
    {
        g_sensor_angle_lpf_state = wrapped_measurement;
        g_sensor_angle_lpf_state_valid = 1U;
        return g_sensor_angle_lpf_state;
    }

    delta = Math_WrapRadDelta(wrapped_measurement - g_sensor_angle_lpf_state);
    g_sensor_angle_lpf_state = Math_WrapRad(g_sensor_angle_lpf_state + alpha * delta);

    return g_sensor_angle_lpf_state;
}
#endif

/* 鑾峰彇婊ゆ尝鍚嶸BUS鐢靛帇 */
float Sensor_GetVBUSVoltage(void)
{
    return sensor_data.vbus_voltage_filtered;
}