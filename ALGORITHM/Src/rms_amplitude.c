/**
 * @file rms_amplitude.c
 * @brief 基于有效值的波形幅度测量实现。
 */

#include "rms_amplitude.h"

#include <math.h>
#include <stddef.h>

#define ADC_LSB_VOLTS (3.3f / 4096.0f)

static float measure_amplitude(uint16_t length,
                               const uint16_t *ad_value,
                               float shape_factor)
{
    if (length == 0U || ad_value == NULL) {
        return 0.0f;
    }

    /* 第一步：计算 ADC 采样均值，即信号的直流分量。 */
    float mean = 0.0f;
    for (uint16_t i = 0U; i < length; ++i) {
        mean += (float)ad_value[i];
    }
    mean /= (float)length;

    /* 第二步：去直流后转换为电压值，累加平方和。 */
    float sum_squares = 0.0f;
    for (uint16_t i = 0U; i < length; ++i) {
        const float centered_volts = ((float)ad_value[i] - mean) * ADC_LSB_VOLTS;
        sum_squares += centered_volts * centered_volts;
    }

    /* 第三步：RMS = sqrt(波形系数 × 平方和 / 点数) 得到峰值幅度。 */
    return sqrtf(shape_factor * sum_squares / (float)length);
}

float Measuring_Sine_Amplitude(uint16_t length, uint16_t *AD_value)
{
    return measure_amplitude(length, AD_value, 2.0f);
}

float Measuring_Square_Amplitude(uint16_t length, uint16_t *AD_value)
{
    return measure_amplitude(length, AD_value, 1.0f);
}

float Measuring_Triangle_Amplitude(uint16_t length, uint16_t *AD_value)
{
    return measure_amplitude(length, AD_value, 3.0f);
}
