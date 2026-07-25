/**
 * @file fft_interp_freq.h
 * @brief FFT 峰值与对数抛物线插值测频接口。
 */

#ifndef FFT_INTERP_FREQ_H
#define FFT_INTERP_FREQ_H

#include <stdint.h>

#include "arm_math.h"

/**
 * @brief 从固定 8192 点 ADC 缓冲区估计信号频率。
 * @param[in] fs 采样频率，单位为 Hz。
 * @param[in] AD_Value ADC 采样缓冲区。
 * @param[in] flag 为 1 时使用对数抛物线峰值插值，为 2 时直接返回峰值频点。
 * @return 估计频率，单位为 Hz；flag 无效时返回 0。
 */
float cfft_f32_fre(float32_t fs, uint16_t *AD_Value, uint8_t flag);

#endif /* FFT_INTERP_FREQ_H */
