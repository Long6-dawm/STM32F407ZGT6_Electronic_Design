/**
 * @file fft_n.h
 * @brief 原位基 2 复数 FFT 接口。
 */

#ifndef FFT_N_H
#define FFT_N_H

#include <stdint.h>

#include "arm_math.h"

#define MAX_FFT_N 8192U

/** 旧版 FFT 实现使用的复数样本结构。 */
struct compx {
    float32_t real;
    float32_t imag;
};

/**
 * @brief 根据 FFT 长度初始化正弦、余弦查找表。
 * @param[in] n 需要初始化的查找表样本数。
 */
void InitTableFFT(uint32_t n);

/**
 * @brief 执行原位基 2 复数 FFT。
 * @param[in,out] _ptr 复数输入、输出共用缓冲区。
 * @param[in] FFT_N 变换长度；必须是 2 的幂且不超过 MAX_FFT_N。
 */
void cfft(struct compx *_ptr, uint32_t FFT_N);

/**
 * @brief 计算 floor(log2(n))。
 * @param[in] n 正整数输入。
 * @return 以 2 为底的指数；n 为 0 时返回 -1。
 */
int find_exponent(unsigned int n);

#endif /* FFT_N_H */
