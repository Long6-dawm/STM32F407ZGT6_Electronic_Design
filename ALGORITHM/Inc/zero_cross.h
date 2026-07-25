/**
 * @file zero_cross.h
 * @brief 基于上升沿过零点的频率与周期测量接口。
 */

#ifndef ZERO_CROSS_H
#define ZERO_CROSS_H

#include "arm_math.h"

/**
 * @brief 对线性插值后的过零周期取平均，估计信号频率。
 * @param[in] input 以零为中心的交流耦合采样缓冲区。
 * @param[in] n 采样点数。
 * @param[in] fs 采样频率，单位为 Hz。
 * @return 频率，单位为 Hz；有效过零点少于两个时返回 0。
 */
float ZeroCross_Freq(float32_t *input, int n, float fs);

/**
 * @brief 统计输入缓冲区中的上升沿过零次数。
 * @param[in] input 采样缓冲区。
 * @param[in] n 采样点数。
 * @return 上升沿过零点数量。
 */
int ZeroCross_Count(float32_t *input, int n);

/**
 * @brief 对全部完整的插值过零间隔取平均，估计信号周期。
 * @param[in] input 以零为中心的交流耦合采样缓冲区。
 * @param[in] n 采样点数。
 * @param[in] fs 采样频率，单位为 Hz。
 * @return 周期，单位为秒；输入或采样率无效时返回 0。
 */
float ZeroCross_Period(float32_t *input, int n, float fs);

#endif /* ZERO_CROSS_H */
