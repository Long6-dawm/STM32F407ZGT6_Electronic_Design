/**
 * @file zero_cross.c
 * @brief 基于上升沿过零点的频率与周期测量实现。
 */

#include "zero_cross.h"

/*
 * 使用线性插值求取更精确的过零位置。
 * 当 sample[i-1] 为负、sample[i] 为正时：
 *   t_cross = (i-1) + |sample[i-1]| / (|sample[i-1]| + |sample[i]|)
 */
static float interp_zero_cross(float32_t y_prev, float32_t y_curr)
{
    float denom = y_curr - y_prev;
    if (denom == 0.0f) return 0.0f;
    return -y_prev / denom;
}

int ZeroCross_Count(float32_t *input, int n)
{
    int count = 0;
    for (int i = 1; i < n; i++)
    {
        if (input[i - 1] <= 0 && input[i] > 0)
        {
            count++;
        }
    }
    return count;
}

float ZeroCross_Period(float32_t *input, int n, float fs)
{
    float first = -1.0f;
    float last = -1.0f;
    int count = 0;

    for (int i = 1; i < n; i++)
    {
        if (input[i - 1] <= 0 && input[i] > 0)
        {
            float frac = interp_zero_cross(input[i - 1], input[i]);
            float t = (float)(i - 1) + frac;

            if (first < 0.0f) first = t;
            last = t;
            count++;
        }
    }

    if (count < 2 || fs <= 0.0f) return 0.0f;

    /* 对全部完整周期取平均，减小单次过零点受噪声抖动的影响。 */
    float period_samples = (last - first) / (float)(count - 1);
    return period_samples / fs;
}

float ZeroCross_Freq(float32_t *input, int n, float fs)
{
    float t_crossings[128];
    int count = 0;
    int max_store = (int)(sizeof(t_crossings) / sizeof(t_crossings[0]));

    for (int i = 1; i < n && count < max_store; i++)
    {
        if (input[i - 1] <= 0 && input[i] > 0)
        {
            float frac = interp_zero_cross(input[i - 1], input[i]);
            t_crossings[count] = (float)(i - 1) + frac;
            count++;
        }
    }

    if (count < 2) return 0.0f;

    float total_time = (t_crossings[count - 1] - t_crossings[0]) / fs;
    int periods = count - 1;
    float avg_period = total_time / (float)periods;

    return 1.0f / avg_period;
}
