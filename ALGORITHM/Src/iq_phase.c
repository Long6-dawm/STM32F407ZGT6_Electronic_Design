/**
 * @file iq_phase.c
 * @brief 基于 I/Q 解调的相位测量实现。
 */

#include "iq_phase.h"
#include "fft_buffer.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <math.h>

void Create_data2handle(float32_t *p)
{
    /* 将 SAMPLE_N 个实数样本按实部-虚部交错排列，构造复数 FFT 输入缓冲区。 */
    for(int i = 0;i < SAMPLE_N; i++)
    {
        data2handle[2 * i]     = p[i];
        data2handle[2 * i + 1] = 0;
    }
}

void CalXiebo(float32_t *input, float32_t *output, int n)
{
    if (input == NULL || output == NULL || n != SAMPLE_N) {
        return;
    }

    /* 将实数输入复制为实部-虚部交错格式，供 CMSIS-DSP FFT 使用。 */
    float32_t temp[2 * SAMPLE_N];
    for(int i = 0; i < n; i++)
    {
        temp[2 * i]     = input[i];
        temp[2 * i + 1] = 0;
    }
    /* 调用 CMSIS-DSP 的 1024 点 CFFT 计算频谱。 */
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, temp, 0, 1);
    /* 将复数频谱转换为幅度谱并输出。 */
    arm_cmplx_mag_f32(temp, output, n);
}

float CalPhase(float f, float fs, int N, float32_t *adc_float)
{
    static const double k_pi =
        3.14159265358979323846264338327950288;
    double sum_y = 0.0;
    double sum_c = 0.0;
    double sum_s = 0.0;
    double sum_cc = 0.0;
    double sum_ss = 0.0;
    double sum_cs = 0.0;
    double sum_yc = 0.0;
    double sum_ys = 0.0;

    if (adc_float == NULL || f <= 0.0f || fs <= 0.0f || N <= 0 ||
        N > SAMPLE_N || f >= 0.5f * fs) {
        return 0.0f;
    }

    for (int i = 0; i < N; ++i) {
        const double angle = 2.0 * k_pi * (double)f * (double)i /
                             (double)fs;
        const double cosine = cos(angle);
        const double sine = sin(angle);
        const double sample = adc_float[i];

        sum_y += sample;
        sum_c += cosine;
        sum_s += sine;
        sum_cc += cosine * cosine;
        sum_ss += sine * sine;
        sum_cs += cosine * sine;
        sum_yc += sample * cosine;
        sum_ys += sample * sine;
    }

    const double sample_count = (double)N;
    const double centered_cc = sum_cc - sum_c * sum_c / sample_count;
    const double centered_ss = sum_ss - sum_s * sum_s / sample_count;
    const double centered_cs = sum_cs - sum_c * sum_s / sample_count;
    const double centered_yc = sum_yc - sum_y * sum_c / sample_count;
    const double centered_ys = sum_ys - sum_y * sum_s / sample_count;
    const double trace = centered_cc + centered_ss;
    const double discriminant = hypot(centered_cc - centered_ss,
                                      2.0 * centered_cs);
    const double lambda_max = 0.5 * (trace + discriminant);
    const double lambda_min = 0.5 * (trace - discriminant);
    const double determinant = centered_cc * centered_ss -
                               centered_cs * centered_cs;

    if (!isfinite(lambda_max) || !isfinite(lambda_min) ||
        lambda_max <= 0.0 || lambda_min <= 1.0e-12 * lambda_max ||
        !isfinite(determinant)) {
        return 0.0f;
    }

    const double cosine_coefficient =
        (centered_yc * centered_ss - centered_ys * centered_cs) /
        determinant;
    const double sine_coefficient =
        (centered_ys * centered_cc - centered_yc * centered_cs) /
        determinant;

    /* 对 x(t)=A*sin(wt+phase)，拟合得到的余弦系数为 A*sin(phase)，
       正弦系数为 A*cos(phase)，因此按此顺序传入 atan2。 */
    return (float)atan2(cosine_coefficient, sine_coefficient);
}
