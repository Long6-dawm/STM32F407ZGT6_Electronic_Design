/**
 * @file fft_interp_freq.c
 * @brief 基于 FFT 峰值及对数抛物线插值的频率估计实现。
 */

#include "fft_interp_freq.h"

#include "fft_n.h"
struct  compx s[MAX_FFT_N];
static uint32_t temp_1;
float32_t f,temp1;
float cfft_f32_fre(float32_t fs,uint16_t *AD_Value,uint8_t flag)
{
	uint32_t i,j,k;
	float32_t magmax = 0.0f;

	InitTableFFT(MAX_FFT_N);

	for(i=0; i<MAX_FFT_N; i++)
	{
		/* 将 ADC 采样值装入 FFT 输入的实部，虚部置零。 */
		s[i].real = AD_Value[i];
		s[i].imag = 0;
	}

	/* 计算 MAX_FFT_N 点 FFT。 */
	cfft(s, MAX_FFT_N);

	/* 将每个复数频点转换为幅值。 */
	for(k=0; k<MAX_FFT_N; k++)
	{
		arm_sqrt_f32((float32_t)(s[k].real *s[k].real+ s[k].imag*s[k].imag ), &s[k].real);
	}

	/* 跳过直流分量，在正频率区间搜索幅值峰值。 */
	temp_1 = 2U;
	for(j=2; j<MAX_FFT_N/2; j++)
	{
		if(s[j].real > magmax)
			{
				 magmax = s[j].real;
				 temp_1 = j;
			}
	}

  if(flag==1)
	{

	/* 对数抛物线插值可减小矩形窗下直接拟合线性幅值产生的较大偏差。 */
	const float32_t left = logf(fmaxf(s[temp_1 - 1U].real, 1.0e-12f));
	const float32_t peak = logf(fmaxf(s[temp_1].real, 1.0e-12f));
	const float32_t right = logf(fmaxf(s[temp_1 + 1U].real, 1.0e-12f));
	const float32_t denominator = left - 2.0f * peak + right;
	float32_t delta = 0.0f;
	if (fabsf(denominator) > 1.0e-12f) {
		delta = 0.5f * (left - right) / denominator;
	}
	temp1 = (float32_t)temp_1 + delta;
	f = temp1 * fs / MAX_FFT_N;
	return f;

	}

	 if(flag==2)
	{

	f=temp_1*fs/MAX_FFT_N;
	return f;

	}

	return 0.0f;


}
