/**
 * @file czt_zoom_fft.c
 * @brief Chirp Z 变换频谱细化实现。
 */

#include "czt_zoom_fft.h"

#include "arm_common_tables.h"
#include <math.h>

#define PI 3.14159265358979f
#define N 2048
#define M  2048
#define NFFT  2048
#define P  4096



__attribute__((aligned(32))) static float gn_fft[2*P];  /* CZT 调制后的输入序列及 FFT 工作缓冲区。 */
__attribute__((aligned(32))) static float hn_fft[2*P];  /* 啁啾卷积核及 FFT 工作缓冲区。 */

void czt_Init_0(float32_t *input, int FS,
        int f_start, int f_end,float32_t *zoom_abs/*P*/)
{

    // 在指定角频率区间内构造 CZT。


    arm_cfft_radix2_instance_f32 scfft;
    const float wStart = 2.0f * PI * f_start / FS;
    const float wEnd = 2.0f * PI * f_end / FS;
    const float Deltaw = (wEnd - wStart) / (M - 1);
    const float APhase = wStart;
    const float WPhase = -Deltaw;

    // 对输入啁啾序列 g[n] 调制并补零。
    for(int k = 0; k < P; k++) {
        const int idx = 2 * k;
        if(k < N) {
            const float angle = WPhase * (k * k) * 0.5f - k * APhase;
            gn_fft[idx] = input[k] * arm_cos_f32(angle);    /* g[n] 的实部。 */
            gn_fft[idx+1] = input[k] * arm_sin_f32(angle);  /* g[n] 的虚部。 */
        } else {
            gn_fft[idx] = 0.0f;
            gn_fft[idx+1] = 0.0f;
        }
    }

    // 构造啁啾卷积核 h[n]，其余位置补零。
    int Count = P - M + 1;
    for(int k = 0; k < P; k++) {
        const int idx = 2 * k;
        if(k < N) {
            const float angle = WPhase * k * k * 0.5f;
            hn_fft[idx] = arm_cos_f32(angle);    /* h[n] 的实部。 */
            hn_fft[idx+1] = -arm_sin_f32(angle); /* h[n] 的虚部。 */
        } else if(k <= (N + 2*(P-M-N))) {
            hn_fft[idx] = 0.0f;
            hn_fft[idx+1] = 0.0f;
        } else {
            const int n = P - Count;
            const float angle = WPhase * n * n * 0.5f;
            hn_fft[idx] = arm_cos_f32(angle);    /* 折叠段 h[n] 的实部。 */
            hn_fft[idx+1] = -arm_sin_f32(angle); /* 折叠段 h[n] 的虚部。 */
            Count++;
        }
    }

    // 对两个序列执行 FFT，以便在频域完成卷积。
    arm_cfft_radix2_init_f32(&scfft, P, 0, 1);
    arm_cfft_radix2_f32(&scfft, gn_fft);
    arm_cfft_radix2_f32(&scfft, hn_fft);

    // 将两个复数频谱逐点相乘。
    for(int k = 0; k < P; k++) {
        const int idx = 2 * k;
        const float real1 = gn_fft[idx];
        const float imag1 = gn_fft[idx+1];
        const float real2 = hn_fft[idx];
        const float imag2 = hn_fft[idx+1];

        // 复数乘法：(a + bi)(c + di) = (ac - bd) + (ad + bc)i。
        gn_fft[idx] = real1 * real2 - imag1 * imag2;   /* 乘积实部。 */
        gn_fft[idx+1] = real1 * imag2 + imag1 * real2; /* 乘积虚部。 */
    }

    // 对卷积结果执行逆变换。
    arm_cfft_radix2_init_f32(&scfft, P, 1, 1);
    arm_cfft_radix2_f32(&scfft, gn_fft);

    // 输出 M 个 CZT 采样点的幅值。

    for(int k = 0; k < M; k++) {
        const int idx = 2 * k;
        const float real = gn_fft[idx];
        const float imag = gn_fft[idx+1];
        zoom_abs[k] = sqrtf(real * real + imag * imag);
    }

}


/********************************************************************************/


/* 返回 CZT 最大幅值频点对应的频率。 */
float czt_result_fre(int FS, int f_start, int f_end,float32_t *zoom_abs/*P*/)
{
	int idx=0;
	float FrequencyEstimate;
	float wStart = 2.0*PI*f_start/FS;
	float wEnd = 2.0*PI*f_end/FS;
	float Deltaw = 1.0*(wEnd - wStart)/(M - 1);
	for(int j=1;j<M;j++)
	{
		if(zoom_abs[j]>=zoom_abs[idx])
			idx = j;
	}

	FrequencyEstimate = (wStart+(idx)*Deltaw)*FS/(2*PI);
	return FrequencyEstimate;
}






/* 返回 CZT 最大幅值频点对应的幅度估计值。 */
float czt_Amp(int FS, int f_start, int f_end,float32_t *zoom_abs/*P*/)
{
	int idx=0;
	float Amplitude;
	float wStart = 2.0*PI*f_start/FS;
	float wEnd = 2.0*PI*f_end/FS;
	float Deltaw = 1.0*(wEnd - wStart)/(M - 1);
	for(int j=1;j<M;j++)
	{
		if(zoom_abs[j]>=zoom_abs[idx])
			idx = j;
	}

	Amplitude = zoom_abs[idx] * (2.0f / (float)N);
	return Amplitude;
}


/* 返回 CZT 最大幅值频点对应的复数相位。 */
float czt_Phase(int FS, int f_start, int f_end,float32_t *zoom_abs/*P*/)
{
	int idx=0;
	float Phase;
	float wStart = 2.0*PI*f_start/FS;
	float wEnd = 2.0*PI*f_end/FS;
	float Deltaw = 1.0*(wEnd - wStart)/(M - 1);
	for(int j=1;j<M;j++)
	{
		if(zoom_abs[j]>=zoom_abs[idx])
			idx = j;
	}
	float Realpart = gn_fft[2*idx];
	float Imagpart = gn_fft[2*idx+1];
	Phase = atan2f(Imagpart,Realpart);
//	 if(Phase < 0) {
//        Phase += 2 * PI;
//    }
	return Phase;
}


