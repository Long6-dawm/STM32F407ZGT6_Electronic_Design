/**
 * @file fft_buffer.h
 * @brief Shared buffers and fixed constants for the legacy I/Q module.
 */

#ifndef FFT_BUFFER_H
#define FFT_BUFFER_H

#include "arm_math.h"

#define SAMPLE_N 1024
#define SAMPLE_FS 51200
#define FFT_DPI (SAMPLE_FS / SAMPLE_N)
#define PI 3.14159265358979f

extern float32_t FFT_Input[SAMPLE_N];
extern float32_t data2handle[SAMPLE_N * 2];
extern float32_t FFT_Output[SAMPLE_N];
extern float32_t xiebo[SAMPLE_N];
extern float32_t phaseA[SAMPLE_N];
extern float32_t phaseB[SAMPLE_N];
extern float32_t phaseC[SAMPLE_N];

#endif /* FFT_BUFFER_H */
