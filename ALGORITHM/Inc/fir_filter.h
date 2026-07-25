/**
 * @file fir_filter.h
 * @brief Fixed-length CMSIS-DSP FIR filter wrapper.
 */

#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include "arm_math.h"

#define NUM_PER_CALL 1024
#define FIR_EFFICIENT_NUM 11U

/** @brief Initialize the fixed FIR filter instance. */
void arm_emg_f32_filter_init(void);

/**
 * @brief Filter one NUM_PER_CALL-sample block.
 * @param[in] dataInput Input sample buffer.
 * @param[out] dataOutput Output sample buffer.
 */
void arm_emg_f32_filter(float32_t *dataInput, float32_t *dataOutput);

#endif /* FIR_FILTER_H */
