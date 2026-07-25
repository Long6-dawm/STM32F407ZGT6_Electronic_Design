/**
 * @file adc_backend.h
 * @brief Platform-neutral ADC conversion interface used by DSP algorithms.
 */

#ifndef ADC_BACKEND_H
#define ADC_BACKEND_H

#include <stdint.h>

#include "arm_math.h"

#define ADC_BUF_LEN 1024

/**
 * @brief Convert raw ADC codes to volts using the linked target backend.
 * @param[in] intArray Raw ADC sample buffer.
 * @param[out] floatArray Converted voltage buffer.
 * @param[in] size Number of samples to convert.
 * @note Link exactly one `adc_backend_*.c` implementation for the target MCU.
 */
void ADC_Backend_RawToVoltage(uint16_t *intArray, float32_t *floatArray,
                             int size);

/** ADC channel buffers supplied by the selected backend implementation. */
extern uint16_t ADC_CHANNEL_1[ADC_BUF_LEN];
extern uint16_t ADC_CHANNEL_2[ADC_BUF_LEN];

#endif /* ADC_BACKEND_H */
