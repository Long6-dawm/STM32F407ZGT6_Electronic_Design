/**
 * @file differ_amp.h
 * @brief Differential triangle-wave amplitude measurement interfaces.
 */

#ifndef DIFFER_AMP_H
#define DIFFER_AMP_H

#include <stdint.h>

#include "arm_math.h"

/**
 * @brief Sort a floating-point array in ascending order.
 * @param[in,out] arr Array to sort.
 * @param[in] length Number of elements in the array.
 */
void quick_sort_large(float32_t *arr, uint32_t length);

/**
 * @brief Estimate triangle-wave peak-to-peak amplitude from ADC samples.
 * @param[in] Length Number of samples; the legacy implementation supports up to 1024.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated peak-to-peak amplitude in volts, or 0 when no edge is found.
 */
float Differ_Tri_Amp(uint16_t Length, uint16_t *AD_value);

#endif /* DIFFER_AMP_H */
