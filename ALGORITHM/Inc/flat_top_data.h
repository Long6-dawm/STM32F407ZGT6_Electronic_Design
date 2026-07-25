/**
 * @file flat_top_data.h
 * @brief Flat-top-window FFT amplitude measurement interfaces.
 */

#ifndef FLAT_TOP_DATA_H
#define FLAT_TOP_DATA_H

#include <stdint.h>

/**
 * @brief Measure sine-wave amplitude from a 4096-sample ADC buffer.
 * @param[in] AD_Value ADC sample buffer.
 * @return Estimated sine-wave amplitude in volts.
 */
float Sin_Amp_FFT(uint16_t *AD_Value);

/**
 * @brief Measure square-wave amplitude from a 4096-sample ADC buffer.
 * @param[in] AD_Value ADC sample buffer.
 * @return Estimated square-wave amplitude in volts.
 */
float Square_Amp_FFT(uint16_t *AD_Value);

/**
 * @brief Measure triangle-wave amplitude from a 4096-sample ADC buffer.
 * @param[in] AD_Value ADC sample buffer.
 * @return Estimated triangle-wave amplitude in volts.
 */
float Triangle_Amp_FFT(uint16_t *AD_Value);

#endif /* FLAT_TOP_DATA_H */
