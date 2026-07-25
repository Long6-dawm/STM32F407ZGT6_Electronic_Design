/**
 * @file rms_amplitude.h
 * @brief RMS-based waveform amplitude measurement interfaces.
 */

#ifndef RMS_AMPLITUDE_H
#define RMS_AMPLITUDE_H

#include <stdint.h>

/**
 * @brief Measure sine-wave peak amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated peak amplitude in volts, or 0 for invalid input.
 */
float Measuring_Sine_Amplitude(uint16_t length, uint16_t *AD_value);

/**
 * @brief Measure square-wave amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated amplitude in volts, or 0 for invalid input.
 */
float Measuring_Square_Amplitude(uint16_t length, uint16_t *AD_value);

/**
 * @brief Measure triangle-wave peak amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated peak amplitude in volts, or 0 for invalid input.
 */
float Measuring_Triangle_Amplitude(uint16_t length, uint16_t *AD_value);

#endif /* RMS_AMPLITUDE_H */
