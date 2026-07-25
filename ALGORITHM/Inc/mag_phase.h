/**
 * @file mag_phase.h
 * @brief Legacy amplitude helpers bundled with the I/Q phase module.
 */

#ifndef MAG_PHASE_H
#define MAG_PHASE_H

#include <stdint.h>

/**
 * @brief Measure sine-wave peak amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated peak amplitude in volts.
 */
float Measuring_Sine_Amplitude(uint16_t length, uint16_t *AD_value);

/**
 * @brief Measure square-wave amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated amplitude in volts.
 */
float Measuring_Square_Amplitude(uint16_t length, uint16_t *AD_value);

/**
 * @brief Measure triangle-wave peak amplitude from ADC samples.
 * @param[in] length Number of samples.
 * @param[in] AD_value ADC sample buffer.
 * @return Estimated peak amplitude in volts.
 */
float Measuring_Triangle_Amplitude(uint16_t length, uint16_t *AD_value);

#endif /* MAG_PHASE_H */
