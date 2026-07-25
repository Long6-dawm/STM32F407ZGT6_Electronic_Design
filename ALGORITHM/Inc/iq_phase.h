/**
 * @file iq_phase.h
 * @brief I/Q demodulation phase measurement interfaces.
 */

#ifndef IQ_PHASE_H
#define IQ_PHASE_H

#include "arm_math.h"

/**
 * @brief Convert SAMPLE_N real samples into an interleaved complex buffer.
 * @param[in] p Real-valued input buffer.
 */
void Create_data2handle(float32_t *p);

/**
 * @brief Calculate a fixed SAMPLE_N-point FFT magnitude spectrum.
 * @param[in] input Real-valued input buffer.
 * @param[out] output Magnitude output buffer.
 * @param[in] n Number of samples; must equal SAMPLE_N or the call is ignored.
 */
void CalXiebo(float32_t *input, float32_t *output, int n);

/**
 * @brief Estimate sine-wave phase with a DC-aware least-squares fit.
 * @param[in] f Signal frequency in hertz; must satisfy 0 < f < fs/2.
 * @param[in] fs Sampling frequency in hertz.
 * @param[in] N Number of input samples; must not exceed SAMPLE_N.
 * @param[in] adc_float Sample buffer.
 * @return Wrapped phase in radians in the interval [-pi, pi], or 0 for invalid input.
 * @note Coherent sampling is not required; the fit uses all N samples.
 */
float CalPhase(float f, float fs, int N, float32_t *adc_float);

#endif /* IQ_PHASE_H */
