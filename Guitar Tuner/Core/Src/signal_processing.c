/*
 * signal_processing.c
 *
 */

#include "signal_processing.h"

#define THRESHOLD 0.05f          // Threshold for the YIN algorithm
#define HIGH_PASS_FREQ 50       // High-pass filter frequency
#define LOW_PASS_FREQ 2000      // Low-pass filter frequency

float32_t yin_detect_frequency(float32_t *buffer, uint32_t length, uint32_t sample_rate) {
	float32_t min_value = 1.0f;
	uint32_t min_index = 0;
	float32_t cumulative_diff[length];
	float32_t threshold = THRESHOLD;

	// Step 1: Calculate difference function
	for (uint32_t lag = 12; lag < length / 2; lag++) {
		float32_t sum = 0.0f;
		for (uint32_t i = 0; i < length - lag; i++) {
			float32_t diff = buffer[i] - buffer[i + lag];
			sum += diff * diff;
		}
		cumulative_diff[lag] = sum / (float32_t)(length - lag);

		// Step 2: Check if cumulative difference crosses the threshold
		if (cumulative_diff[lag] < threshold && lag > 1) {
			if (cumulative_diff[lag] < min_value) {
				min_value = cumulative_diff[lag];
				min_index = lag;
			}
		}
	}

	if (min_index > 0) {
		return  (float32_t)(sample_rate / min_index);
	} else {
		return 0.0f;  // No valid frequency found
	}
}

void applyHannWindow(float32_t *signal, float32_t *windowedSignal, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        float32_t hannValue = 0.5 * (1 - cosf((2 * M_PI * i) / (length - 1)));
        windowedSignal[i] = signal[i] * hannValue;
    }
}
