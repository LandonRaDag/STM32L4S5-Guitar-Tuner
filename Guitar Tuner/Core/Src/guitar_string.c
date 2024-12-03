/*
 * String.c
 *
 *  Created on: Dec 2, 2024
 *      Author: Landon Ra Dagenais
 */
#include <guitar_string.h>

GuitarString createGuitarString(int number, float frequency, float targetFrequency, const char* note) {
    GuitarString string = {number, frequency, targetFrequency, note};
    return string;
}

void initializeGuitarStrings(GuitarString strings[], GuitarString** currentString) {

    strings[0] = createGuitarString(0, 82.41, 82.41, "E2");  // Low E
    strings[1] = createGuitarString(1, 110.00, 110.00, "A2");
    strings[2] = createGuitarString(2, 146.83, 146.83, "D3");
    strings[3] = createGuitarString(3, 196.00, 196.00, "G3");
    strings[4] = createGuitarString(4, 246.94, 246.94, "B3");
    strings[5] = createGuitarString(5, 329.63, 329.63, "E4");  // High E

    // Default to the first string (low E in this case)
    *currentString = &strings[0];
}

float calculateTuningOffset(const GuitarString* string) {
    return string->frequency - string->targetFrequency;
}

void switchString(GuitarString strings[], GuitarString** currentString) {

    int currentIndex = -1;
    for (int i = 0; i < 6; i++) {
        if (&strings[i] == *currentString) {
            currentIndex = i;
            break;
        }
    }

    int nextIndex = (currentIndex + 1) % 6;
    *currentString = &strings[nextIndex];
}

#define THRESHOLD 0.025f          // Threshold for the YIN algorithm

void yin_detect_frequency(float32_t *buffer, uint32_t length, uint32_t sample_rate, GuitarString* string) {
	float32_t min_value = 1.0f;
	uint32_t min_index = 0;
	float32_t cumulative_diff[length];
	float32_t threshold = THRESHOLD;

	float32_t target_frequency = string->targetFrequency;
	uint32_t target_lag = (uint32_t)(sample_rate / target_frequency);

	uint32_t start_lag = target_lag - 50;
	uint32_t end_lag = target_lag + 50;

	// Step 1: Calculate difference function
	for (uint32_t lag = start_lag ; lag < end_lag; lag++) {
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

		float32_t lag_refined = min_index;
	    if (min_index > start_lag && min_index < end_lag) {
	        float32_t prev = cumulative_diff[min_index - 1];
	        float32_t next = cumulative_diff[min_index + 1];
	        lag_refined = min_index + (prev - next) / (2.0f * (prev - 2.0f * cumulative_diff[min_index] + next));
	    }
		float32_t detected_freq = ((float32_t)sample_rate / lag_refined) + 1.0f;
		string->frequency = detected_freq;  // Update the string's frequency
	}

}
