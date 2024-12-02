/*
 * mic_input.c
 *

 */

/**
 * Fills buffer with PCM values of microphone through DMA
 */

#include "mic_input.h"

float clip = 2000.0f;

void mic_DMASampleBuffer(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length){
	HAL_DFSDM_FilterRegularStart_DMA(hdfsdm_filter, pData, Length);
}

float limit_val(float input) {
    if (input > 0) {
        return (input < clip) ? input : clip; // Minimum of 200 and input
    } else {
        return (input > -clip) ? input : -clip; // Maximum of -200 and input
    }
}


//needed to remove low freq information thats messing everything up
void mic_process(int32_t *interim_buffer, float32_t *good_buffer, int good_buffer_length) {

	// Filter coefficients for a 50 Hz high-pass filter with a 48 kHz sample rate
    float alpha = (2 * M_PI * 50) / (48000 + 2 * M_PI * 50);
    float alphinv = 1.0f / (1.0f + (48000 / (2 * M_PI * 2000)));

    // Initialize the previous output (y[n-1]) and previous input (x[n-1])
    float prev_output = 0.0f;
    float prev_input = 0.0f;
    float sum = 0.0f;

    //high pass
    for (int i = 0; i < good_buffer_length; i++) {
        // Get the current input
        float current_input = (float32_t)interim_buffer[i + 200];

        // Apply the high-pass filter equation
        float current_output = alpha * (prev_output + current_input - prev_input);

        // Store the filtered output
        good_buffer[i] = current_output;
        sum += current_output * current_output;

        // Update previous input and output for the next iteration
        prev_input = current_input;
        prev_output = current_output;
    }

    // Check against the threshold

    float32_t rms = sqrtf((float)sum / (float)good_buffer_length);
    if (rms < 500) {
        good_buffer[0] = -25.0f; // Sentinel value to indicate skipping
        return;
    }
    //lowpass
    prev_output = 0.0f;
    for (int i = 0; i < good_buffer_length; i++) {
    	float current_input = good_buffer[i];
    	float current_output = alphinv * current_input + (1 - alphinv) * prev_output;

        // Store the filtered output
        good_buffer[i] = (float)limit_val(current_output) / clip;

        // Update previous input and output for the next iteration
        prev_input = current_input;
        prev_output = current_output;

    }

    //kalman
    kalmanFilter(good_buffer, good_buffer, good_buffer_length);
}


#include "mic_input.h"
