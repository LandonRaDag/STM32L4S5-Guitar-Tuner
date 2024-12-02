#include "KalmanFilter_CMSIS.h"
#include "arm_math.h" // CMSIS-DSP library

// Function to update Kalman filter using CMSIS DSP functions
void kalmanFilter(float32_t* inputArray, float32_t* outputArray, int length) {
	kalman_state_t kstate_cmsis;

	// Initialize the internal Kalman state
    kstate_cmsis.q = 0.01f;  // Small process noise covariance
    kstate_cmsis.r = 0.1f;   // Small measurement noise covariance
    kstate_cmsis.x = 0.0f;   // Initial estimate
    kstate_cmsis.p = 1.0f;   // Initial estimation error covariance (large uncertainty)
    kstate_cmsis.k = 0.0f;   // Kalman gain initialized to 0

	for (int i = 0; i < length; i++) {
		kstate_cmsis.p += kstate_cmsis.q;

		float32_t div = kstate_cmsis.p + kstate_cmsis.r;

		// Calculate the Kalman gain
		kstate_cmsis.k = kstate_cmsis.p / div;

		// Update the estimated value
		kstate_cmsis.x += kstate_cmsis.k * (inputArray[i] - kstate_cmsis.x);

		// Update the estimation error covariance
		kstate_cmsis.p *= (1 - kstate_cmsis.k);

		// Store the result in the output array
		outputArray[i] = kstate_cmsis.x;
	}


}
