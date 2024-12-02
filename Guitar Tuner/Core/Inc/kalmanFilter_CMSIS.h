#ifndef KALMANFILTER_CMSIS_H
#define KALMANFILTER_CMSIS_H

#include "global_header.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float32_t q; // Process noise covariance
    float32_t r; // Measurement noise covariance
    float32_t x; // Estimated value
    float32_t p; // Estimation error covariance
    float32_t k; // Kalman gain
} kalman_state_t;
// Function to update Kalman filter using CMSIS DSP functions
void kalmanFilter(float32_t* inputArray, float32_t* outputArray, int length);

#ifdef __cplusplus
}
#endif

#endif // KALMANFILTER_CMSIS_H
