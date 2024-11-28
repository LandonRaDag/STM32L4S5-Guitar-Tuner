// global_header.h

#ifndef GLOBAL_HEADER_H
#define GLOBAL_HEADER_H

// STM32 HAL includes
#include "stm32l4xx_hal.h"

#include "arm_math.h"
#include "signal_processing.h"
#include "input_jack.h"
#include "mic_input.h"

// Standard libraries
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Global stuff
#define SAMPLE_RATE 48000       // Sampling rate in Hz
#define BUFFER_SIZE 2048    // Buffer size for recorded (microphone OR jack) data

#endif // GLOBAL_HEADER_H
