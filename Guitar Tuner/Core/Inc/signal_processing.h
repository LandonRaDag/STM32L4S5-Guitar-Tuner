/*
 * signal_processing.h
 *
 */

#ifndef INC_SIGNAL_PROCESSING_H_
#define INC_SIGNAL_PROCESSING_H_

#include "global_header.h"

float32_t yin_detect_frequency(float32_t *buffer, uint32_t length, uint32_t sample_rate);

#endif /* INC_SIGNAL_PROCESSING_H_ */
