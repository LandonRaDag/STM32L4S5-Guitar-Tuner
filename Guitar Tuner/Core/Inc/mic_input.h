/*
 * mic_input.h
 *
 */

#ifndef INC_MIC_INPUT_H_
#define INC_MIC_INPUT_H_

#include "global_header.h"

void mic_DMASampleBuffer(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length);

#endif /* INC_MIC_INPUT_H_ */
