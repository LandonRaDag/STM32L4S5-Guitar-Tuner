/*
 * mic_input.c
 *

 */

/**
 * Fills buffer with PCM values of microphone through DMA
 */

#include "mic_input.h"

void mic_DMASampleBuffer(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length){
	HAL_DFSDM_FilterRegularStart_DMA(hdfsdm_filter, pData, Length);
}



#include "mic_input.h"
