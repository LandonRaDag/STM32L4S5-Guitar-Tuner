/*
 * input_jack.c
 *
 */

#include "input_jack.h"

void inputJack_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim) {
	HAL_TIM_Base_Start(htim);
    //HAL_ADC_Start(hadc);
}

uint32_t inputJack_getValue(ADC_HandleTypeDef *hadc) {
    return HAL_ADC_GetValue(hadc);
}

void inputJack_UARTValue(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart) {
	HAL_ADC_Start(hadc);
	int adcValue = inputJack_getValue(hadc);
    char jackMsg[100];
    sprintf(jackMsg, "ADC Value: %d\r\n", adcValue);
    HAL_UART_Transmit(huart, (uint8_t*)jackMsg, strlen(jackMsg), 1000);
}

void inputJack_DMASampleBuffer(ADC_HandleTypeDef *hadc, uint32_t *buffer, uint32_t buffer_size){
    HAL_ADC_Start_DMA(hadc, buffer, buffer_size);
}

