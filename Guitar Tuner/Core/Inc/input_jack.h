/*
 * input_jack.h
 *
 */

#ifndef INC_INPUT_JACK_H_
#define INC_INPUT_JACK_H_

#include "global_header.h"

void inputJack_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);
uint32_t inputJack_getValue(ADC_HandleTypeDef *hadc);
void inputJack_UARTValue(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);
void inputJack_DMASampleBuffer(ADC_HandleTypeDef *hadc, uint32_t *buffer, uint32_t buffer_size);
void inputJack_StopDMA(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

#endif /* INC_INPUT_JACK_H_ */
