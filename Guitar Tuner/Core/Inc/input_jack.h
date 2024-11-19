/*
 * input_jack.h
 *
 */

#ifndef INC_INPUT_JACK_H_
#define INC_INPUT_JACK_H_

#include "global_header.h"

void inputJack_Init(ADC_HandleTypeDef *hadc);
uint32_t inputJack_getValue(ADC_HandleTypeDef *hadc);
void inputJack_UARTValue(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart);

#endif /* INC_INPUT_JACK_H_ */
