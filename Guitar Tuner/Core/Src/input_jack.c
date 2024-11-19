/*
 * input_jack.c
 *
 */

#include "input_jack.h"

void inputJack_Init(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Start(hadc);
}

uint32_t inputJack_getValue(ADC_HandleTypeDef *hadc) {
    return HAL_ADC_GetValue(hadc);
}

void inputJack_UARTValue(ADC_HandleTypeDef *hadc, UART_HandleTypeDef *huart) {
    int adcValue = inputJack_getValue(hadc);
    char jackMsg[100];
    sprintf(jackMsg, "ADC Value: %d\r\n", adcValue);
    HAL_UART_Transmit(huart, (uint8_t*)jackMsg, strlen(jackMsg), 1000);
    HAL_Delay(100);
}
