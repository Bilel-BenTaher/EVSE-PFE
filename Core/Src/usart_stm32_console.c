/*
 * usart_stm32_console.c
 *
 *  Created on: Jul 19, 2024
 *      Author: hp
 */

#include "stm32u5xx.h"
#include "usart_stm32_console.h"
#include "string.h"
#include <stdlib.h>


void USART_STM32_sendStringToUSART(const char *givenString) {
    // Send each character of the string
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);
        givenString++;
    }

    // Send newline (LF)
    uint8_t newline = 0x0A;  // Line Feed
    HAL_UART_Transmit(&huart1, &newline, 1, HAL_MAX_DELAY);

    // Send carriage return (CR)
    uint8_t carriageReturn = 0x0D;  // Carriage Return
    HAL_UART_Transmit(&huart1, &carriageReturn, 1, HAL_MAX_DELAY);
}


void USART_STM32_sendIntegerToUSART(const char *givenString, uint16_t myInteger) {
    // Send the string
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);
        givenString++;
    }

    // Convert integer to string
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%u", myInteger);

    // Send the string representation of the integer
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
