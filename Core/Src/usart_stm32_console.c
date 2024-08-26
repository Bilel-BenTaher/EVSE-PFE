/*
 * @file usart_stm32_console.c
 * @brief Source file for STM32 USART interface functions.
 *
 * This file contains the implementation of functions used to send strings and integers over
 * a USART connection on an STM32U5 microcontroller. The functions rely on HAL libraries for
 * UART communication and handle both strings and integers with basic formatting.
 *
 * @date July 19, 2024
 * @author hp
 */

#include "stm32u5xx.h"
#include "usart_stm32_console.h"
#include "string.h"
#include <stdlib.h>

/**
 * @brief Sends a null-terminated string followed by a newline and carriage return over the USART interface.
 *
 * This function iterates through a given null-terminated string and transmits each character via the USART interface.
 * After the string, it appends a newline (`\n`) and a carriage return (`\r`) for proper formatting.
 *
 * @param[in] givenString Pointer to the null-terminated string to be sent.
 * @note This function uses `HAL_UART_Transmit` for blocking transmission and assumes the USART has been properly initialized.
 */
void USART_STM32_sendStringToUSART(const char *givenString) {
    // Send each character of the string
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);  // Transmit one character at a time
        givenString++;  // Move to the next character
    }

    // Send newline (LF) and carriage return (CR) for standard terminal formatting
    uint8_t newline = 0x0A;  // Line Feed (LF)
    HAL_UART_Transmit(&huart1, &newline, 1, HAL_MAX_DELAY);

    uint8_t carriageReturn = 0x0D;  // Carriage Return (CR)
    HAL_UART_Transmit(&huart1, &carriageReturn, 1, HAL_MAX_DELAY);
}

/**
 * @brief Sends a string followed by a 16-bit unsigned integer (formatted as a string) over the USART interface.
 *
 * This function first sends a given string, followed by the integer value (formatted as a string).
 * The integer is converted to its string representation using `snprintf` before being transmitted.
 *
 * @param[in] givenString Pointer to the null-terminated string to be sent before the integer.
 * @param[in] myInteger The 16-bit unsigned integer (`uint16_t`) to be transmitted.
 * @note The integer is sent in base 10 format. The function blocks until the transmission is complete.
 */
void USART_STM32_sendIntegerToUSART(const char *givenString, uint16_t myInteger) {
    // Send the string portion
    while (*givenString) {
        HAL_UART_Transmit(&huart1, (uint8_t*)givenString, 1, HAL_MAX_DELAY);  // Transmit one character at a time
        givenString++;  // Move to the next character
    }

    // Buffer to store the converted integer as a string
    char buffer[8];  // Buffer size can hold up to a maximum 5-digit unsigned integer (65535)

    // Convert the integer to string (base 10)
    snprintf(buffer, sizeof(buffer), "%u", myInteger);  // Convert integer to string in decimal format

    // Transmit the integer (now in string format)
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
