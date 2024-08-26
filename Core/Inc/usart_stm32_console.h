/*
 * @file usart_stm32_console.h
 * @brief Header file for STM32 USART interface functions.
 *
 * This file contains the declarations of functions used to interface an STM32U5 microcontroller
 * with an FTDI-based UART device for sending strings and integers over a USART connection.
 * The library provides basic functionalities such as sending data strings and formatted integers.
 *
 * @date July 19, 2024
 * @author hp
 */

#ifndef INC_USART_STM32_CONSOLE_H_
#define INC_USART_STM32_CONSOLE_H_

#include "stm32u5xx_hal.h"

// Baud rate for the USART interface
#define USART_STM32_BAUDRATE 115200 /**< @brief USART baud rate for communication */

/**
 * @brief Sends a null-terminated string over the USART interface.
 *
 * This function takes a string (const char *) and transmits it via the USART interface configured for the STM32.
 * It sends the entire string character by character until the null terminator is reached.
 *
 * @param[in] givenString A pointer to the null-terminated string to be sent.
 * @note This function assumes that the USART interface has been correctly initialized prior to calling.
 * @note The function blocks until all characters in the string have been transmitted.
 */
void USART_STM32_sendStringToUSART(const char *givenString);

/**
 * @brief Sends a formatted string with an integer value over the USART interface.
 *
 * This function sends a string (const char *) followed by an integer value formatted as a string (in base 10).
 * The integer is converted to a string and concatenated to the given string before transmission.
 *
 * @param[in] givenString A pointer to the null-terminated string to send before the integer.
 * @param[in] myInteger The 16-bit unsigned integer (uint16_t) to be sent after the string.
 * @note This function assumes that the USART interface has been correctly initialized prior to calling.
 * @note The function blocks until the string and the integer have been transmitted.
 */
void USART_STM32_sendIntegerToUSART(const char *givenString, uint16_t myInteger);

#endif /* INC_USART_STM32_CONSOLE_H_ */
