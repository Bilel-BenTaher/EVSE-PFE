/*
 * usart_stm32_console.h
 *
 *  Created on: Jul 19, 2024
 *      Author: hp
 */

#ifndef INC_USART_STM32_CONSOLE_H_
#define INC_USART_STM32_CONSOLE_H_

#include "stm32u5xx_hal.h"

// USART_STM32_CONSOLE library: This library enables interfacing an FTDI-based UART device with an STM32U5 chip.
// Parameter Definitions
#define USART_STM32_BAUDRATE        115200

// Function Declarations
void USART_STM32_sendStringToUSART(const char *givenString);
void USART_STM32_sendIntegerToUSART(const char *givenString, uint16_t myInteger);

#endif /* INC_USART_STM32_CONSOLE_H_ */
