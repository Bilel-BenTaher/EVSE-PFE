/*
 * usart_stm32_console.h
 *
 *  Created on: Jul 19, 2024
 *      Author: hp
 */

#ifndef INC_USART_STM32_CONSOLE_H_
#define INC_USART_STM32_CONSOLE_H_

// USART_STM32_CONSOLE library: This library shall enable interfacing an FTDI-based UART device to an STM32u5 chip.
// Parameter Definitions
#define    USART_STM32_BAUDRATE        115200

// Variable Declarations

// Function Definitions

// Function Declarations
void USART_STM32_sendStringToUSART(char const *givenString);
void USART_STM32_sendIntegerToUSART(char const *givenString, uint16_t myInteger);

#endif /* INC_USART_STM32_CONSOLE_H_ */
