/*
 * DS1621_stm32.h
 *
 *  Created on: Jul 24, 2024
 *      Author: hp
 */

#ifndef INC_DS1621_STM32_H_
#define INC_DS1621_STM32_H_

#include "stm32u5xx_hal.h" // Include HAL library header
#include "usart_stm32_console.h" // Include USART for debug messages

// I2C Address of DS1621 sensor
#define DS1621_I2C_ADDRESS (0x48 << 1)
#define DEBOUNCE_READINGS 5 // Number of readings for debounce

// Function Declarations
HAL_StatusTypeDef DS1621_startConversion(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DS1621_readTemperature(I2C_HandleTypeDef *hi2c, float *temperature);
float DS1621_getTemperature(I2C_HandleTypeDef *hi2c);

#endif /* INC_DS1621_STM32_H_ */
