/*
 * DS1621_stm32.h
 *
 *  Created on: Jul 24, 2024
 *      Author: hp
 */

#ifndef INC_DS1621_STM32_H_
#define INC_DS1621_STM32_H_

#include "stm32u5xx.h"
#include "usart_stm32_console.h"

// Adresse I2C du capteur DS1621
#define TEMP_ADRESS (0x48 << 1)
#define DEBOUNCE_READINGS 5 // Nombre de lectures pour le debounce

HAL_StatusTypeDef DS1621_startConversion(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DS1621_readTemperature(I2C_HandleTypeDef *hi2c, float *temperature);
float DS1621_getTemperature(I2C_HandleTypeDef *hi2c);

#endif /* INC_DS1621_STM32_H_ */
