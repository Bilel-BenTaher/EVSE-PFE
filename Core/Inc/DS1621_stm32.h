/*
 * @file DS1621_stm32.h
 * @brief Header file for interfacing the DS1621 temperature sensor with STM32 via I2C.
 *
 * This file contains the necessary definitions and function declarations to interface with
 * the DS1621 temperature sensor using I2C communication on an STM32 microcontroller.
 * The DS1621 sensor provides digital temperature readings in Celsius.
 *
 * @date July 24, 2024
 * @author Bilel BENTAHER
 */

#ifndef INC_DS1621_STM32_H_
#define INC_DS1621_STM32_H_
#include "stm32u5xx_hal.h"  // Include the STM32 HAL library for I2C communication
/**
 * @brief I2C address of the DS1621 temperature sensor.
 *
 * The DS1621 sensor operates over the I2C interface. The default address is 0x48, which is
 * shifted left by 1 to comply with the STM32 HAL library's 7-bit addressing format.
 */
#define DS1621_I2C_ADDRESS (0x48 << 1)  // Default I2C address for DS1621

/**
 * @brief Reads the temperature from the DS1621 sensor.
 *
 * This function communicates with the DS1621 temperature sensor over I2C to retrieve the
 * current temperature value. The temperature is returned as a float value representing
 * degrees Celsius.
 *
 * @param[in] hi2c A pointer to the I2C handle (I2C_HandleTypeDef) to manage the I2C communication.
 *                 This handle should be properly initialized before calling this function.
 *
 * @return The temperature in degrees Celsius as a floating-point value.
 * @note The DS1621 temperature sensor provides an accuracy of ±0.5°C for typical applications.
 */
float DS1621_getTemperature(I2C_HandleTypeDef *hi2c);

#endif /* INC_DS1621_STM32_H_ */

