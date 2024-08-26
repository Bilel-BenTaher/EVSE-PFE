/*
 * @file DS1621_stm32.c
 * @brief Source file for interfacing the DS1621 temperature sensor with STM32 via I2C.
 *
 * This file contains the implementation of functions used to communicate with the DS1621
 * temperature sensor over I2C and retrieve the temperature in degrees Celsius.
 *
 * @date July 24, 2024
 * @author hp
 */

#include "DS1621_stm32.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief Retrieves the temperature from the DS1621 sensor.
 *
 * This function initiates a temperature conversion on the DS1621 temperature sensor using
 * the I2C protocol. Once the conversion is complete, it reads the temperature value from
 * the sensor, converts it to Celsius, and returns the value as a float.
 *
 * @param[in] hi2c Pointer to the I2C_HandleTypeDef structure that contains the configuration
 *                 information for the I2C module used for communication.
 *
 * @return Temperature in degrees Celsius as a floating-point value.
 * @note The conversion time is approximately 1 second, hence the delay introduced before
 *       reading the temperature.
 */
float DS1621_getTemperature(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd_start = 0xEE;  // Command to start temperature conversion
    uint8_t cmd_read = 0xAA;   // Command to read the temperature data
    uint8_t buf[2];            // Buffer to hold the received data (2 bytes)
    int16_t val;               // 16-bit variable to store the signed temperature value

    // Ensure the DS1621 device is ready for communication
    if (HAL_I2C_IsDeviceReady(hi2c, DS1621_I2C_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();  // Handle the error if the device is not ready
    }

    // Start the temperature conversion process
    if (HAL_I2C_Master_Transmit(hi2c, DS1621_I2C_ADDRESS, &cmd_start, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();  // Handle transmission error
    }

    HAL_Delay(1000);  // Wait 1 second for the conversion to complete

    // Request the temperature data from the DS1621
    if (HAL_I2C_Master_Transmit(hi2c, DS1621_I2C_ADDRESS, &cmd_read, 1, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();  // Handle transmission error
    }

    // Receive the temperature data (2 bytes)
    if (HAL_I2C_Master_Receive(hi2c, DS1621_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY) != HAL_OK) {
        Error_Handler();  // Handle reception error
    }

    // Combine the two received bytes to form the temperature value (12-bit resolution)
    val = ((int16_t)buf[0] << 8) | (buf[1] & 0xF0);  // Mask lower nibble of buf[1]

    // Handle negative temperatures
    if (val & 0x0800) {  // Check if the sign bit (bit 11) is set
            val |= 0xF000;  // Extend the sign to 16 bits
            val = -((~val + 1) & 0x0FFF);  // Two's complement conversion to negative value
    }

    // Convert to Celsius, considering the sensor resolution (0.0625°C per bit)
    return val * 0.0625f;
}
