/*
 * DS1621_stm32.c
 *
 *  Created on: Jul 24, 2024
 *      Author: hp
 */

#include "DS1621_stm32.h"
#include <string.h>
#include <stdio.h>

HAL_StatusTypeDef DS1621_startConversion(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd = 0xEE; // Command to start temperature conversion
    // Check if device is ready
    if (HAL_I2C_IsDeviceReady(hi2c, DS1621_I2C_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK) {
        // Uncomment the following line if you are using USART for debugging
        // USART_STM32_sendStringToUSART("Error: Device not ready");
        Error_Handler();
    }
    // Send command to start conversion
    return HAL_I2C_Master_Transmit(hi2c, DS1621_I2C_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef DS1621_readTemperature(I2C_HandleTypeDef *hi2c, float *temperature) {
    uint8_t buf[2];
    uint16_t val;
    HAL_StatusTypeDef status;
    float temp_sum = 0.0f;

    for (int i = 0; i < DEBOUNCE_READINGS; i++) {
        // Start temperature conversion
        status = DS1621_startConversion(hi2c);
        if (status != HAL_OK) {
            // Uncomment the following line if you are using USART for debugging
            // USART_STM32_sendStringToUSART("Error: Conversion failed");
            Error_Handler();
        }
        HAL_Delay(1000); // Wait for conversion to complete

        // Request temperature reading
        buf[0] = 0xAA; // Command to read temperature
        status = HAL_I2C_Master_Transmit(hi2c, DS1621_I2C_ADDRESS, buf, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            // Uncomment the following line if you are using USART for debugging
            // USART_STM32_sendStringToUSART("Error: Transmit failed");
            Error_Handler();
        }
        // Receive temperature data
        status = HAL_I2C_Master_Receive(hi2c, DS1621_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            // Uncomment the following line if you are using USART for debugging
            // USART_STM32_sendStringToUSART("Error: Receive failed");
            Error_Handler();
        }

        // Combine bytes to get temperature value
        val = ((uint16_t)buf[0] << 4) | (buf[1] >> 4);
        // Handle negative temperatures
        if (val > 0x7FF) {
            val |= 0xF000;
        }
        // Convert to Celsius
        float temp_c = val * 0.0625f;
        temp_sum += temp_c;
    }

    *temperature = temp_sum / DEBOUNCE_READINGS; // Average temperature readings
    return HAL_OK;
}

float DS1621_getTemperature(I2C_HandleTypeDef *hi2c) {
    float temperature;
    HAL_StatusTypeDef status = DS1621_readTemperature(hi2c, &temperature);

    if (status == HAL_OK) {
        return temperature;
    } else {
        // Error handling: return an impossible value
        return -999.0f;
    }
}
