/*
 * @file PZEM004T.h
 * @brief Header file for interfacing the PZEM-004T sensor with STM32 via UART/RS485.
 *
 * This file contains the necessary definitions and function declarations to interface with
 * the PZEM-004T power sensor using Modbus-RTU communication on an STM32 microcontroller.
 * The sensor provides readings of voltage, current, and power, which are crucial for
 * monitoring electrical parameters in various applications.
 *
 * @date September 3, 2024
 * @author Bilel BENTAHER
 */

#ifndef INC_PZEM004T_H_
#define INC_PZEM004T_H_

#include "stm32u5xx_hal.h"  // Include the STM32 HAL library for UART communication

/**
 * @brief Modbus slave address of the PZEM-004T sensor.
 *
 * This address is used to identify the PZEM-004T sensor on the Modbus network.
 * The default address is 0x01.
 */
#define PZEM_SLAVE_ADDRESS 0x01

/**
 * @brief Register addresses for the PZEM-004T sensor measurements.
 *
 * These registers store the measurement data for voltage, current, and power
 * from the PZEM-004T sensor. Each register is 16 bits in size.
 */
#define PZEM_VOLTAGE_REGISTER      0x0000   // Register address for voltage measurement
#define PZEM_CURRENT_REGISTER      0x0001   // Register address for current measurement
#define PZEM_POWER_REGISTER        0x0003   // Register address for power measurement

/**
 * @brief Reads a 16-bit value from a specified register of the PZEM-004T sensor.
 *
 * This function communicates with the PZEM-004T sensor over UART/RS485 using
 * the Modbus-RTU protocol to retrieve the value stored in a specified register.
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *                  This handle should be properly initialized before calling this function.
 * @param[in] reg The address of the register to be read.
 *
 * @return The 16-bit value read from the specified register.
 */
uint32_t PZEM_ReadRegister(UART_HandleTypeDef *huart, uint16_t reg);

/**
 * @brief Retrieves the voltage measurement from the PZEM-004T sensor.
 *
 * This function reads the voltage value from the corresponding register of the
 * PZEM-004T sensor and converts it to a floating-point value representing the voltage
 * in volts (V).
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The voltage in volts (V) as a floating-point value.
 */
float PZEM_GetVoltage(UART_HandleTypeDef *huart);

/**
 * @brief Retrieves the current measurement from the PZEM-004T sensor.
 *
 * This function reads the current value from the corresponding register of the
 * PZEM-004T sensor and converts it to a floating-point value representing the current
 * in amperes (A).
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The current in amperes (A) as a floating-point value.
 */
float PZEM_GetCurrent(UART_HandleTypeDef *huart);

/**
 * @brief Retrieves the power measurement from the PZEM-004T sensor.
 *
 * This function reads the power value from the corresponding register of the
 * PZEM-004T sensor and converts it to a floating-point value representing the power
 * in watts (W).
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The power in watts (W) as a floating-point value.
 */
float PZEM_GetPower(UART_HandleTypeDef *huart);

/**
 * @brief Calculates the CRC16 checksum for the Modbus communication.
 *
 * This function calculates the CRC16 checksum required for verifying data integrity
 * in Modbus-RTU communication.
 *
 * @param[in] data A pointer to the data buffer for which the CRC is to be calculated.
 * @param[in] len The length of the data buffer.
 *
 * @return The calculated CRC16 checksum as a 16-bit value.
 */
uint16_t CRC16(const uint8_t *data, uint16_t len);

#endif /* INC_PZEM004T_H_ */
