/*
 * @file PZEM004T.c
 * @brief Source file for interfacing the PZEM-004T sensor with STM32 via UART/RS485.
 *
 * This file implements the functions necessary to communicate with the PZEM-004T
 * power sensor using the Modbus-RTU protocol. The sensor provides readings of
 * voltage, current, and power.
 *
 * @date September 3, 2024
 * @author Bilel BENTAHER
 */

#include "PZEM004T.h"

/**
 * @brief Reads multiple registers from the PZEM-004T sensor.
 *
 * This function sends a Modbus-RTU request to the PZEM-004T sensor to read the specified
 * number of registers starting from the provided register address. The response is
 * validated using a CRC check, and the combined value of the registers is returned.
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 * @param[in] reg The starting register address to be read.
 * @param[in] numRegs The number of consecutive registers to read.
 *
 * @return The combined value of the registers read, or 0 in case of an error.
 */
uint32_t PZEM_ReadRegisters(UART_HandleTypeDef *huart, uint16_t reg, uint16_t numRegs) {
    uint8_t request[] = {
        PZEM_SLAVE_ADDRESS,
        0x04, // Modbus function code for reading input registers
        (reg >> 8) & 0xFF, // High byte of the register address
        reg & 0xFF,        // Low byte of the register address
        (numRegs >> 8) & 0xFF, // High byte of the number of registers
        numRegs & 0xFF,        // Low byte of the number of registers
        0x00, 0x00  // Placeholder for CRC (to be calculated later)
    };

    // Calculate CRC for the request
    uint16_t crc = CRC16(request, 6);
    request[6] = (crc >> 8) & 0xFF;  // High byte of CRC
    request[7] = crc & 0xFF;         // Low byte of CRC

    // Response buffer: 5 fixed bytes + 2 bytes per register
    uint8_t response[5 + 2 * numRegs];

    // Transmit the request using DMA
    if (HAL_UART_Transmit_DMA(huart, request, sizeof(request)) != HAL_OK) {
        return 0; // Handle transmission error
    }

    // Wait for the transmission to complete
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY);

    // Receive the response using DMA
    if (HAL_UART_Receive_DMA(huart, response, sizeof(response)) != HAL_OK) {
        return 0;  // Handle reception error
    }

    // Wait for the reception to complete
    while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY);

    // Validate the CRC of the response
    crc = CRC16(response, 3 + 2 * numRegs);
    if (crc != ((response[3 + 2 * numRegs] << 8) | response[4 + 2 * numRegs])) {
        return 0; // CRC error
    }

    // Combine the registers into a 32-bit value (if necessary)
    uint32_t value = 0;
    for (int i = 0; i < numRegs; i++) {
        value |= (response[3 + 2 * i] << 8) | response[4 + 2 * i];
        if (i < numRegs - 1) {
            value <<= 16;  // Shift if combining two registers
        }
    }

    return value;
}

/**
 * @brief Retrieves the voltage measurement from the PZEM-004T sensor.
 *
 * This function reads the voltage register from the PZEM-004T sensor and
 * converts the raw value into volts.
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The voltage in volts as a floating-point value.
 */
float PZEM_GetVoltage(UART_HandleTypeDef *huart) {
    uint16_t rawVoltage = PZEM_ReadRegisters(huart, PZEM_VOLTAGE_REGISTER, 1);
    return rawVoltage / 10.0f;  // Resolution: 1 LSB = 0.1V
}

/**
 * @brief Retrieves the current measurement from the PZEM-004T sensor.
 *
 * This function reads the current registers from the PZEM-004T sensor and
 * converts the raw value into amperes.
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The current in amperes as a floating-point value.
 */
float PZEM_GetCurrent(UART_HandleTypeDef *huart) {
    uint32_t rawCurrent = PZEM_ReadRegisters(huart, PZEM_CURRENT_REGISTER, 2);
    return rawCurrent / 1000.0f;  // Resolution: 1 LSB = 0.001A
}

/**
 * @brief Retrieves the power measurement from the PZEM-004T sensor.
 *
 * This function reads the power registers from the PZEM-004T sensor and
 * converts the raw value into watts.
 *
 * @param[in] huart A pointer to the UART handle (UART_HandleTypeDef) used for Modbus communication.
 *
 * @return The power in watts as a floating-point value.
 */
float PZEM_GetPower(UART_HandleTypeDef *huart) {
    uint32_t rawPower = PZEM_ReadRegisters(huart, PZEM_POWER_REGISTER, 2);
    return rawPower / 10.0f;  // Resolution: 1 LSB = 0.1W
}

/**
 * @brief Calculates the CRC16 checksum for Modbus-RTU communication.
 *
 * This function computes the CRC16 checksum for the given data buffer,
 * which is essential for ensuring data integrity in Modbus-RTU communication.
 *
 * @param[in] data A pointer to the data buffer for which the CRC is to be calculated.
 * @param[in] len The length of the data buffer.
 *
 * @return The calculated CRC16 checksum as a 16-bit value.
 */
uint16_t CRC16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;  // Initial CRC value
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];  // XOR with the data byte
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {  // If the LSB is 1
                crc >>= 1;
                crc ^= 0xA001;  // Apply the polynomial
            } else {
                crc >>= 1;  // Otherwise, just shift
            }
        }
    }
    return crc;
}
