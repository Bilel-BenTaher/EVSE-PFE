/*
 * @file helper_stm32.h
 * @brief Header file for the HELPER_STM32 library.
 *
 * This library provides helper functions and variable definitions for managing current measurements,
 * voltage readings, and temperature data on an STM32U5XX microcontroller. The functions allow for
 * setting and retrieving values related to current, voltage, and temperature.
 *
 * @date June 25, 2024
 * @author hp
 */

#ifndef HELPER_STM32_H_
#define HELPER_STM32_H_

#include <stdint.h>  // For standard integer types

// Variable Declarations
/**
 * @brief Current amperage in amperes.
 *
 * This variable holds the current amperage as a value between 0 and 255.
 * The `volatile` keyword indicates that the variable may be changed by hardware or other processes.
 */
extern volatile uint8_t currentAmpere;

/**
 * @brief Current CP voltage in volts.
 *
 * This variable holds the current CP (Control Pilot) voltage as a floating-point value to accommodate fractional voltages.
 */
extern volatile float CurrentCPVoltage;

/**
 * @brief Current temperature in degrees Celsius.
 *
 * This variable holds the current temperature as an unsigned 16-bit integer. It provides a larger range of values.
 */
extern volatile uint16_t CurrentTemp;

// Function Declarations
/**
 * @brief Sets the current amperage.
 *
 * This function sets the value of the current amperage.
 *
 * @param newCurrentAmpere The new amperage value to be set (0-255).
 */
void HELPER_STM32_setCurrentAmpere(uint8_t newCurrentAmpere);

/**
 * @brief Retrieves the current amperage.
 *
 * This function gets the current amperage value.
 *
 * @return The current amperage value (0-255).
 */
uint8_t HELPER_STM32_getCurrentAmpere(void);

/**
 * @brief Sets the current CP voltage.
 *
 * This function sets the value of the current CP voltage.
 *
 * @param newCurrentCPVoltage The new CP voltage value in volts.
 */
void HELPER_STM32_setCurrentCPVoltage(float newCurrentCPVoltage);

/**
 * @brief Retrieves the current CP voltage.
 *
 * This function gets the current CP voltage value.
 *
 * @return The current CP voltage value in volts.
 */
float HELPER_STM32_getCurrentCPVoltage(void);

/**
 * @brief Sets the current temperature.
 *
 * This function sets the value of the current temperature.
 *
 * @param newCurrentTemp The new temperature value in degrees Celsius.
 */
void HELPER_STM32_setCurrentTemp(uint16_t newCurrentTemp);

/**
 * @brief Retrieves the current temperature.
 *
 * This function gets the current temperature value.
 *
 * @return The current temperature value in degrees Celsius.
 */
uint16_t HELPER_STM32_getCurrentTemp(void);

#endif /* HELPER_STM32_H_ */

