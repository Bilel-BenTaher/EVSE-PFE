/*
 * @file helper_stm32.h
 * @brief Header file for the HELPER_STM32 library.
 *
 * This library provides helper functions and variable definitions for managing current measurements,
 * voltage readings, and temperature data on an STM32U5XX microcontroller. The functions allow for
 * setting and retrieving values related to current, voltage, and temperature.
 *
 * @date June 25, 2024
 * @author Bilel BENTAHER
 */

#ifndef HELPER_STM32_H_
#define HELPER_STM32_H_

#include <stdint.h>  // For standard integer types

// Variable Declarations

/**
 * @brief Current CP voltage in volts.
 *
 * This variable holds the current CP (Control Pilot) voltage .
 */
extern float CurrentCPVoltage;

/**
 * @brief Current temperature in degrees Celsius.
 *
 * This variable holds the current temperature .
 */
extern float CurrentTemp;

// Function Declarations

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
void HELPER_STM32_setCurrentTemp(float newCurrentTemp);

/**
 * @brief Retrieves the current temperature.
 *
 * This function gets the current temperature value.
 *
 * @return The current temperature value in degrees Celsius.
 */
float HELPER_STM32_getCurrentTemp(void);

#endif /* HELPER_STM32_H_ */

