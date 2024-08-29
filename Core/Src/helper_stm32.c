/*
 * @file helper_stm32.c
 * @brief Implementation of helper functions for managing current, voltage, and temperature on STM32U5XX.
 *
 * This file provides the implementation of functions for setting and retrieving values related to
 * current amperage, CP voltage, and temperature. It also initializes the global variables used
 * to store these values.
 *
 * @date June 25, 2024
 * @author Bilel BENTAHER
 */

#include "stm32u5xx_hal.h"
#include "helper_stm32.h"

// Initializations
float currentAmpere = 0.0f;    /**< @brief Current amperage initialized to 0. */
float CurrentCPVoltage = 0.0f; /**< @brief Current CP voltage initialized to 0.0 volts. */
float CurrentTemp = 0.0f;     /**< @brief Current temperature initialized to 0 degrees Celsius. */

// Function Implementations

/**
 * @brief Sets the current amperage.
 *
 * This function updates the global variable `currentAmpere` with a new value.
 *
 * @param newCurrentAmpere The new amperage value  .
 */
void HELPER_STM32_setCurrentAmpere(float newCurrentAmpere) {
    currentAmpere = newCurrentAmpere; // Update the current amperage
}

/**
 * @brief Retrieves the current amperage.
 *
 * This function returns the current value of `currentAmpere`.
 *
 * @return The current amperage value .
 */
float HELPER_STM32_getCurrentAmpere(void) {
    return currentAmpere; // Return the current amperage value
}

/**
 * @brief Sets the current CP voltage.
 *
 * This function updates the global variable `CurrentCPVoltage` with a new floating-point value.
 *
 * @param newCurrentCPVoltage The new CP voltage value in volts.
 */
void HELPER_STM32_setCurrentCPVoltage(float newCurrentCPVoltage) {
    CurrentCPVoltage = newCurrentCPVoltage; // Update the current CP voltage
}

/**
 * @brief Retrieves the current CP voltage.
 *
 * This function returns the current value of `CurrentCPVoltage`.
 *
 * @return The current CP voltage value in volts.
 */
float HELPER_STM32_getCurrentCPVoltage(void) {
    return CurrentCPVoltage; // Return the current CP voltage value
}

/**
 * @brief Sets the current temperature.
 *
 * This function updates the global variable `CurrentTemp` with a new value.
 *
 * @param newCurrentTemp The new temperature value in degrees Celsius.
 */
void HELPER_STM32_setCurrentTemp(float newCurrentTemp) {
    CurrentTemp = newCurrentTemp; // Update the current temperature
}

/**
 * @brief Retrieves the current temperature.
 *
 * This function returns the current value of `CurrentTemp`.
 *
 * @return The current temperature value in degrees Celsius.
 */
float HELPER_STM32_getCurrentTemp(void) {
    return CurrentTemp; // Return the current temperature value
}
