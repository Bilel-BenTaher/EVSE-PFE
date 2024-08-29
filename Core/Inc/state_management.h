/*
 * @file state_management.h
 * @brief Header file for state management functions and global variables.
 *
 * This file contains declarations for the management of system states, including the states of several components
 * and GPIO pin control. It provides function declarations for checking system states and managing hardware components.
 *
 * @date August 15, 2024
 * @author Bilel BENTAHER */

#ifndef INC_STATE_MANAGEMENT_H_
#define INC_STATE_MANAGEMENT_H_

#include <stdbool.h> // Standard library for boolean data type support
#include <stdint.h>  // Standard library for fixed-width integer types

// Global variable declarations
// These variables are used to track the status of the system and different components

extern bool state_A1;     /**< @brief Represents the operational state of component A1. */
extern bool state_A2;     /**< @brief Represents the operational state of component A2. */
extern bool state_B1;     /**< @brief Represents the operational state of component B1. */
extern bool state_B2;     /**< @brief Represents the operational state of component B2. */
extern bool state_C1;     /**< @brief Represents the operational state of component C1. */
extern bool state_C2;     /**< @brief Represents the operational state of component C2. */

// Macros for controlling GPIO pins
// These macros are used to toggle GPIO pins to control hardware components

/**
 * @brief Activates the contactor.
 *
 * This macro sets the appropriate GPIO pin to enable the high-voltage contactor.
 */
#define HIGHVOLTAGE_STM32_contactorOn()   HAL_GPIO_WritePin(HIGHVOLTAGE_STM32_CTCTR_PIN_GPIO_Port, HIGHVOLTAGE_STM32_CTCTR_PIN_Pin, GPIO_PIN_SET)

/**
 * @brief Deactivates the contactor.
 *
 * This macro resets the appropriate GPIO pin to disable the high-voltage contactor.
 */
#define HIGHVOLTAGE_STM32_contactorOff()  HAL_GPIO_WritePin(HIGHVOLTAGE_STM32_CTCTR_PIN_GPIO_Port, HIGHVOLTAGE_STM32_CTCTR_PIN_Pin, GPIO_PIN_RESET)

// Function declarations
// These functions are used to monitor and manage the states of various components in the system

/**
 * @brief Checks the state of component A1.
 *
 * This function verifies the operational state of component A1 and updates the global variable `state_A1`.
 */
void CheckStateA1(void);

/**
 * @brief Checks the state of component A2.
 *
 * This function verifies the operational state of component A2 and updates the global variable `state_A2`.
 */
void CheckStateA2(void);

/**
 * @brief Checks the state of component B1.
 *
 * This function verifies the operational state of component B1 and updates the global variable `state_B1`.
 */
void CheckStateB1(void);

/**
 * @brief Checks the state of component B2.
 *
 * This function verifies the operational state of component B2 and updates the global variable `state_B2`.
 */
void CheckStateB2(void);

/**
 * @brief Checks the state of component C1.
 *
 * This function verifies the operational state of component C1 and updates the global variable `state_C1`.
 */
void CheckStateC1(void);

/**
 * @brief Checks the state of component C2.
 *
 * This function verifies the operational state of component C2 and updates the global variable `state_C2`.
 */
void CheckStateC2(void);

/**
 * @brief Applies a low-pass filter to smooth the input signal.
 *
 * This function uses a simple low-pass filter formula to smoothen out the input signal.
 * It is useful for eliminating high-frequency noise in the voltage readings.
 *
 * @param[in] input The current input value to the filter.
 * @param[in] previous The previous filtered value.
 * @param[in] alpha The smoothing factor, which determines how much influence the previous value has.
 *                  (Typical values are between 0 and 1, where a smaller value makes the filter slower.)
 *
 * @return The filtered value.
 */
float lowPassFilter(float input, float previous, float alpha);

/**
 * @brief Starts an ADC conversion with DMA mode on four channels.
 *
 * This function initiates an ADC conversion in DMA mode on the ADC4 peripheral.
 * It configures the ADC to convert data from four channels and stores the results
 * in the buffer pointed to by `ADC_raw`.
 *
 * @note This function assumes that the `ADC_raw` buffer is correctly initialized
 *       and that DMA is properly configured for ADC4.
 */
void CONTROLPILOT_STM32_startADCConversion(void);

/**
 * @brief Configures the PWM duty cycle for a specific timer and channel.
 *
 * @param htim Pointer to the TIM_HandleTypeDef structure that contains the configuration information for TIM module.
 * @param Channel Specifies the TIM channel.
 * @param dutyCycle A float value between 0 and 100 representing the percentage of the PWM duty cycle.
 *
 * This function configures the PWM signal for the specified timer and channel.
 */
void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, float dutyCycle);

#endif /* INC_STATE_MANAGEMENT_H_ */
