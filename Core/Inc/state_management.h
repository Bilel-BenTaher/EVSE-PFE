/*
 * state_management.h
 *
 *  Created on: Aug 15, 2024
 *      Author: hp
 */

#ifndef INC_STATE_MANAGEMENT_H_
#define INC_STATE_MANAGEMENT_H_

#include <stdbool.h> // Include for boolean type support

// Global variable declarations
// These variables are used to track the state of the system and other parameters
extern bool systemReady;  // Indicates if the system is fully initialized and ready
extern bool state_A1;    // Represents the state of component A1
extern bool state_A2;    // Represents the state of component A2
extern bool state_B1;    // Represents the state of component B1
extern bool state_B2;    // Represents the state of component B2
extern bool state_C1;    // Represents the state of component C1
extern bool state_C2;    // Represents the state of component C2
extern bool state_E;     // Represents the state of component E

// Macros for controlling GPIO pins
#define CONTROLPILOT_STM32_contactorOn()   HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_CTCTR_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_CTCTR_PIN_Pin, GPIO_PIN_SET)
#define CONTROLPILOT_STM32_contactorOff()  HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_CTCTR_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_CTCTR_PIN_Pin, GPIO_PIN_RESET)

// Function declarations
// These functions will be used to check the status of various system states
void CheckStateA1(void);  // Function to check the state of component A1
void CheckStateB1(void);  // Function to check the state of component B1
void CheckStateB2(void);  // Function to check the state of component B2
void CONTROLPILOT_STM32_startADCConversion(void);
void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, float dutyCycle); //Function to Configure the PWM Duty Cycle

#endif /* INC_STATE_MANAGEMENT_H_ */
