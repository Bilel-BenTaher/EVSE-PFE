/*
 * state_management.c
 *
 *  Created on: Aug 15, 2024
 *      Author: hp
 */

#include "state_management.h"
#include "helper_stm32.h"
#include <stdlib.h>

// Global variables definition
// Initialize global variables with default values
bool systemReady = false;   // Flag indicating if the system is initialized and ready
bool state_A1 = false;      // Flag indicating the status of state A1
bool state_A2 = false;      // Flag indicating the status of state A2
bool state_B1 = false;      // Flag indicating the status of state B1
bool state_B2 = false;      // Flag indicating the status of state B2
bool state_C1 = false;      // Flag indicating the status of state C1
bool state_C2 = false;      // Flag indicating the status of state C2
bool state_E = false;       // Flag indicating an error state E
#define VREFINT_CAL_ADDRPTR  ((uint16_t*) ((uint32_t) 0x0BFA07A5))  // Address for Vrefint calibration
// Global variables for the module
static float sensitivity = 0.66f; // Sensitivity for the 30A model
static float rawVoltage = 0.0f;
static float current = 0.0f;
static uint32_t ADC_raw[3];  // Array to store raw ADC values

// Function to check state A1
// This function waits for a task notification and then starts an ADC conversion
void CheckStateA1() {
    state_A1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

// Function to check state A2
// This function waits for a task notification and then starts an ADC conversion
void CheckStateA2() {
    state_A2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

// Function to check and update state B1
// This function waits for a task notification and then starts an ADC conversion
void CheckStateB1() {
    state_B1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

// Function to check state B2
// This function waits for a task notification and then starts an ADC conversion
void CheckStateB2() {
    state_B2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

// Function to check state C1
// This function waits for a task notification and then starts an ADC conversion
void CheckStateC1() {
    state_C1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

// Function to check state C2
// This function waits for a task notification and then starts an ADC conversion
void CheckStateC2() {
    state_C2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();           // Start ADC conversion to measure voltage
}

void CONTROLPILOT_STM32_startADCConversion(void)
{
    // Start the ADC conversion
    HAL_ADC_Start(&hadc4);
    // Wait for the conversion sequence to finish
    for (uint8_t i = 0; i < 3; i++)
    {
        // Poll for the end of conversion
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        // Store the ADC value in the ADC_raw array
        ADC_raw[i] = HAL_ADC_GetValue(&hadc4);
    }
    // Stop the ADC conversion
    HAL_ADC_Stop(&hadc4);
    // Calculate Vdd using the internal reference calibration value
    float Vrefint = (float)ADC_raw[1];
    float Vrefint_cal_float = (float)(*VREFINT_CAL_ADDRPTR);
    float Vddfloat = 3000.0f * Vrefint_cal_float / Vrefint;

    // Calculate the voltage on the EVSE input
    float ADCVoltageFloat = (float)ADC_raw[0];
    float cpVoltageFloat = Vddfloat * ADCVoltageFloat / 4095.0f;
    HELPER_STM32_setCurrentCPVoltage(cpVoltageFloat);

    // Current sensor
    rawVoltage = ((float)ADC_raw[2] * 3.3f * 2.0f / 4095.0f) * 1.035f;
    current = (rawVoltage - 2.5f) / sensitivity;
    HELPER_STM32_setCurrentAmpere(current);
}

void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, float dutyCycle)
{
    // Declare and initialize a structure to configure PWM settings
    TIM_OC_InitTypeDef sConfigOC = {0};
    // Calculate the pulse value based on the desired duty cycle and timer period
    uint32_t pulse = (uint32_t)(dutyCycle * (htim->Init.Period + 1));
    // Configure the PWM settings
    sConfigOC.OCMode = TIM_OCMODE_PWM1;             // Set PWM mode to Mode 1
    sConfigOC.Pulse = pulse;                        // Set the pulse value (duty cycle)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;     // Set the polarity to high
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;   // Set the complementary polarity to high
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;      // Disable fast mode
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;  // Set idle state to reset
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;// Set complementary idle state to reset
    // Configure the PWM channel with the new pulse value
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
    {
        // Handle errors if the configuration fails
        Error_Handler();
    }
    // Restart the PWM to apply the new settings
    if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        // Handle errors if the start operation fails
        Error_Handler();
    }
}



