/*
 * @file state_management.c
 * @brief Source file for state management and hardware interaction functions.
 *
 * This file contains the implementation of functions for managing the state of various system components,
 * handling ADC conversions, controlling PWM signals, and filtering sensor data on an STM32 microcontroller.
 *
 * @date August 15, 2024
 * @author Bilel BENTAHER
 */

#include "state_management.h"
#include "helper_stm32.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
#include "Diode_led.h"
#include "oled_stm32_ssd1306.h"
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim16;

// Global variable definitions
// Initialize global variables with default values
bool state_A1 = false;      /**< @brief State flag for component A1 */
bool state_A2 = false;      /**< @brief State flag for component A2 */
bool state_B1 = false;      /**< @brief State flag for component B1 */
bool state_B2 = false;      /**< @brief State flag for component B2 */
bool state_C1 = false;      /**< @brief State flag for component C1 */
bool state_C2 = false;      /**< @brief State flag for component C2 */

// Addresses for calibration values in system memory
#define TS_CAL1_ADDR ((uint16_t*) 0x0BFA0710) // ADC value at 30°C
#define TS_CAL2_ADDR ((uint16_t*) 0x0BFA0742) // ADC value at 110°C

// Constants for the known temperatures
#define TEMP1 (30.0f)   // T1 = 30°C
#define TEMP2 (130.0f)  // T2 = 110°C

// Global variables for the module
static uint16_t ADC_raw[3]; /**< @brief Array to store raw ADC values (4 elements for different sensor readings) */

// Function definitions

/**
 * @brief Checks the state of component A1.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateA1() {
    state_A1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Checks the state of component A2.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateA2() {
    state_A2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Checks the state of component B1.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateB1() {
    state_B1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Checks the state of component B2.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateB2() {
    state_B2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Checks the state of component C1.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateC1() {
    state_C1 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Checks the state of component C2.
 *
 * This function waits for a task notification and initiates an ADC conversion to measure voltage.
 */
void CheckStateC2() {
    state_C2 = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for a task notification
    CONTROLPILOT_STM32_startADCConversion();             // Start ADC conversion to measure voltage
}

/**
 * @brief Simple low-pass filter for smoothing voltage readings.
 *
 * This function implements an Infinite Impulse Response (IIR) low-pass filter. It smooths the input signal
 * by blending the current input with the previously filtered value. The coefficient `alpha` controls the
 * "weight" given to the current input vs. the previous output, effectively controlling the smoothing factor.
 *
 * @param[in] input The current input value to the filter (the current voltage reading).
 * @param[in] previous The previously filtered output value.
 * @param[in] alpha The filter coefficient, with values between 0 and 1 (a lower value slows the filter response).
 *
 * @return The filtered output value.
 */
float lowPassFilter(float input, float previous, float alpha) {
    return alpha * input + (1.0f - alpha) * previous;
}

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

void CONTROLPILOT_STM32_startADCConversion(void) {
    // Start the ADC in DMA mode, converting 4 channels
    HAL_ADC_Start_DMA(&hadc4,(uint32_t*)ADC_raw, 4);
    // Wait for all conversions to complete
    while (HAL_ADC_GetState(&hadc4) != HAL_ADC_STATE_READY) {
    	// Nothing to do here, just wait for the ADC to finish
       }
}

/**
 * @brief Callback function executed when ADC conversion is complete.
 *
 * This function processes the ADC conversion results from multiple channels
 * and performs the following operations:
 *
 * - Calculates the supply voltage (Vdd) using the internal reference voltage.
 * - Computes the voltage on the Control Pilot (CP) line and applies a filtering
 *   algorithm to update the voltage value in the system.
 * - Calculates the current from the current sensor(ACS712-30A), applies filtering, and updates
 *   the current value in the system.
 * - Computes the temperature from the temperature sensor, applies filtering, and
 *   updates the temperature value in the system.
 *
 * After processing the ADC data, the function stops the ADC and DMA to conclude
 * the conversion process.
 *
 * @param hadc Pointer to the ADC handle structure that contains the configuration
 *             information for the specified ADC.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // Constants and static variables for filtering and calculations
    static const float alpha = 0.9f;                 // High alpha value for minimal filtering effect
    static float previousFilteredVoltage = 12.0f;   // Initial filtered voltage value (in volts)
    static float previousFilteredTemp = 30.0f;      // Initial filtered temperature value (in degrees Celsius)
    static float Vddfloat = 3.0f;                   //vdd
    float ts_cal1 = (float)(*TS_CAL1_ADDR);
    float ts_cal2 = (float)(*TS_CAL2_ADDR);
    // Read the raw ADC values
    uint16_t rawADC_CP = ADC_raw[0];   // Raw ADC value for Control Pilot voltage
    uint16_t rawTempSensor = ADC_raw[1];   // Raw ADC value for temperature sensor

    // Calculate the voltage on the Control Pilot (CP) line
    float cpVoltageFloat = Vddfloat * (float)rawADC_CP / 4095.0f;  // Convert ADC value to actual CP voltage
    float filteredVoltage = lowPassFilter(cpVoltageFloat, previousFilteredVoltage, alpha);  // Apply filtering
    previousFilteredVoltage = filteredVoltage;  // Update the previous filtered voltage value
    HELPER_STM32_setCurrentCPVoltage(filteredVoltage);  // Update the CP voltage in the system

    // Calculate the temperature from the temperature sensor
    float Temp_ADC= Vddfloat * (float)rawTempSensor / 4095.0f;  // Convert ADC value to temperature
    float currentTemperature = ((Temp_ADC- ts_cal1) * (TEMP2 - TEMP1) / (ts_cal2 - ts_cal1)) + TEMP1; // Calculate the temperature using linear interpolation
    float filteredTemp = lowPassFilter(currentTemperature, previousFilteredTemp, alpha);  // Apply filtering
    previousFilteredTemp = filteredTemp;  // Update the previous filtered temperature value
    HELPER_STM32_setCurrentTemp(filteredTemp);  // Update the temperature in the system

    // Stop ADC and DMA after conversion
    HAL_ADC_Stop_DMA(&hadc4);
}

/**
 * @brief Configures the PWM duty cycle for a specific timer and channel.
 *
 * @param htim Pointer to the TIM_HandleTypeDef structure containing timer configuration.
 * @param Channel Specifies the TIM channel to configure.
 * @param dutyCycle A float value between 0 and 100 representing the PWM duty cycle percentage.
 *
 * This function configures the PWM signal for the specified timer and channel,
 * controlling the duty cycle of the PWM signal to drive motors, LEDs, or other PWM-driven devices.
 */
void SetPWMDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, float dutyCycle) {
    // Declare and initialize the configuration structure for PWM
    TIM_OC_InitTypeDef sConfigOC = {0};
    // Calculate the pulse value based on the desired duty cycle and timer period
    uint32_t pulse = (uint32_t)(dutyCycle * (htim->Init.Period + 1));
    // Configure the PWM mode and settings
    sConfigOC.OCMode = TIM_OCMODE_PWM1;              // Set PWM mode to Mode 1
    sConfigOC.Pulse = pulse;                         // Set the pulse value (duty cycle)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;      // Set polarity to high
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;    // Set complementary polarity to high
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // Disable fast mode
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // Set idle state to reset
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // Set complementary idle state to reset

    // Configure the PWM channel with the new settings
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK) {
        // Handle error if configuration fails
        Error_Handler();
    }
}

/**
 * @brief ADC Analog Watchdog Callback
 *
 * This callback function is called when the ADC detects a value that goes out of the defined
 * threshold range (configured using the Analog Watchdog feature).
 * It handles the interrupt triggered by the ADC's Analog Watchdog (AWD) when the voltage
 * level crosses the predefined high or low threshold.
 *
 * @param hadc Pointer to the ADC handle structure that contains the configuration
 *             information for the specified ADC.
 *
 * This function checks if the ADC's Analog Watchdog flag is set, indicating that the
 * monitored voltage has gone outside the specified window. If the flag is set,
 * the function clears the flag to prevent re-entering the interrupt.
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	// Open the contactor (critical process)
	 HIGHVOLTAGE_STM32_contactorOff();
	HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1); //low output
	Error_Handler();

}
