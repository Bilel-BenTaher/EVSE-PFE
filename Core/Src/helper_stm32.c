/*
 * helper_stm32.c
 *
 *  Created on: Jun 25, 2024
 *      Author: hp
 */

#include "stm32u5xx.h"
#include "helper_stm32.h"

// Initializations
float voltage_samples[ADC_SAMPLES];
volatile CONTROLPILOT_STM32_EVSE_MODE currentStatus = DISCONNECTED;
volatile CONTROLPILOT_STM32_EVSE_MODE lastStatus = DISCONNECTED;
volatile float  CurrentCPVoltage = 0;
volatile float  CurrentPPVoltage = 0;
volatile uint8_t currentAmpere = 0;
volatile uint8_t currentPower = 0;
volatile uint8_t currentVoltage = 0;
volatile uint16_t VsenseCurrent = 752;
volatile uint16_t previousTempArray[HELPER_STM32_MOVINGAVERAGE] = {415};
volatile uint8_t needsUpdate = 0;


// Function Implementations

void HELPER_STM32_initSystemVariables(void) {
    // Initialize system variables
    HELPER_STM32_setMaximumAmpere(32);
    lastStatus = DISCONNECTED;
    VsenseCurrent = 752;
    for (int i = 0; i < HELPER_STM32_MOVINGAVERAGE; i++) {
        previousTempArray[i] = 415;
    }
    needsUpdate = 0;
}

CONTROLPILOT_STM32_EVSE_MODE HELPER_STM32_getCurrentStatus(void) {
    return currentStatus;
}

void HELPER_STM32_setCurrentStatus(CONTROLPILOT_STM32_EVSE_MODE newCurrentStatus) {
    currentStatus = newCurrentStatus;
}

uint8_t HELPER_STM32_getCurrentAmpere(void) {
    return currentAmpere;
}

void HELPER_STM32_setCurrentAmpere(uint8_t newCurrentAmpere) {
    currentAmpere = newCurrentAmpere;
}
float HELPER_STM32_getCurrentCPVoltage(void) {
    return CurrentCPVoltage;
}

void HELPER_STM32_setCurrentCPVoltage(float newCurrentCPVoltage) {
	CurrentCPVoltage = newCurrentCPVoltage;
}

uint8_t HELPER_STM32_getCurrentPower(void) {
    return currentPower;
}

void HELPER_STM32_setCurrentPower(uint8_t newCurrentPower) {
    currentPower = newCurrentPower;
}

float HELPER_STM32_getCurrentVoltage(void) {
    return VoltageSensor_GetRMSVoltage(); // Ensure this function is defined elsewhere
}

int8_t HELPER_STM32_getCurrentTemp(void) {
    double TSCALraw = (double)(*TS_CAL1_ADDRPTR);
    double VsenseTScal = 3000.0 * TSCALraw / 4095.0;
    double Tdelta = (VsenseTScal - VsenseCurrent) / 2.5;
    int16_t Tresult = (int16_t)(300.0 + (Tdelta * 10.0));
    for (int i = 0; i < HELPER_STM32_MOVINGAVERAGE - 1; i++) {
        previousTempArray[i] = previousTempArray[i + 1];
    }
    previousTempArray[HELPER_STM32_MOVINGAVERAGE - 1] = Tresult;
    uint32_t temperatureAverage = 0;
    for (int i = 0; i < HELPER_STM32_MOVINGAVERAGE; i++) {
        temperatureAverage += previousTempArray[i];
    }
    int8_t Tfinal = (int8_t)(temperatureAverage / (10 * HELPER_STM32_MOVINGAVERAGE));
    if (temperatureAverage % 10 > 4) {
        Tfinal++;
    }
    return Tfinal;
}

void HELPER_STM32_setCurrentTemp(uint16_t newVsenseCurrent) {
    VsenseCurrent = newVsenseCurrent;
}

void HELPER_STM32_setNeedsUpdate(uint8_t newNeedsUpdate) {
    needsUpdate = newNeedsUpdate;
}

void HELPER_STM32_getSetting(void) {
    // Variables for filtered ADC values
    float ADC_filtered[4] = {0};
    int num_samples = 5; // Number of measurements for filtering
    int i;

    // Take multiple samples and calculate the average to avoid spurious values
    for (i = 0; i < num_samples; i++) {
        // Start ADC conversion
        HAL_ADC_Start(&hadc4);
        // Wait for conversion to complete
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        // Add the read value to the sum for filtering
        ADC_filtered[0] += HAL_ADC_GetValue(&hadc4);
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        ADC_filtered[1] += HAL_ADC_GetValue(&hadc4);
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        ADC_filtered[2] += HAL_ADC_GetValue(&hadc4);
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        ADC_filtered[3] += HAL_ADC_GetValue(&hadc4);
        // Stop ADC conversion
        HAL_ADC_Stop(&hadc4);
    }

    // Calculate the average values for each channel
    for (i = 1; i < 4; i++) {
        ADC_filtered[i] /= num_samples;
    }

    // Vdd calculation
    double Vrefint = (double)ADC_filtered[1];
    double Vrefint_cal_float = (double)(*VREFINT_CAL_ADDR);
    double Vddfloat = 3000.0 * Vrefint_cal_float / Vrefint;

    // Temperature calculation
    double Temprefint = (double)ADC_filtered[3];
    VsenseCurrent = Vddfloat * Temprefint / 4095.0;
    HELPER_STM32_setCurrentTemp(VsenseCurrent);
    int8_t MaximumTemp_int = HELPER_STM32_getCurrentTemp();

    // Current sensor
       rawVoltage = ((float)ADC_raw[2] * 3.3f * 2.0f / 4095.0f) * 1.035f;
       current = (rawVoltage - 2.5f) / sensitivity;
       HELPER_STM32_setCurrentAmpere(current);
}










