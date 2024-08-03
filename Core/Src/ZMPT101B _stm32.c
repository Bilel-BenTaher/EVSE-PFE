/*
 * ZMPT101B _stm32.c
 *
 *  Created on: Jul 30, 2024
 *      Author: hp
 */

#include "ZMPT101B _stm32.h"
#include <math.h> // Pour la fonction sqrt
// Buffer to store ADC samples
static uint16_t adcBuffer[ADC_BUFFER_SIZE];

// Simple low-pass filter IIR (Infinite Impulse Response)
static float lowPassFilter(float input, float previous, float alpha) {
    return alpha * input + (1.0f - alpha) * previous;
}

// Function to calculate the RMS voltage
float VoltageSensor_GetRMSVoltage(void) {
	 uint32_t sumOfSquares = 0;
	 float filteredValue = 3998.0f; // Valeur numérique approximative correspondante 230v
	 float alpha = 0.1f; // Coefficient de filtrage passe-bas
	    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
	    // Start ADC conversion
	    HAL_ADC_Start(&hadc1);
	    // Wait for conversion to complete
	    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
	    // Get the ADC value
	    adcBuffer[i] = HAL_ADC_GetValue(&hadc1);
	        }
	    // Stop ADC conversion
	    HAL_ADC_Stop(&hadc1);
	    // Apply low-pass filter
	    float voltage = (adcBuffer[i] * V_REF) / ADC_MAX_VALUE;
	    filteredValue = lowPassFilter(voltage, filteredValue, alpha);
	    sumOfSquares += filteredValue * filteredValue;
	    }

	    // Calculate RMS value
	    float rms = sqrt(sumOfSquares / ADC_BUFFER_SIZE);
	    return rms / ZMPT101B_SENSITIVITY; // Convert to actual voltage
}

