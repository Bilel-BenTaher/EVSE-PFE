/*
 * @file ZMPT101B_stm32.c
 * @brief Source file for interfacing the ZMPT101B voltage sensor with STM32 microcontroller.
 *
 * This file contains the implementation of functions used to read ADC values from the ZMPT101B voltage sensor,
 * apply a low-pass filter, and calculate the RMS voltage. The low-pass filter helps to smooth out noise in the signal.
 *
 * @date July 30, 2024
 * @author hp
 */

#include "ZMPT101B _stm32.h"
#include <math.h>  // For the sqrt function

// Buffer to store ADC samples
static uint16_t adcBuffer[ADC_BUFFER_SIZE];

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
static float lowPassFilter(float input, float previous, float alpha) {
    return alpha * input + (1.0f - alpha) * previous;
}

/**
 * @brief Computes the RMS (Root Mean Square) voltage from ADC readings.
 *
 * This function captures ADC values from the ZMPT101B voltage sensor, applies a low-pass filter to reduce noise,
 * and computes the RMS voltage. The RMS value is calculated from the square root of the sum of squares of the
 * filtered voltage values, which provides an accurate measure of AC voltage.
 *
 * @note This function assumes a steady voltage signal. The RMS voltage is returned as an integer value, adjusted
 * by the sensor's sensitivity.
 *
 * @return The RMS voltage (scaled by the sensor's sensitivity).
 */
uint16_t VoltageSensor_GetRMSVoltage(void) {
    uint32_t sumOfSquares = 0;       // Accumulator for the sum of squared voltages
    float filteredValue = 3998.0f;   // Initial guess for the filtered voltage value
    float alpha = 0.1f;              // Low-pass filter coefficient (smoothing factor)

    // Loop over ADC buffer to collect and process voltage readings
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
        // Start ADC conversion
        HAL_ADC_Start(&hadc1);

        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            // Get the ADC value and store it in the buffer
            adcBuffer[i] = HAL_ADC_GetValue(&hadc1);
        }

        // Stop ADC conversion
        HAL_ADC_Stop(&hadc1);

        // Convert ADC value to voltage
        float voltage = (adcBuffer[i] * V_REF) / ADC_MAX_VALUE;

        // Apply the low-pass filter to the voltage reading
        filteredValue = lowPassFilter(voltage, filteredValue, alpha);

        // Accumulate the square of the filtered voltage for RMS calculation
        sumOfSquares += filteredValue * filteredValue;
    }

    // Calculate the RMS voltage from the sum of squares
    float rms = sqrt(sumOfSquares / ADC_BUFFER_SIZE);

    // Convert the RMS voltage to the actual value using the sensor's sensitivity
    return (uint16_t)(rms / ZMPT101B_SENSITIVITY);
}
