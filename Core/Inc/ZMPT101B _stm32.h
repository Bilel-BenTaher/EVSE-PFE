/*
 * @file ZMPT101B_stm32.h
 * @brief Header file for interfacing the ZMPT101B voltage sensor with STM32 microcontroller.
 *
 * This file contains the necessary configurations, constants, and function prototypes
 * for reading and processing voltage values using the ZMPT101B sensor with an STM32U5 microcontroller.
 * It includes an implementation of a low-pass filter and a function to calculate the RMS voltage.
 *
 * @date July 30, 2024
 * @author hp
 */

#ifndef INC_ZMPT101B__STM32_H_
#define INC_ZMPT101B__STM32_H_

#include "stm32u5xx_hal.h"

// Configuration and definitions

/** @brief Buffer size for storing ADC values from the voltage sensor. */
#define ADC_BUFFER_SIZE      1024    // 1024 samples in the ADC buffer

/** @brief Reference voltage used for ADC conversions (in volts). */
#define V_REF                3.3     // 3.3V reference voltage for the ADC

/** @brief Maximum ADC value corresponding to the 12-bit resolution (0-4095). */
#define ADC_MAX_VALUE        4095    // Maximum possible ADC value (2^12 - 1)

/** @brief Sensitivity of the ZMPT101B voltage sensor (in V per volt). */
#define ZMPT101B_SENSITIVITY  0.014   // 14 mV per volt sensitivity of the ZMPT101B sensor


// Function Prototypes

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
static float lowPassFilter(float input, float previous, float alpha);

/**
 * @brief Retrieves the RMS (Root Mean Square) voltage measured by the ZMPT101B sensor.
 *
 * This function processes the ADC readings from the ZMPT101B voltage sensor, applies necessary
 * transformations, and returns the calculated RMS voltage. The RMS voltage is typically used
 * to measure AC voltage.
 *
 * @return The RMS voltage as a 16-bit unsigned integer.
 */
uint16_t VoltageSensor_GetRMSVoltage(void);

#endif /* INC_ZMPT101B__STM32_H_ */
