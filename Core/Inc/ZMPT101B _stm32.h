/*
 * ZMPT101B _stm32.h
 *
 *  Created on: Jul 30, 2024
 *      Author: hp
 */

#ifndef INC_ZMPT101B__STM32_H_
#define INC_ZMPT101B__STM32_H_

#include "stm32u5xx_hal.h"

// Configuration and definitions
#define ADC_BUFFER_SIZE  1024   // Size of the buffer for ADC values
#define V_REF            3.3    // Reference voltage (V)
#define ADC_MAX_VALUE    4095   // Maximum ADC value (12-bit resolution)
#define ZMPT101B_SENSITIVITY  0.014   // Sensitivity (V per volt) of the ZMPT101B

// Function prototypes
static float lowPassFilter(float input, float previous, float alpha);
float VoltageSensor_GetRMSVoltage(void);

#endif /* INC_ZMPT101B__STM32_H_ */
