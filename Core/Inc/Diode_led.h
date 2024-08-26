/*
 * @file Diode_led.h
 * @brief Header file for controlling RGB LEDs using GPIO pins on STM32U5XX.
 *
 * This file defines macros for setting the state of RGB LEDs connected to GPIO pins.
 * The macros use STM32 HAL library functions to control the LEDs' color by setting or resetting
 * the corresponding GPIO pins.
 *
 * @date August 22, 2024
 * @author hp
 */

#ifndef INC_DIODE_LED_H_
#define INC_DIODE_LED_H_

#include "stm32u5xx_hal.h" // Include the HAL library header for STM32U5XX

// Macros for controlling LED colors using GPIO pins

/**
 * @brief Sets the red LED pin to high (turns on the red LED).
 *
 * This macro sets the GPIO pin connected to the red LED to a high state, turning on the red LED.
 */
#define SET_DIODE_LED_RED_HIGH()    HAL_GPIO_WritePin(DIODE_LED_GPIO_RED_PIN_GPIO_Port, DIODE_LED_GPIO_RED_PIN_Pin, GPIO_PIN_SET)

/**
 * @brief Sets the green LED pin to high (turns on the green LED).
 *
 * This macro sets the GPIO pin connected to the green LED to a high state, turning on the green LED.
 */
#define SET_DIODE_LED_GREEN_HIGH()  HAL_GPIO_WritePin(DIODE_LED_GPIO_GREEN_PIN_GPIO_Port, DIODE_LED_GPIO_GREEN_PIN_Pin, GPIO_PIN_SET)

/**
 * @brief Sets the blue LED pin to high (turns on the blue LED).
 *
 * This macro sets the GPIO pin connected to the blue LED to a high state, turning on the blue LED.
 */
#define SET_DIODE_LED_BLUE_HIGH()   HAL_GPIO_WritePin(DIODE_LED_GPIO_BLUE_PIN_GPIO_Port, DIODE_LED_GPIO_BLUE_PIN_Pin, GPIO_PIN_SET)

/**
 * @brief Sets the red LED pin to low (turns off the red LED).
 *
 * This macro sets the GPIO pin connected to the red LED to a low state, turning off the red LED.
 */
#define SET_DIODE_LED_RED_LOW()     HAL_GPIO_WritePin(DIODE_LED_GPIO_RED_PIN_GPIO_Port, DIODE_LED_GPIO_RED_PIN_Pin, GPIO_PIN_RESET)

/**
 * @brief Sets the green LED pin to low (turns off the green LED).
 *
 * This macro sets the GPIO pin connected to the green LED to a low state, turning off the green LED.
 */
#define SET_DIODE_LED_GREEN_LOW()   HAL_GPIO_WritePin(DIODE_LED_GPIO_GREEN_PIN_GPIO_Port, DIODE_LED_GPIO_GREEN_PIN_Pin, GPIO_PIN_RESET)

/**
 * @brief Sets the blue LED pin to low (turns off the blue LED).
 *
 * This macro sets the GPIO pin connected to the blue LED to a low state, turning off the blue LED.
 */
#define SET_DIODE_LED_BLUE_LOW()    HAL_GPIO_WritePin(DIODE_LED_GPIO_BLUE_PIN_GPIO_Port, DIODE_LED_GPIO_BLUE_PIN_Pin, GPIO_PIN_RESET)

#endif /* INC_DIODE_LED_H_ */
