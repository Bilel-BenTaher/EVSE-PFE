/*
 * userbutton_stm32.h
 *
 *  Created on: Jul 15, 2024
 *      Author: hp
 */

#ifndef INC_USERBUTTON_STM32_H_
#define INC_USERBUTTON_STM32_H_

#include "stm32u5xx_hal.h"

// USERBUTTON_STM32 library: This library enables three push-buttons for an STM32U5 chip.
// This library also handles the increase and decrease of charging current, as well as the setup
// mode to change the preset maximum value in the FLASH ROM.

// Global variables for debounce
#define DEBOUNCE_TIME_MS 50 // Debounce delay in milliseconds

// Function Declarations
void USERBUTTON_STM32_startTIM3(void);
void USERBUTTON_STM32_stopTIM3(void);
void StartDebounceTimer(void);
void StopDebounceTimer(void);
void EXTI2_3_IRQHandler(void);
void TIM7_IRQHandler(void);
void TIM3_IRQHandler(void);

#endif /* INC_USERBUTTON_STM32_H_ */
