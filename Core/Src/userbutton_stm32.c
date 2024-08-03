/*
 * userbutton_stm32.c
 *
 *  Created on: Jul 15, 2024
 *      Author: hp
 */

#include "stm32u5xx_hal.h"
#include "userbutton_stm32.h"
#include "helper_stm32.h"
#include "oled_stm32_ssd1306.h"

// External TIM handles
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;

// External GPIO definitions
extern GPIO_TypeDef* USERBUTTON_GPIO_PORT;
extern uint16_t USERBUTTON_GPIO_ENTER_PIN;

// Global variables for debounce
static volatile uint32_t debounce_counter = 0;
static volatile uint8_t debounce_active = 0;

void USERBUTTON_STM32_startTIM3(void) {
    HAL_TIM_Base_Start(&htim3); // Start TIM3
}

void USERBUTTON_STM32_stopTIM3(void) {
    HAL_TIM_Base_Stop(&htim3);  // Stop TIM3
}

// Function to start the debounce timer
void StartDebounceTimer(void) {
    HAL_TIM_Base_Start_IT(&htim7); // Start TIM7 with interrupt
}

// Function to stop the debounce timer
void StopDebounceTimer(void) {
    HAL_TIM_Base_Stop_IT(&htim7);  // Stop TIM7 with interrupt
    debounce_active = 0;
}

// EXTI interrupt handler
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (!debounce_active)
    {
        debounce_active = 1;
        StartDebounceTimer();
    }
    else
    {
        __NOP();
    }
}

// TIM7 interrupt handler for debounce
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if (htim==&htim7) {
         StopDebounceTimer();

    // Check button status after debounce timeout
    if (HAL_GPIO_ReadPin(USERBUTTON_GPIO_PORT, USERBUTTON_GPIO_ENTER_PIN) == GPIO_PIN_RESET) {
        // ENTER has been released
        USERBUTTON_STM32_stopTIM3();
        CONTROLPILOT_STM32_setLow();
        CONTROLPILOT_STM32_timerHighStop();
        CONTROLPILOT_STM32_timerLowStop();
        CONTROLPILOT_STM32_contactorOff();
        DIODE_STM32_SET_LED_GREEN_LOW();
        DIODE_STM32_SET_LED_RED_LOW();
        DIODE_STM32_SET_LED_BLUE_LOW();
    } else {
        // ENTER was pressed
        USERBUTTON_STM32_startTIM3();
        DIODE_STM32_SET_LED_GREEN_HIGH();
        OLED_STM32_updateMain_BienvenueView();
        CONTROLPILOT_STM32_timerHighStart();
    }
        }
}

// TIM3 interrupt handler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if (htim==&htim3) {
    OLED_STM32_updateMainView();
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE); // Clear TIM3 update interrupt flag
}
}

