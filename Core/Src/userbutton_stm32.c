/*
 * userbutton_stm32.c
 *
 *  Created on: Jul 15, 2024
 *      Author: hp
 */
#include "stm32u5xx.h"
#include "userbutton_stm32.h"
#include "helper_stm32.h"
//#include "usart_stm32_console.h"
#include "oled_stm32_ssd1306.h"


void USERBUTTON_STM32_startTIM3(void) {
	 HAL_TIM_Base_Start(&htim3); // Démarrer le timer TIM3
}


void USERBUTTON_STM32_stopTIM3(void) {
	 HAL_TIM_Base_Stop(&htim3);  // Arrêter le timer TIM3
}

// Function to start the debounce timer
void StartDebounceTimer(void) {
	// Activate the timer
    HAL_TIM_Base_Start_IT(&htim7);
}

// Function to stop the debounce timer
void StopDebounceTimer(void) {
	// Disable timer
    HAL_TIM_Base_Stop_IT(&htim7);
    debounce_active = 0;
}


// Déclarer le handler d'interruption EXTI
void EXTI2_3_IRQHandler(void) {
	if (!debounce_active) {
		// Start debounce
	        debounce_active = 1;
	        StartDebounceTimer();
	    }
	else{
        __NOP();
    }
}

void TIM7_IRQHandler(void)
{
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
      DIODE_STM32_SET_LED_blue_LOW();
	  }
	  else {
	  // ENTER was pressed
	  USERBUTTON_STM32_startTIM3();
	  DIODE_STM32_SET_LED_GREEN_High();
	  OLED_STM32_updateMain_BienvenueView();
	  CONTROLPILOT_STM32_timerHighStart();

           }
}
void TIM3_IRQHandler(void) {

	OLED_STM32_updateMainView();
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

}


