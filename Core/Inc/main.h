/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USERBUTTON_GPIO_MA_PIN_Pin GPIO_PIN_1
#define USERBUTTON_GPIO_MA_PIN_GPIO_Port GPIOC
#define USERBUTTON_GPIO_MA_PIN_EXTI_IRQn EXTI1_IRQn
#define OLED_DC_PIN_Pin GPIO_PIN_0
#define OLED_DC_PIN_GPIO_Port GPIOA
#define OLED_SCK_PIN_Pin GPIO_PIN_1
#define OLED_SCK_PIN_GPIO_Port GPIOA
#define OLED_RST_PIN_Pin GPIO_PIN_3
#define OLED_RST_PIN_GPIO_Port GPIOA
#define CONTROLPILOT_STM32_ADC_CHANNE_CURRENT_Pin GPIO_PIN_5
#define CONTROLPILOT_STM32_ADC_CHANNE_CURRENT_GPIO_Port GPIOA
#define CONTROLPILOT_STM32_ADC_CHANNEL_EVSE_Pin GPIO_PIN_6
#define CONTROLPILOT_STM32_ADC_CHANNEL_EVSE_GPIO_Port GPIOA
#define OLED_MOSI_PIN_Pin GPIO_PIN_7
#define OLED_MOSI_PIN_GPIO_Port GPIOA
#define OLED_CS_PIN_Pin GPIO_PIN_1
#define OLED_CS_PIN_GPIO_Port GPIOB
#define DIODE_LED_GPIO_RED_PIN_Pin GPIO_PIN_11
#define DIODE_LED_GPIO_RED_PIN_GPIO_Port GPIOD
#define DIODE_LED_GPIO_GREEN_PIN_Pin GPIO_PIN_12
#define DIODE_LED_GPIO_GREEN_PIN_GPIO_Port GPIOD
#define DIODE_LED_GPIO_YELLOW_PIN_Pin GPIO_PIN_13
#define DIODE_LED_GPIO_YELLOW_PIN_GPIO_Port GPIOD
#define USART_STM32_GPIO_TX_Pin GPIO_PIN_9
#define USART_STM32_GPIO_TX_GPIO_Port GPIOA
#define USART_STM32_GPIO_RX_Pin GPIO_PIN_10
#define USART_STM32_GPIO_RX_GPIO_Port GPIOA
#define CONTROLPILOT_STM32_GPIO_OUT_PIN_Pin GPIO_PIN_0
#define CONTROLPILOT_STM32_GPIO_OUT_PIN_GPIO_Port GPIOE
#define CONTROLPILOT_STM32_GPIO_CTCTR_PIN_Pin GPIO_PIN_1
#define CONTROLPILOT_STM32_GPIO_CTCTR_PIN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
