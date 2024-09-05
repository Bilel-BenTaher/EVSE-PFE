/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.h
  * Description        : FreeRTOS applicative header file
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
#ifndef __APP_FREERTOS_H__
#define __APP_FREERTOS_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Exported macro -------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */
extern osThreadId_t Task_HandleA1Handle;
extern osThreadId_t Task_HandleA2Handle;
extern osThreadId_t Task_HandleB1Handle;
extern osThreadId_t Task_HandleB2Handle;
extern osThreadId_t Task_HandleC1Handle;
extern osThreadId_t Task_HandleC2Handle;
extern osThreadId_t Task_HandleEHandle;
extern osThreadId_t Task_ButtonHandle;
extern osThreadId_t Task_OledHandle;
extern osThreadId_t Task_RTCHandle;

/* Exported function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CONTROLPILOT_STM32_HandleA1(void *argument);
void CONTROLPILOT_STM32_HandleA2(void *argument);
void CONTROLPILOT_STM32_HandleB1(void *argument);
void CONTROLPILOT_STM32_HandleB2(void *argument);
void CONTROLPILOT_STM32_HandleC1(void *argument);
void CONTROLPILOT_STM32_HandleC2(void *argument);
void CONTROLPILOT_STM32_HandleE(void *argument);
void CONTROLEVSE_STM32_ButtonTask(void *argument);
void CONTROLDISPLAY_STM32_OledTask(void *argument);
void CONTROLDISPLAY_STM32_RTCTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

#ifdef __cplusplus
}
#endif
#endif /* __APP_FREERTOS_H__ */
