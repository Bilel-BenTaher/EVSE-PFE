/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_HandleA1 */
osThreadId_t Task_HandleA1Handle;
const osThreadAttr_t Task_HandleA1_attributes = {
  .name = "Task_HandleA1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleA2 */
osThreadId_t Task_HandleA2Handle;
const osThreadAttr_t Task_HandleA2_attributes = {
  .name = "Task_HandleA2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleB1 */
osThreadId_t Task_HandleB1Handle;
const osThreadAttr_t Task_HandleB1_attributes = {
  .name = "Task_HandleB1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleB2 */
osThreadId_t Task_HandleB2Handle;
const osThreadAttr_t Task_HandleB2_attributes = {
  .name = "Task_HandleB2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleC1 */
osThreadId_t Task_HandleC1Handle;
const osThreadAttr_t Task_HandleC1_attributes = {
  .name = "Task_HandleC1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleC2 */
osThreadId_t Task_HandleC2Handle;
const osThreadAttr_t Task_HandleC2_attributes = {
  .name = "Task_HandleC2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_HandleE */
osThreadId_t Task_HandleEHandle;
const osThreadAttr_t Task_HandleE_attributes = {
  .name = "Task_HandleE",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for xSemaphoreStateB2 */
osSemaphoreId_t xSemaphoreStateB2Handle;
const osSemaphoreAttr_t xSemaphoreStateB2_attributes = {
  .name = "xSemaphoreStateB2 "
};
/* Definitions for xSemaphoreStateC2 */
osSemaphoreId_t xSemaphoreStateC2Handle;
const osSemaphoreAttr_t xSemaphoreStateC2_attributes = {
  .name = "xSemaphoreStateC2 "
};
/* Definitions for xSemaphoreStateC1 */
osSemaphoreId_t xSemaphoreStateC1Handle;
const osSemaphoreAttr_t xSemaphoreStateC1_attributes = {
  .name = "xSemaphoreStateC1 "
};
/* Definitions for xSemaphoreStateE */
osSemaphoreId_t xSemaphoreStateEHandle;
const osSemaphoreAttr_t xSemaphoreStateE_attributes = {
  .name = "xSemaphoreStateE "
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CONTROLPILOT_STM32_HandleA1(void *argument);
void CONTROLPILOT_STM32_HandleA2(void *argument);
void CONTROLPILOT_STM32_HandleB1(void *argument);
void CONTROLPILOT_STM32_HandleB2(void *argument);
void CONTROLPILOT_STM32_HandleC1(void *argument);
void CONTROLPILOT_STM32_HandleC2 (void *argument);
void CONTROLPILOT_STM32_HandleE(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* creation of xSemaphoreStateB2 */
  xSemaphoreStateB2Handle = osSemaphoreNew(1, 1, &xSemaphoreStateB2_attributes);

  /* creation of xSemaphoreStateC2 */
  xSemaphoreStateC2Handle = osSemaphoreNew(1, 1, &xSemaphoreStateC2_attributes);

  /* creation of xSemaphoreStateC1 */
  xSemaphoreStateC1Handle = osSemaphoreNew(1, 1, &xSemaphoreStateC1_attributes);

  /* creation of xSemaphoreStateE */
  xSemaphoreStateEHandle = osSemaphoreNew(1, 1, &xSemaphoreStateE_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of Task_HandleA1 */
  Task_HandleA1Handle = osThreadNew(CONTROLPILOT_STM32_HandleA1, NULL, &Task_HandleA1_attributes);

  /* creation of Task_HandleA2 */
  Task_HandleA2Handle = osThreadNew(CONTROLPILOT_STM32_HandleA2, NULL, &Task_HandleA2_attributes);

  /* creation of Task_HandleB1 */
  Task_HandleB1Handle = osThreadNew(CONTROLPILOT_STM32_HandleB1, NULL, &Task_HandleB1_attributes);

  /* creation of Task_HandleB2 */
  Task_HandleB2Handle = osThreadNew(CONTROLPILOT_STM32_HandleB2, NULL, &Task_HandleB2_attributes);

  /* creation of Task_HandleC1 */
  Task_HandleC1Handle = osThreadNew(CONTROLPILOT_STM32_HandleC1, NULL, &Task_HandleC1_attributes);

  /* creation of Task_HandleC2 */
  Task_HandleC2Handle = osThreadNew(CONTROLPILOT_STM32_HandleC2 , NULL, &Task_HandleC2_attributes);

  /* creation of Task_HandleE */
  Task_HandleEHandle = osThreadNew(CONTROLPILOT_STM32_HandleE, NULL, &Task_HandleE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleA1 */
/**
* @brief Function implementing the Task_HandleA1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleA1 */
void CONTROLPILOT_STM32_HandleA1(void *argument)
{
  /* USER CODE BEGIN Task_HandleA1 */
  /* Infinite loop */
	for(;;)
	  {
		  CheckStateA1();
		  // Check voltage to trigger state E if necessary
		  if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
		  {
			  xTaskNotifyGive(Task_HandleEHandle); // Using a notification to report status E
		  }
		  if (state_A1)
		  {
		      // Send a notification to HandleA1 task
		      xTaskNotifyGive(Task_HandleB1Handle);
		  }
	  }
	  /* USER CODE END Task_HandleA2 */
  /* USER CODE END Task_HandleA1 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleA2 */
/**
* @brief Function implementing the Task_HandleA2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleA2 */
void CONTROLPILOT_STM32_HandleA2(void *argument)
{
  /* USER CODE BEGIN Task_HandleA2 */
  /* Infinite loop */
  for(;;)
  {
   ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //  Wait for a task notification
   CONTROLPILOT_STM32_startADCConversion();   // Start ADC conversion to measure voltage
   // Check voltage to trigger state E if necessary
  if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
  {
	  xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
  }
  if (state_A2)
  {
	  if (HELPER_STM32_getCurrentCPVoltage() == 12)
	  {
	    if (HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1) != HAL_OK)
	    {
	    	HandleError("Failed to stop PWM in Task_HandleA2");  // Handle error if PWM does not stop properly
	    }
	        // Send a notification to HandleA1 task
	        xTaskNotifyGive(Task_HandleA1Handle);
	  }
	  if (HELPER_STM32_getCurrentCPVoltage() == 9)
	  {
		  state_B1 = true;
		  // Send a notification to HandleB2 task
		  xTaskNotifyGive(Task_HandleA1Handle);
	  }
   }
  }
  /* USER CODE END Task_HandleA2 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleB1 */
/**
* @brief Function implementing the Task_HandleB1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleB1 */
void CONTROLPILOT_STM32_HandleB1(void *argument)
{
  /* USER CODE BEGIN Task_HandleB1 */
  /* Infinite loop */
  for(;;)
  {
	  CheckStateB1();
	  // Check voltage to trigger state E if necessary
	  if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	  {
		  xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	  }
	  if (state_B1)
	  {
	       xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	  }
	  if (HELPER_STM32_getCurrentCPVoltage() == 12)
	  {
	     // Send a notification to HandleA1 task
	      xTaskNotifyGive(Task_HandleA1Handle);  // Notification to HandleA1
	  }
  }
  /* USER CODE END Task_HandleB1 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleB2 */
/**
* @brief Function implementing the Task_HandleB2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleB2 */
void CONTROLPILOT_STM32_HandleB2(void *argument)
{
  /* USER CODE BEGIN Task_HandleB2 */
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //  Wait for a task notification
	    CheckStateB2();
	    // Check voltage to trigger state E if necessary
	 if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	 {
		 xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	 }
	 if (state_B2 && HELPER_STM32_getCurrentCPVoltage() == 9)
	 {
	    float maxCurrent = HELPER_STM32_getCurrentAmpere();
	    float dutyCycle = maxCurrent / 0.6;
	    SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	    vTaskDelay(pdMS_TO_TICKS(3000));
	    if (dutyCycle < 0.8 || dutyCycle > 0.97)
	    {
	        if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)
	        {
	        	 HandleError("Failed to stop PWM in Task_HandleB2");
	        }
	        // Send a notification to HandleA1 task
	       	   xTaskNotifyGive(Task_HandleB1Handle); // Notification to HandleB1
	           state_A1 = true;
	    }
	    else
	       {
	    	   state_C2 = true;
	    	   xTaskNotifyGive(Task_HandleC2Handle); // Notification to HandleC2
	       }
	 }
    if (HELPER_STM32_getCurrentCPVoltage() == 12)
     {
	    state_A2 = true;
	    // Send a notification to HandleA1 task
	    xTaskNotifyGive(Task_HandleA2Handle); // Notification to HandleA2

	  }

  }
  /* USER CODE END Task_HandleB2 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleC1 */
/**
* @brief Function implementing the Task_HandleC1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleC1 */
void CONTROLPILOT_STM32_HandleC1(void *argument)
{
  /* USER CODE BEGIN Task_HandleC1 */
  /* Infinite loop */
 for(;;)
 {
	 ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //  Wait for a task notification
	  CONTROLPILOT_STM32_startADCConversion();   // Start ADC conversion to measure voltag
	   // Check voltage to trigger state E if necessary
	   if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	   {
		   xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	   }
	   if (state_C1 && HELPER_STM32_getCurrentCPVoltage() = 6)
	   {
	       float maxCurrent = HELPER_STM32_getCurrentAmpere();
	       float dutyCycle = maxCurrent / 0.6;
	       SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	       if (dutyCycle > 0.8 || dutyCycle < 0.97)
	       {
	       vTaskDelay(pdMS_TO_TICKS(3000));
	       state_C2 = true;
	       xTaskNotifyGive(Task_HandleC2Handle);  // Notification to HandleC2
	       }
	       else
	       {
	    	   xTaskNotifyGive(Task_HandleC1Handle);  // Notification to HandleC1
	       }
	 }
	 if (HELPER_STM32_getCurrentCPVoltage() == 9)
	 {
		 vTaskDelay(pdMS_TO_TICKS(100));
		 if (!CONTROLPILOT_STM32_contactorOff())
		 {
		     HandleError("Failed to turn off contactor in Task_HandleC1");
		 }
	     state_A1 = true;
	     // Send a notification to HandleA1 task
	     xTaskNotifyGive(Task_HandleB1Handle); // Notification to HandleB1

	 }
 }
  /* USER CODE END Task_HandleC1 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleC2 */
/**
* @brief Function implementing the Task_HandleC2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleC2 */
void CONTROLPILOT_STM32_HandleC2 (void *argument)
{
  /* USER CODE BEGIN Task_HandleC2 */
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //  Wait for a task notification
		CONTROLPILOT_STM32_startADCConversion();   // Start ADC conversion to measure voltage
	    // Check voltage to trigger state E if necessary
	    if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	    {
	    	xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	    }
	    if (state_C2 && HELPER_STM32_getCurrentCPVoltage() = 6)
	    {
	    	if (!CONTROLPILOT_STM32_contactorOn())
	    	{
	    	  HandleError("Failed to turn on contactor in Task_HandleC2");
	        }
	       float maxCurrent = HELPER_STM32_getCurrentAmpere();
	       dutyCycle = maxCurrent / 0.6;
	       SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	       vTaskDelay(pdMS_TO_TICKS(5000));
	    }
	    if (CONTROLPILOT_STM32_getPilotVoltage() == 9.0)
	    {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!CONTROLPILOT_STM32_contactorOff())
	    	  {
	    	     HandleError("Failed to turn off contactor in Task_HandleC2");
	    	  }
	          state_B1 = true;
	          xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	    }
	    if (HELPER_STM32_getCurrentCPVoltage() == 12)
	    {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!CONTROLPILOT_STM32_contactorOff())
	          {
	    	     HandleError("Failed to turn off contactor in Task_HandleC2");
	    	  }
	          state_A2 = true;
	          // Send a notification to HandleA1 task
	          xTaskNotifyGive(Task_HandleA2Handle); // Notification to HandleA2
	     }
	     if (dutyCycle < 0.8 || dutyCycle > 0.97)
	     {
	        if (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK)
	          {
	        	 HandleError("Failed to stop PWM in Task_HandleC2");
	          }
	          vTaskDelay(pdMS_TO_TICKS(6000));
	          if (!CONTROLPILOT_STM32_contactorOff())
	          {
	            HandleError("Failed to turn off contactor in Task_HandleC2");
	          }
	          state_C1 = true;
	          xTaskNotifyGive(Task_HandleC1Handle); // Notification to HandleC1
	     }
  }
  /* USER CODE END Task_HandleC2 */
}

/* USER CODE BEGIN Header_CONTROLPILOT_STM32_HandleE */
/**
* @brief Function implementing the Task_HandleE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLPILOT_STM32_HandleE */
void CONTROLPILOT_STM32_HandleE(void *argument)
{
  /* USER CODE BEGIN Task_HandleE */
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a task notification
	  // Wait for a maximum delay of 3 seconds to complete the operation
	  vTaskDelay(pdMS_TO_TICKS(3000));
	  // Open the contactor (critical process)
      if (!CONTROLPILOT_STM32_contactorOff())
      {
          HandleError("Failed to turn off contactor in Task_HandleE");
      }
      state_A1 = false;
      state_A2 = false;
      state_B1 = false;
      state_B2 = false;
      state_C1 = false;
      state_C2 = false;
   }
  /* USER CODE END Task_HandleE */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

