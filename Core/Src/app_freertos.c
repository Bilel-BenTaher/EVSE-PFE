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
#include "helper_stm32.h"
#include "oled_stm32_ssd1306.h"
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
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_Button */
osThreadId_t Task_ButtonHandle;
const osThreadAttr_t Task_Button_attributes = {
  .name = "Task_Button",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
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
void CONTROLEVSE_STM32_ButtonTask(void *argument);

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

  /* creation of Task_Button */
  Task_ButtonHandle = osThreadNew(CONTROLEVSE_STM32_ButtonTask, NULL, &Task_Button_attributes);

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
		  if(state_A1)
		  {
		     // Check voltage to trigger state E if necessary
		     if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
		     {
			     xTaskNotifyGive(Task_HandleEHandle); // Using a notification to report status E
		     }
		     else if (HELPER_STM32_getCurrentCPVoltage()== 12.0)
		     {
		         // Send a notification to HandleA1 task
		         xTaskNotifyGive(Task_HandleB1Handle);
		     }
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
	  CheckStateA2();
	  if(state_A2)
	  {
          // Check voltage to trigger state E if necessary
          if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
          {
	          xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
          }
          else if (HELPER_STM32_getCurrentCPVoltage() == 9.0)
          {
              // Send a notification to HandleB2 task
              xTaskNotifyGive(Task_HandleB2Handle);
          }
          else if (HELPER_STM32_getCurrentCPVoltage() == 12.0)
	      {
        	  SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 100%, resulting in a constant high output.
	           // Send a notification to HandleA1 task
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
	  if (state_B1)
	  {
	      // Check voltage to trigger state E if necessary
	      if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	      {
		      xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() == 9.0)
	      {
	          xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() == 12.0)
	      {
	         // Send a notification to HandleA1 task
	         xTaskNotifyGive(Task_HandleA1Handle);  // Notification to HandleA1
	      }
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
	    CheckStateB2();
	    if (state_B2)
	    {
	       // Check voltage to trigger state E if necessary
	       if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	       {
		      xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() == 9.0)
	       {
	          float maxCurrent = HELPER_STM32_getCurrentAmpere();
	          float dutyCycle = maxCurrent / 0.6;
	          SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	          vTaskDelay(pdMS_TO_TICKS(3000));
	          if (dutyCycle < 0.8 || dutyCycle > 0.97)
	          {
	        	  SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 100%, resulting in a constant high output.
	              // Send a notification to HandleB1 task
	        	  xTaskNotifyGive(Task_HandleB1Handle); // Notification to HandleB1
	          }
	          else
	          {
	    	      xTaskNotifyGive(Task_HandleC2Handle); // Notification to HandleC2
	          }
	        }
	        else if (HELPER_STM32_getCurrentCPVoltage() == 12.0)
            {
	           // Send a notification to HandleA1 task
	           xTaskNotifyGive(Task_HandleA2Handle); // Notification to HandleA2
	        }
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
	 CheckStateC1();
	   if(state_C1)
	   {
	     // Check voltage to trigger state E if necessary
	     if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	     {
		     xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	     }
	     else if (HELPER_STM32_getCurrentCPVoltage() = 6.0)
	     {
	        float maxCurrent = HELPER_STM32_getCurrentAmpere();
	        float dutyCycle = maxCurrent / 0.6;
	        SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	        if (dutyCycle > 0.8 || dutyCycle < 0.97)
	        {
	           vTaskDelay(pdMS_TO_TICKS(3000));
	           xTaskNotifyGive(Task_HandleC2Handle);  // Notification to HandleC2
	        }
	        else
	        {  SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 100%, resulting in a constant high output.
	    	   xTaskNotifyGive(Task_HandleC1Handle);  // Notification to HandleC1
	        }
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() == 9.0)
	      {
	    	 vTaskDelay(pdMS_TO_TICKS(100));
		     if (!HIGHVOLTAGE_STM32_contactorOff())
		     {
		        HandleError("Failed to turn off contactor in Task_HandleC1");
		     }
	         // Send a notification to HandleB1 task
	         xTaskNotifyGive(Task_HandleB1Handle); // Notification to HandleB1
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() == 12.0)
	      {  vTaskDelay(pdMS_TO_TICKS(100));
    	     if (!HIGHVOLTAGE_STM32_contactorOff())
             {
    	        HandleError("Failed to turn off contactor in Task_HandleC2");
    	     }
             // Send a notification to HandleA2 task
             xTaskNotifyGive(Task_HandleA1Handle); // Notification to HandleA1
	      }
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
	float dutyCycle;
  /* Infinite loop */
  for(;;)
  {
	    CheckStateC2();
	    if(state_C2)
	    {
	       // Check voltage to trigger state E if necessary
	       if (HELPER_STM32_getCurrentCPVoltage() == 0.0)
	       {
	    	  xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	       }
	       else if(HELPER_STM32_getCurrentCPVoltage() == 3.0)
	       {
	    	   vTaskDelay(pdMS_TO_TICKS(3000));
	    	   if (!HIGHVOLTAGE_STM32_contactorOff())
	    	   {
	    	   	   HandleError("Failed to turn off contactor in Task_HandleC2");
	    	   }
	    	   SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);
	    	   xTaskNotifyGive(Task_HandleC1Handle); // Notification to HandleC1
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() = 6.0)
	       {
	    	  if (!HIGHVOLTAGE_STM32_contactorOn())
	    	  {
	    	     HandleError("Failed to turn on contactor in Task_HandleC2");
	          }
	          float maxCurrent = HELPER_STM32_getCurrentAmpere();
	          dutyCycle = maxCurrent / 0.6;
	          SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	       }
	       else if (CONTROLPILOT_STM32_getPilotVoltage() == 9.0)
	       {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!HIGHVOLTAGE_STM32_contactorOff())
	    	  {
	    	     HandleError("Failed to turn off contactor in Task_HandleC2");
	    	  }
	          xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() == 12.0)
	       {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!HIGHVOLTAGE_STM32_contactorOff())
	          {
	    	     HandleError("Failed to turn off contactor in Task_HandleC2");
	    	  }
	          // Send a notification to HandleA2 task
	          xTaskNotifyGive(Task_HandleA2Handle); // Notification to HandleA2
	       }
	       else if (dutyCycle < 0.8 || dutyCycle > 0.97)
	       {
	    	  SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 100%, resulting in a constant high output.
	          vTaskDelay(pdMS_TO_TICKS(6000));
	          if (!HIGHVOLTAGE_STM32_contactorOff())
	          {
	            HandleError("Failed to turn off contactor in Task_HandleC2");
	          }
	          xTaskNotifyGive(Task_HandleC1Handle); // Notification to HandleC1
	       }
	       vTaskDelay(pdMS_TO_TICKS(5000));
	       xTaskNotifyGive(Task_HandleC2Handle); // Notification to HandleC2
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
      if (!HIGHVOLTAGE_STM32_contactorOff())
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

/* USER CODE BEGIN Header_CONTROLEVSE_STM32_ButtonTask */
/**
* @brief Function implementing the Task_Button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLEVSE_STM32_ButtonTask */
void CONTROLEVSE_STM32_ButtonTask(void *argument)
{
  /* USER CODE BEGIN Task_Button */
  GPIO_PinState lastButtonState = GPIO_PIN_RESET; // Initial state: button is not pressed

  /* Infinite loop */
  for (;;)
  {
    // Read the current state of the button
    GPIO_PinState currentButtonState = HAL_GPIO_ReadPin(USERBUTTON_GPIO_MA_PIN_GPIO_Port, USERBUTTON_GPIO_MA_PIN_Pin);

    if (currentButtonState != lastButtonState) // Detect a state change
    {
      vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms debounce delay
      // Read the button state again after the delay
      currentButtonState = HAL_GPIO_ReadPin(USERBUTTON_GPIO_MA_PIN_GPIO_Port, USERBUTTON_GPIO_MA_PIN_Pin);
      if (currentButtonState != lastButtonState) // Confirm the state change
      {
        if (currentButtonState == GPIO_PIN_SET) // If the button is now pressed
        {
          OLED_STM32_initDisplay();
          vTaskDelay(pdMS_TO_TICKS(50));
          OLED_STM32_updateMain_BienvenueView();
          SET_DIODE_LED_GREEN_HIGH();
          SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 0, resulting in a constant high output.
          // The button press is validated
          xTaskNotifyGive(Task_HandleA1Handle); // Start all state machine tasks
        }
        else if (currentButtonState == GPIO_PIN_RESET) // If the button is now released
        {
        	SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,0);// The function adjusts the pulse width to 0, resulting in a constant low output.
        	OLED_DISPLAYOFF ;
        	SET_DIODE_lED_RED_LOW();
        	SET_DIODE_LED_GREEN_LOW();
        	SET_DIODE_LED_BLUE_LOW();
        }
        // Update the lastButtonState to the current state
        lastButtonState = currentButtonState;
      }
     }
    }
  /* USER CODE END Task_Button */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

