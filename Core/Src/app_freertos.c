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
#include "oled_stm32_ssd1306.h"
#include "helper_stm32.h"
#include "RTC_stm32.h"
#include "Diode_led.h"
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
bool notificationSentToOled = false; // Variable to track if the message is already displayed
bool enChargeDisplayed = false; // Track if "En Charge" message has been displayed
bool DisplayedState = false;
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
/* Definitions for Task_Button */
osThreadId_t Task_ButtonHandle;
const osThreadAttr_t Task_Button_attributes = {
  .name = "Task_Button",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};
/* Definitions for Task_Oled */
osThreadId_t Task_OledHandle;
const osThreadAttr_t Task_Oled_attributes = {
  .name = "Task_Oled",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task_RTC */
osThreadId_t Task_RTCHandle;
const osThreadAttr_t Task_RTC_attributes = {
  .name = "Task_RTC",
  .priority = (osPriority_t) osPriorityNormal,
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
void CONTROLDISPLAY_STM32_OledTask(void *argument);
void CONTROLDISPLAY_STM32_RTCTask(void *argument);

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

  /* creation of Task_Oled */
  Task_OledHandle = osThreadNew(CONTROLDISPLAY_STM32_OledTask, NULL, &Task_Oled_attributes);

  /* creation of Task_RTC */
  Task_RTCHandle = osThreadNew(CONTROLDISPLAY_STM32_RTCTask, NULL, &Task_RTC_attributes);

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
			 if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
			 {
			     xTaskNotifyGive(Task_HandleEHandle); // Using a notification to report status E
		     }
		     else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
		     {
		    	 if(!notificationSentToOled) // Only display if not already displayed
		    	 {
		    	   // Send a notification to OledHandle task
		    	   xTaskNotifyGive(Task_OledHandle);
		    	   notificationSentToOled = true; // Mark as displayed
		    	 }
		    	 // Start the ADC conversion process
		    	 HAL_ADC_Start(&hadc1);

		    	 // Suspend the SysTick timer to stop the FreeRTOS tick
		    	 HAL_SuspendTick();

		    	 // Configure the power settings for Stop 1 mode
		    	 HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		    	 // Resume the SysTick timer
		    	 HAL_ResumeTick();

		    	 // Start the ADC conversion process
		    	 HAL_ADC_Stop(&hadc1);
		    	// Send a notification to HandleB1 task
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
		  if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
          {
	          xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
          }
          else if (HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
          {
              // Send a notification to HandleB2 task
              xTaskNotifyGive(Task_HandleB2Handle);
          }
          else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
	      {
        	  SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);// The function adjusts the pulse width to 100%, resulting in a constant high output.
        	  notificationSentToOled = false;
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
		  if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
	      {
		      xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
	      {
	          xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
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
	       if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
	       {
		      xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
	       {
	          float maxCurrent = HELPER_STM32_getCurrentAmpere();
	          float dutyCycle = maxCurrent / 0.6;
	          SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	          // Send a notification to OledHandle task
	          xTaskNotifyGive(Task_OledHandle);
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
	        else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
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
		 if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
	     {
		     xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	     }
	     else if (HELPER_STM32_getCurrentCPVoltage() >= 5.0 && HELPER_STM32_getCurrentCPVoltage() <= 7.0)
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
	      else if (HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
	      {
	    	 vTaskDelay(pdMS_TO_TICKS(100));
		     if (!HIGHVOLTAGE_STM32_contactorOff())
		     {
		    	 Error_Handler();
		     }
	         // Send a notification to HandleB1 task
	         xTaskNotifyGive(Task_HandleB1Handle); // Notification to HandleB1
	      }
	      else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
	      {  vTaskDelay(pdMS_TO_TICKS(100));
    	     if (!HIGHVOLTAGE_STM32_contactorOff())
             {
    	    	 Error_Handler();
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
	float TempRelais=0;
  /* Infinite loop */
  for(;;)
  {
	    CheckStateC2();
	    if(state_C2)
	    {
	       // Check voltage to trigger state E if necessary
	       if (HELPER_STM32_getCurrentCPVoltage() >= -1.0 && HELPER_STM32_getCurrentCPVoltage() <= 1.0)
	       {
	    	  xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	       }
	       else if(TempRelais>85||HELPER_STM32_getCurrentTemp>85)
	       {
	    	   xTaskNotifyGive(Task_HandleEHandle); // Notification to HandleE
	       }
	       else if(HELPER_STM32_getCurrentCPVoltage() >= 2.0 && HELPER_STM32_getCurrentCPVoltage() <= 4.0)
	       {
	    	   vTaskDelay(pdMS_TO_TICKS(3000));
	    	   if (!HIGHVOLTAGE_STM32_contactorOff())
	    	   {
	    		   Error_Handler();
	    	   }
	    	   SetPWMDutyCycle(&htim16, TIM_CHANNEL_1,100);
	    	   xTaskNotifyGive(Task_HandleC1Handle); // Notification to HandleC1
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() >= 5.0 && HELPER_STM32_getCurrentCPVoltage() <= 7.0)
	       {
	    	  if (!HIGHVOLTAGE_STM32_contactorOn())
	    	  {
	    		  Error_Handler();
	          }
	          float maxCurrent = HELPER_STM32_getCurrentAmpere();
	          dutyCycle = maxCurrent / 0.6;
	          SetPWMDutyCycle(&htim16, TIM_CHANNEL_1, dutyCycle);
	          // Send a notification to OledHandle task
	          xTaskNotifyGive(Task_OledHandle);
	          TempRelais=DS1621_getTemperature(hi2c1);
	          vTaskDelay(pdMS_TO_TICKS(5000));
	          xTaskNotifyGive(Task_HandleC2Handle); // Notification to HandleC2
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
	       {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!HIGHVOLTAGE_STM32_contactorOff())
	    	  {
	    		  Error_Handler();
	    	  }
	    	  xTaskNotifyGive(Task_OledHandle);
	          xTaskNotifyGive(Task_HandleB2Handle); // Notification to HandleB2
	       }
	       else if (HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
	       {
	    	  vTaskDelay(pdMS_TO_TICKS(100));
	    	  if (!HIGHVOLTAGE_STM32_contactorOff())
	          {
	    		  Error_Handler();
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
	        	  Error_Handler();
	          }
	          xTaskNotifyGive(Task_HandleC1Handle); // Notification to HandleC1
	       }
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
    	  Error_Handler();
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
          DisplayedState=true;
          vTaskDelay(pdMS_TO_TICKS(10000));
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
        	DisplayedState=false;
        }
        // Update the lastButtonState to the current state
        lastButtonState = currentButtonState;
      }
     }
    }
  /* USER CODE END Task_Button */
}

/* USER CODE BEGIN Header_CONTROLDISPLAY_STM32_OledTask */
/**
* @brief Function implementing the Task_Oled thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLDISPLAY_STM32_OledTask */
void CONTROLDISPLAY_STM32_OledTask(void *argument)
{
  /* USER CODE BEGIN Task_Oled */

  // Infinite loop to continuously check and update OLED display
  for(;;)
  {
    // Wait indefinitely for a task notification
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
      // Check for state and voltage conditions
      if (state_A1 && HELPER_STM32_getCurrentCPVoltage() >= 11.0 && HELPER_STM32_getCurrentCPVoltage() <= 13.0)
      {
        // Display "Please connect your charger" if voltage is within specified range
        OLED_STM32_clearArea(38, 32, "Bienvenue"); // Clear the area before drawing new text
        OLED_STM32_drawMonospaceString(12, 32, "Veuillez connecter");
        OLED_STM32_drawMonospaceString(21, 33, "votre chargeur");
        OLED_STM32_updateDisplay();

        // Update LED indicators
        SET_DIODE_LED_GREEN_LOW();
        SET_DIODE_LED_RED_HIGH();
      }
      else if (state_B2 && HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
      {
        // Clear the "Please connect your charger" message if voltage falls within another range
        OLED_STM32_clearArea(12, 32, "Veuillez connecter");
        OLED_STM32_clearArea(21, 33, "votre chargeur");
        OLED_STM32_updateDisplay();

        // Update LED indicators
        SET_DIODE_LED_RED_LOW();
        SET_DIODE_LED_BLUE_HIGH();
      }
      else if (state_C2 && HELPER_STM32_getCurrentCPVoltage() >= 5.0 && HELPER_STM32_getCurrentCPVoltage() <= 7.0)
      {
        // Static variables to store previous sensor readings
        static int8_t previousTemp = -128;  // Initialize with an impossible value for temperature
        static uint8_t previousAmp = 255;   // Initialize with an impossible value for current
        static uint16_t previousVolt = 65535; // Initialize with an impossible value for voltage
        static uint16_t previousPower = 65535; // Initialize with an impossible value for power

        // Get current sensor readings
        int8_t currentTemp = HELPER_STM32_getCurrentTemp();
        uint8_t currentAmp = HELPER_STM32_getCurrentAmpere();
        uint16_t currentPower = currentAmp*220;
        //uint16_t currentVolt =VoltageSensor_GetRMSVoltage();

        // Update temperature display only if it has changed
        if (currentTemp != previousTemp)
        {
          previousTemp = currentTemp;

          char tempStr[6];
          // Convert temperature to string format
          snprintf(tempStr, sizeof(tempStr), "%+d°C", currentTemp); // Format with a plus sign and degree symbol

          //calculate width in pixel
          int len_maxTempStr=strlen(tempStr)*6 - 2;
          for(int Tpos1 = 0; Tpos1 < 6; Tpos1++)
          {
             if(tempStr[Tpos1]=="1")
             {
          	    len_maxTempStr -= 2;
          	 }
          }

          // Draw temperature on the OLED display, centered at the top
          OLED_STM32_drawMonospaceString(64 - ( len_maxTempStr/ 2), 0, tempStr);
        }

        // Update current display only if it has changed
        if (currentAmp != previousAmp)
        {
          previousAmp = currentAmp;
          // Convert current to string format
          char ampStr[4];
          snprintf(ampStr, sizeof(ampStr), "%dA", currentAmp); // Format with 'A' for amperes

          // Draw current on the OLED display, bottom left
          OLED_STM32_drawMonospaceString(0, 54, ampStr);
        }

        // Update voltage display only if it has changed
        if (currentVolt != previousVolt)
        {
          previousVolt = currentVolt;
          // Convert voltage to string format
          char voltStr[6];
          snprintf(voltStr, sizeof(voltStr), "%dV", currentVolt); // Format with 'V' for volts

          //calculate width in pixel
          int len_currentVoltgStr = strlen(voltStr) * 6;
          for(int Vpos1 = 0; Vpos1 < 5; Vpos1++)
          {
          	  if(voltStr[Vpos1]=="1")
          	  {
          	     len_currentVoltgStr -= 2;
          	  }
          }

          // Draw voltage on the OLED display, bottom right
          OLED_STM32_drawMonospaceString(128 - (strlen(voltStr) * 6), 54, voltStr);
        }

        // Update power display only if it has changed
        if (currentPower != previousPower)
        {
          previousPower = currentPower;
          // Convert power to string format
          char powerStr[7];
          snprintf(powerStr, sizeof(powerStr), "%dW", currentPower); // Format with 'W' for watts

          //calculate width in pixel
          int len_currentPowerStr = strlen(powerStr) * 6;
          for (int POWpos1 = 0; POWpos1 < 6; POWpos1++)
          {
          	  if (powerStr[POWpos1] == '1')
          	  {
          		 len_currentPowerStr -= 2;
          	  }
          }

          // Draw power on the OLED display, bottom center
          OLED_STM32_drawMonospaceString(64 - (strlen(powerStr) * 6 / 2), 54, powerStr);
        }

        // Display "Charging" message only once
        if (!enChargeDisplayed)
        {
          OLED_STM32_drawMonospaceString(38, 32, "En Charge");
          enChargeDisplayed = true; // Set flag to avoid re-drawing
        }
      }
      else if (state_C2 && HELPER_STM32_getCurrentCPVoltage() >= 8.0 && HELPER_STM32_getCurrentCPVoltage() <= 10.0)
      {
        // Reset "Charging" message and update display with date and time
        enChargeDisplayed = false;
        get_time(); // Fetch current time
        get_date(); // Fetch current date
        OLED_STM32_clearDisplay(); // Clear the entire display
        OLED_STM32_drawLine(0, 9, 127, 9); // Draw horizontal lines for layout
        OLED_STM32_drawLine(0, 53, 127, 53);
        OLED_STM32_drawMonospaceString(0, 0, Time); // Display current time
        OLED_STM32_drawMonospaceString(86, 0, date); // Display current date
        OLED_STM32_drawMonospaceString(21, 32, "Batterie_chargee"); // Display "Battery charged" message
        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay to keep the message displayed for a while
        OLED_STM32_updateDisplay(); // Update the OLED display

        // Update LED indicators
        SET_DIODE_LED_BLUE_LOW();
        SET_DIODE_LED_GREEN_HIGH();
      }
    }
  }
  /* USER CODE END Task_Oled */
}

/* USER CODE BEGIN Header_CONTROLDISPLAY_STM32_RTCTask */
/**
* @brief Function implementing the Task_RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CONTROLDISPLAY_STM32_RTCTask */
void CONTROLDISPLAY_STM32_RTCTask(void *argument)
{
  /* USER CODE BEGIN Task_RTC */

  // Main loop to continuously update display
  for(;;)
  {
    if (DisplayedState) // Only update if display is active
    {
      // Static variables to hold previous date and time values
      static uint8_t previousDay = 0, previousMonth = 0, previousYear = 0;
      static uint8_t previousHour = 0, previousMinute = 0;

      // Buffers to store the current date and time as strings
      char currentDate[11];  // Format: DD.MM.YY
      char currentTime[6];   // Format: HH:MM

      // --- Date Handling ---

      // Fetch the current date from the RTC
      get_date(); // Assumes this function populates the global 'date' variable
      sscanf(date, "%02d.%02d.%2d", &currentDay, &currentMonth, &currentYear);

      // Only update the display if the date has changed
      if (currentDay != previousDay || currentMonth != previousMonth || currentYear != previousYear)
      {
        // Update previous date values
        previousDay = currentDay;
        previousMonth = currentMonth;
        previousYear = currentYear;

        // Display the new date on the OLED screen at coordinates (86, 0)
        OLED_STM32_drawMonospaceString(86, 0, date);
      }

      // --- Time Handling ---

      // Fetch the current time from the RTC
      get_time(); // Assumes this function populates the global 'Time' variable
      sscanf(Time, "%02d:%02d", &currentHour, &currentMinute);

      // Only update the display if the minute has changed
      if (currentHour != previousHour || currentMinute != previousMinute)
      {
        // Update previous time values
        previousHour = currentHour;
        previousMinute = currentMinute;

        // Display the new time on the OLED screen at coordinates (0, 0)
        OLED_STM32_drawMonospaceString(0, 0, Time);
      }

      // Delay task execution for 60 seconds (60000 ms) to avoid unnecessary updates
      vTaskDelay(pdMS_TO_TICKS(60000));
    }
  }
  /* USER CODE END Task_RTC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

