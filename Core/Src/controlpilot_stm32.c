/*
 * controlpilot_stm32.c
 *
 *  Created on: Jun 27, 2024
 *      Author: hp
 */

#include "stm32u5xx.h"
#include "controlpilot_stm32.h"
#include "helper_stm32.h"
//#include "usart_stm32_console.h"
#include "oled_stm32_ssd1306.h"
#include <stdlib.h>


void CONTROLPILOT_STM32_configure(void) {

    // Start calibration
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

    // Variable Initialization
    CONTROLPILOT_STM32_EVSE_ACTIVE_MODE = DISCONNECTED;
    CONTROLPILOT_STM32_EVSE_REQUESTED_MODE = DISCONNECTED;
    CONTROLPILOT_STM32_CP_VOLTAGE_LOW = 0;
    CONTROLPILOT_STM32_CP_VOLTAGE_HIGH = 0;
    CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = INACTIVE;
    CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER = 0;
    adcDelayCounterHigh = 0;
    adcDelayCounterLow = 0;

}

void CONTROLPILOT_STM32_timerHighStart(void) {
    HAL_TIM_Base_Start_IT(&htim16);
}


void CONTROLPILOT_STM32_timerHighStop(void) {
    HAL_TIM_Base_Stop_IT(&htim16);
}

void CONTROLPILOT_STM32_timerLowStart(void) {
    HAL_TIM_Base_Start_IT(&htim117);
}


void CONTROLPILOT_STM32_timerLowStop(void) {
    HAL_TIM_Base_Stop_IT(&htim17);
}

void CONTROLPILOT_STM32_startADCConversion(CONTROLPILOT_STM32_EVSE_SIDE activeSide) {
	// Start ADC conversion
	   HAL_ADC_Start(&hadc4);
	    // Wait for the end of sequence
	    for (int i = 0; i < 4; i++) {
	        // Poll for end of conversion
	        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);

	        // Get the ADC value
	        ADC_raw[i] = HAL_ADC_GetValue(&hadc4);
	    }
	    // Stop ADC conversion
	    HAL_ADC_Stop(&hadc4);

	    // Vdd calculation
	    double Vrefint  = (double)ADC_raw[3];
	    double Vrefint_cal_float = (double)(*VREFINT_CAL_ADDR);
	    double Vddfloat = 3000.0 * Vrefint_cal_float / Vrefint;


	    // EVSE_IN calculation
	    double ADCVoltageFloat = (double)ADC_raw[0];
	    double cpVoltageFloat = Vddfloat * ADCVoltageFloat / 4095.0;
	   	        if (activeSide == HIGH) {
	   	            CONTROLPILOT_STM32_CP_VOLTAGE_HIGH = (uint16_t)cpVoltageFloat;
	   	        } else {
	   	            CONTROLPILOT_STM32_CP_VOLTAGE_LOW = (uint16_t)cpVoltageFloat;
	   	        }


	    // input current
	   	double  ADCCurrentFloat = (double)ADC_raw[1];
	    double cpCurrentFloat = Vddfloat * ADCCurrentFloat / 4095.0;
	    HELPER_STM32_setCurrentAmpere(cpCurrentFloat);

	    // Temp calculation; reduced to Vsense to lower workload on this function
	    double Temprefint = (double)ADC_raw[2];
	    double VsenseCurrent = Vddfloat * Temprefint / 4095.0;
	    HELPER_STM32_setCurrentTemp(VsenseCurrent);
	    newMaximumTemp=HELPER_STM32_getCurrentTemp();
	}


void CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CONTROLPILOT_STM32_EVSE_MODE vehicleMode) {

    // FAULT needs to be acted upon IMMEDIATELY!

    // FAULT can only be cleared by DISCONNECTED, perhaps change to acitve-mode == fault
    if (CONTROLPILOT_STM32_EVSE_ACTIVE_MODE == FAULT) {
        if (vehicleMode != DISCONNECTED) {
        	 CONTROLPILOT_STM32_contactorOff();
            return;
        }
    }

    // A counter is used to ensure a solid EVSE_MODE change request is provided.
    if (vehicleMode != CONTROLPILOT_STM32_EVSE_ACTIVE_MODE) {
        if (CONTROLPILOT_STM32_EVSE_REQUESTED_MODE == vehicleMode) {
            CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER++;
        } else {
            CONTROLPILOT_STM32_EVSE_REQUESTED_MODE = vehicleMode;
            CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER = 0;
        }
        if (CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER < CONTROLPILOT_STM32_MODE_DELAY) { return; }

        // After ensuring the EVSE_MODE request is safe, change the mode.
        CONTROLPILOT_STM32_EVSE_ACTIVE_MODE = vehicleMode;
        switch (CONTROLPILOT_STM32_EVSE_ACTIVE_MODE) {
            case DISCONNECTED:
                CONTROLPILOT_STM32_contactorOff();
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = INACTIVE;
                CONTROLPILOT_STM32_CP_VOLTAGE_LOW = 0; // Reset voltage to zero as it's not being measured actively
                //USART_STM32_sendStringToUSART("New Vehicle Mode: DISCONNECTED");
                break;
            case CONNECTED_NO_PWM:
                CONTROLPILOT_STM32_contactorOff();
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE;
                //USART_STM32_sendStringToUSART("New Vehicle Mode: CONNECTED_NO_PWM");
                break;
            case CONNECTED:
                CONTROLPILOT_STM32_contactorOff();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) { CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE; }
                //USART_STM32_sendStringToUSART("New Vehicle Mode: CONNECTED");
                break;
            case CHARGING:
                CONTROLPILOT_STM32_contactorOn();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) { CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE; }
                //USART_STM32_sendStringToUSART("New Vehicle Mode: CHARGING");
                break;
            case CHARGING_COOLED:
                CONTROLPILOT_STM32_contactorOff();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) { CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE; }
                //USART_STM32_sendStringToUSART("New Vehicle Mode: CHARGING_COOLED");
                break;
            case FAULT:
                CONTROLPILOT_STM32_contactorOff();
                // Perhaps trigger third timer to retry after x-seconds if fault still exists...
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = INACTIVE;
                //USART_STM32_sendStringToUSART("New Vehicle Mode: FAULT");
                break;
        }
        HELPER_STM32_setCurrentStatus(CONTROLPILOT_STM32_EVSE_ACTIVE_MODE);
    }

}


void CONTROLPILOT_STM32_setChargingCurrent(void) {
	uint8_t ampereValue=HELPER_STM32_getCurrentAmpere();
    double ampereValueFloat = (double)ampereValue;
    double calibratedDutyCycle = ampereValueFloat * 10.0 / 0.6;
    TIM_SetAutoreload(CONTROLPILOT_STM32_TIMER_LOW, calibratedDutyCycle);
    //USART_STM32_sendIntegerToUSART("calibratedDutyCycle = ", calibratedDutyCycle);
}



// BUG found! when connecting with the fault condition, no fault is detected.
void TIM16_IRQHandler(void) {

    if (TIM_GetITStatus(CONTROLPILOT_STM32_TIMER_HIGH, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(CONTROLPILOT_STM32_TIMER_HIGH, TIM_IT_Update);
        if(newMaximumTemp<80){
        CONTROLPILOT_STM32_setHigh();
        CONTROLPILOT_STM32_startADCConversion(HIGH);
        CONTROLPILOT_STM32_setChargingCurrent();
        if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE == ACTIVE) { CONTROLPILOT_STM32_timerLowStart(); }
        // Subtract offset if PWM is disabled
       // if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE == INACTIVE) {
         //   CONTROLPILOT_STM32_CP_VOLTAGE_HIGH = CONTROLPILOT_STM32_CP_VOLTAGE_HIGH - CONTROLPILOT_STM32_ADC_PWM_CORRECTOR;
       // }
        // Checking for PWM_STATE_HIGH by checking voltage against table
        switch (CONTROLPILOT_STM32_CP_VOLTAGE_HIGH) {
            case 2952 ... 3200:
                CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(DISCONNECTED);
                needsUpdate == 1;
                break;
            case 2582 ... 2830:
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE == INACTIVE) {
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CONNECTED_NO_PWM);
                } else {
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CONNECTED);
                }
                break;
            case 2212 ... 2459:
                CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CHARGING);
                needsUpdate == 1;
                break;
            case 1841 ... 2089:
                CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CHARGING_COOLED);
                break;
            default:
                // No Change or throw error?
                break;
        }
    }
        else {CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(FAULT);}
        myStatus = currentStatus;
        if ((lastStatus != myStatus)| (needsUpdate == 1) )
        {
        OLED_STM32_updateMainView();
        lastStatus = myStatus;
        }
    }
}


void TIM17_IRQHandler(void) {

    if (TIM_GetITStatus(CONTROLPILOT_STM32_TIMER_LOW, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(CONTROLPILOT_STM32_TIMER_LOW, TIM_IT_Update);
        CONTROLPILOT_STM32_timerLowStop();
		CONTROLPILOT_STM32_setLow();
        if (adcDelayCounterLow > CONTROLPILOT_STM32_ADC_DELAY) {
            CONTROLPILOT_STM32_startADCConversion(LOW);
            adcDelayCounterLow = 0;
        } else {
            adcDelayCounterLow++;
        }
        // Set EVSE_STATE to FAULT if negative PWM voltage is reduced due to shorted or faulty safety diode.
        if (CONTROLPILOT_STM32_CP_VOLTAGE_LOW > 150 ||newMaximumAmpere>60 ) { CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(FAULT); }
        myStatus = currentStatus;
        if ((lastStatus != myStatus)| (needsUpdate == 1))
          {
            OLED_STM32_updateMainView();
            lastStatus = myStatus;
          }
	}

}
