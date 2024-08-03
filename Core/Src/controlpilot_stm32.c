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

// Define ADC raw values array
#define ADC_RAW_SIZE 4
static uint16_t ADC_raw[ADC_RAW_SIZE];

// Calibration constants (placeholder values, adjust as needed)
#define VREFINT_CAL_ADDR ((uint16_t*)0x1FFF75A8) // Example address, adjust if needed
#define CONTROLPILOT_STM32_MODE_DELAY 5
#define CONTROLPILOT_STM32_ADC_DELAY 10

void CONTROLPILOT_STM32_configure(void) {
    // Start ADC calibration
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
    HAL_TIM_Base_Start_IT(&htim17);
}

void CONTROLPILOT_STM32_timerLowStop(void) {
    HAL_TIM_Base_Stop_IT(&htim17);
}

void CONTROLPILOT_STM32_startADCConversion(CONTROLPILOT_STM32_EVSE_SIDE activeSide) {
    // Start ADC conversion
    HAL_ADC_Start(&hadc4);
    // Wait for the end of sequence
    for (int i = 0; i < ADC_RAW_SIZE; i++) {
        // Poll for end of conversion
        HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
        // Get the ADC value
        ADC_raw[i] = HAL_ADC_GetValue(&hadc4);
    }
    // Stop ADC conversion
    HAL_ADC_Stop(&hadc4);

    // Vdd calculation
    double Vrefint = (double)ADC_raw[1];
    double Vrefint_cal_float = (double)(*VREFINT_CAL_ADDR);
    double Vddfloat = 3000.0 * Vrefint_cal_float / Vrefint;

    // EVSE_IN calculation
    double ADCVoltageFloat = (double)ADC_raw[0];
    double cpVoltageFloat = Vddfloat * ADCVoltageFloat / 4095.0;
    HELPER_STM32_setCurrentVoltage(cpVoltageFloat);

    if (activeSide == HIGH) {
        CONTROLPILOT_STM32_CP_VOLTAGE_HIGH = (uint16_t)cpVoltageFloat;
    } else {
        CONTROLPILOT_STM32_CP_VOLTAGE_LOW = (uint16_t)cpVoltageFloat;
    }

    // Power calculation
    double cpCurrentFloat = HELPER_STM32_getCurrentAmpere(); // Get current value
    double cpPowerFloat = cpCurrentFloat * cpVoltageFloat;
    HELPER_STM32_setCurrentPower(cpPowerFloat);

    // Temp calculation; reduced to Vsense to lower workload on this function
    double Temprefint = (double)ADC_raw[3];
    double VsenseCurrent = Vddfloat * Temprefint / 4095.0;
    HELPER_STM32_setCurrentTemp(VsenseCurrent);
    MaximumTemp_int = HELPER_STM32_getCurrentTemp();
}

void CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CONTROLPILOT_STM32_EVSE_MODE vehicleMode) {
    // FAULT needs to be acted upon IMMEDIATELY!
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
        if (CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER < CONTROLPILOT_STM32_MODE_DELAY) {
            return;
        }

        // After ensuring the EVSE_MODE request is safe, change the mode.
        CONTROLPILOT_STM32_EVSE_ACTIVE_MODE = vehicleMode;
        switch (CONTROLPILOT_STM32_EVSE_ACTIVE_MODE) {
            case DISCONNECTED:
                CONTROLPILOT_STM32_contactorOff();
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = INACTIVE;
                CONTROLPILOT_STM32_CP_VOLTAGE_LOW = 0; // Reset voltage to zero as it's not being measured actively
                break;
            case CONNECTED_NO_PWM:
                CONTROLPILOT_STM32_contactorOff();
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE;
                break;
            case CONNECTED:
                CONTROLPILOT_STM32_contactorOff();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) {
                    CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE;
                }
                break;
            case CHARGING:
                CONTROLPILOT_STM32_contactorOn();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) {
                    CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE;
                }
                break;
            case CHARGING_COOLED:
                CONTROLPILOT_STM32_contactorOff();
                if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE != ACTIVE) {
                    CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = ACTIVE;
                }
                break;
            case FAULT:
                CONTROLPILOT_STM32_contactorOff();
                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE = INACTIVE;
                break;
        }
        HELPER_STM32_setCurrentStatus(CONTROLPILOT_STM32_EVSE_ACTIVE_MODE);
    }
}

void CONTROLPILOT_STM32_setChargingCurrent(void) {
    uint8_t ampereValue = HELPER_STM32_getCurrentAmpere();
    double ampereValueFloat = (double)ampereValue;
    double calibratedDutyCycle = ampereValueFloat * 10.0 / 0.6;
    __HAL_TIM_SET_AUTORELOAD(&htim17, (uint32_t)calibratedDutyCycle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if (htim==&htim16) {
        MaximumTemp_ext = DS1621_getTemperature(&hi2c1);
        if (MaximumTemp_int < 85 && MaximumTemp_ext < 85) {
            CONTROLPILOT_STM32_setHigh();
            CONTROLPILOT_STM32_startADCConversion(HIGH);
            CONTROLPILOT_STM32_setChargingCurrent();
            if (CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE == ACTIVE) {
                CONTROLPILOT_STM32_timerLowStart();
            }

            // Checking for PWM_STATE_HIGH by checking voltage against table
            switch (CONTROLPILOT_STM32_CP_VOLTAGE_HIGH) {
                case 2952 ... 3200:
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(DISCONNECTED);
                    break;
                case 2582 ... 2830:
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(
                        CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE == INACTIVE ?
                        CONNECTED_NO_PWM : CONNECTED);
                    break;
                case 2212 ... 2459:
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CHARGING);
                    break;
                case 1841 ... 2089:
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(CHARGING_COOLED);
                    break;
                default:
                    CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(FAULT);
                    break;
            }
        } else {
            CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(FAULT);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if (htim==&htim17) {
        CONTROLPILOT_STM32_timerLowStop();
        CONTROLPILOT_STM32_setLow();
        if (adcDelayCounterLow > CONTROLPILOT_STM32_ADC_DELAY) {
            CONTROLPILOT_STM32_startADCConversion(LOW);
            adcDelayCounterLow = 0;
        } else {
            adcDelayCounterLow++;
        }
        // Set EVSE_STATE to FAULT if negative PWM voltage is reduced due to shorted or faulty safety diode.
        if (CONTROLPILOT_STM32_CP_VOLTAGE_LOW > 150 || newMaximumAmpere > 60) {
            CONTROLPILOT_STM32_SWITCH_VEHICLE_STATUS(FAULT);
        }
    }
}
