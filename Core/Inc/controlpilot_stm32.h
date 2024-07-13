/*
 * controlpilot_stm32.h
 *
 *  Created on: Jun 27, 2024
 *      Author: hp
 */

#ifndef INC_CONTROLPILOT_STM32_H_
#define INC_CONTROLPILOT_STM32_H_

#endif /* INC_CONTROLPILOT_STM32_H_ */

// CONTROLPILOT_STM32 library: This library shall enable the J1772 Control Pilot Signal on an STM32F0 chip.

// Type Definitions
#ifndef EVSEMODE_H
#define EVSEMODE_H
typedef enum {          LOW = 0,             HIGH = 1 }                                                              CONTROLPILOT_STM32_EVSE_SIDE;
typedef enum {     INACTIVE = 0,           ACTIVE = 1 }                                                              CONTROLPILOT_STM32_STATE;
#endif /* EVSEMODE_H */


#define    CONTROLPILOT_STM32_ADC_PWM_CORRECTOR  60
#define    VREFINT_CAL_ADDRPTR                   ((uint16_t*) ((uint32_t) 0x0BFA07A5))
#define    CONTROLPILOT_STM32_ADC_DELAY          3
#define    CONTROLPILOT_STM32_MODE_DELAY         21

// Variable Definitions
volatile uint16_t                                ADC_raw[4];
volatile uint8_t                                 adcDelayCounterHigh;
volatile uint8_t                                 adcDelayCounterLow;
volatile uint8_t                                 newMaximumTemp;
volatile CONTROLPILOT_STM32_EVSE_MODE            CONTROLPILOT_STM32_EVSE_ACTIVE_MODE;
volatile CONTROLPILOT_STM32_EVSE_MODE            CONTROLPILOT_STM32_EVSE_REQUESTED_MODE;
volatile CONTROLPILOT_STM32_STATE                CONTROLPILOT_STM32_EVSE_ACTIVE_PWM_STATE;
volatile CONTROLPILOT_STM32_EVSE_MODE            myStatus;
volatile uint16_t                                CONTROLPILOT_STM32_CP_VOLTAGE_LOW;
volatile uint16_t                                CONTROLPILOT_STM32_CP_VOLTAGE_HIGH;
volatile uint8_t                                 CONTROLPILOT_STM32_EVSE_MODE_SWITCH_COUNTER;

// Function Definitions

#define CONTROLPILOT_STM32_setHigh()    HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_OUT_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_OUT_PIN_Pin, GPIO_PIN_SET)
#define CONTROLPILOT_STM32_setLow()     HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_OUT_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_OUT_PIN_Pin, GPIO_PIN_RESET)
#define CONTROLPILOT_STM32_contactorOn()   HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_CTCTR_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_CTCTR_PIN_Pin, GPIO_PIN_SET)
#define CONTROLPILOT_STM32_contactorOff()  HAL_GPIO_WritePin(CONTROLPILOT_STM32_GPIO_CTCTR_PIN_GPIO_Port, CONTROLPILOT_STM32_GPIO_CTCTR_PIN_Pin, GPIO_PIN_RESET)


// Function Declarations
void CONTROLPILOT_STM32_configure(void);
void CONTROLPILOT_STM32_timerHighStart(void);
void CONTROLPILOT_STM32_timerHighStop(void);
void CONTROLPILOT_STM32_timerLowStart(void);
void CONTROLPILOT_STM32_timerLowStop(void);
void CONTROLPILOT_STM32_startADCConversion(CONTROLPILOT_STM32_EVSE_SIDE activeSide);
void CONTROLPILOT_STM32_setChargingCurrent(uint8_t ampereValue);

