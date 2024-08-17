/* helper_stm32.h
 *  Created on: Jun 25, 2024
 *      Author: hp
 */

// HELPER_STM32 library: Provides helper functions and definitions for STM32U5XX chip.

#ifndef HELPER_STM32_H_
#define HELPER_STM32_H_

// Constants
#define TS_CAL1_ADDRPTR              ((uint16_t*) 0x0BFA0710)
#define HELPER_STM32_MOVINGAVERAGE   32
#define ADC_SAMPLES                  128

// Variable Declarations
extern float voltage_samples[ADC_SAMPLES];
extern volatile CONTROLPILOT_STM32_EVSE_MODE currentStatus;
extern volatile CONTROLPILOT_STM32_EVSE_MODE lastStatus;
extern volatile uint8_t currentAmpere;
extern volatile uint8_t currentPower;
extern volatile uint8_t currentVoltage;
extern volatile uint16_t VsenseCurrent;
extern volatile uint16_t previousTempArray[HELPER_STM32_MOVINGAVERAGE];
extern volatile uint8_t needsUpdate;


// Function Declarations
void HELPER_STM32_initSystemVariables(void);
CONTROLPILOT_STM32_EVSE_MODE HELPER_STM32_getCurrentStatus(void);
void HELPER_STM32_setCurrentStatus(CONTROLPILOT_STM32_EVSE_MODE newCurrentStatus);
float HELPER_STM32_getCurrentCPVoltage(void);
void HELPER_STM32_setCurrentCPVoltage(float newCurrentCPVoltage);
uint8_t HELPER_STM32_getCurrentAmpere(void);
void HELPER_STM32_setCurrentAmpere(uint8_t newCurrentAmpere);
float HELPER_STM32_getCurrentVoltage(void);
uint8_t HELPER_STM32_getCurrentPower(void);
void HELPER_STM32_setCurrentPower(uint8_t newCurrentPower);
void HELPER_STM32_setCurrentTemp(uint16_t VsenseCurrent);
int8_t HELPER_STM32_getCurrentTemp(void);
void HELPER_STM32_setNeedsUpdate(uint8_t newNeedsUpdate);
void HELPER_STM32_updateLoop(void);
void HELPER_STM32_getSetting(void);

#endif /* HELPER_STM32_H_ */

