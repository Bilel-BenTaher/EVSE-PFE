/*
 * RTC_stm32.h
 *
 *  Created on: Jul 22, 2024
 *      Author: hp
 */

#ifndef INC_RTC_STM32_H_
#define INC_RTC_STM32_H_

// Variable Definitions
extern char Time[10];
extern char date[10];

// Function Declarations
void get_time(void);
void get_date(void);

#endif /* INC_RTC_STM32_H_ */
