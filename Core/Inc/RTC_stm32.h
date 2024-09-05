/*
 * @file RTC_stm32.h
 * @brief Header file for STM32 RTC (Real-Time Clock) interface.
 *
 * This file contains the declarations and variable definitions used for interacting with the RTC on an STM32u575vgt6 microcontroller.
 * It provides functions to retrieve the current time and date.
 *
 * @date July 22, 2024
 * @author Bilel BENTAHER
 */

#ifndef INC_RTC_STM32_H_
#define INC_RTC_STM32_H_

// External Variables
/**
 * @brief Stores the current time as a string in the format HH:MM.
 *
 * The time is retrieved from the RTC and stored in this variable for easy access and display.
 */
extern char Time[10];

/**
 * @brief Stores the current date as a string in the format DD/MM/YYYY.
 *
 * The date is retrieved from the RTC and stored in this variable for easy access and display.
 */
extern char Date[12];

// Function Prototypes
/**
 * @brief Retrieves the current time from the RTC and stores it in the Time variable.
 *
 * This function interacts with the STM32 RTC hardware to get the current time and formats it as a string.
 * The formatted time is then stored in the global variable `Time`.
 */
void get_time(void);

/**
 * @brief Retrieves the current date from the RTC and stores it in the Date variable.
 *
 * This function interacts with the STM32 RTC hardware to get the current date and formats it as a string.
 * The formatted date is then stored in the global variable `Date`.
 */
void get_date(void);

#endif /* INC_RTC_STM32_H_ */
