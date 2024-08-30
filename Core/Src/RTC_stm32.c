/*
 * @file RTC_stm32.c
 * @brief Source file for STM32 RTC (Real-Time Clock) interface functions.
 *
 * This file contains the implementation of functions used to retrieve and format the current time and date
 * from the RTC on an STM32 microcontroller.
 *
 * @date July 22, 2024
 * @author Bilel BENTAHER
 */

#include "stm32u5xx.h"
#include "RTC_stm32.h"
#include <stdio.h>
#include <string.h>
extern RTC_HandleTypeDef hrtc;
/**
 * @brief Retrieves the current time from the RTC and formats it as a string.
 *
 * This function gets the current time from the RTC, formats it in HH:MM, and stores it in the global variable `Time`.
 *
 * @note The RTC time is retrieved in binary format (RTC_FORMAT_BIN) and converted to a string format.
 */
// Definition of global variables
char Time[10] = {0};  // Definition and initialization of Time
char Date[10] = {0};  // Definition and initialization of Date

void get_time(void)
{
    RTC_TimeTypeDef gTime;

    // Retrieve the current time from the RTC
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

    // Format the time as "HH:MM" and store it in the Time variable
    snprintf(Time, sizeof(Time), "%02d:%02d", gTime.Hours, gTime.Minutes);
}

/**
 * @brief Retrieves the current date from the RTC and formats it as a string.
 *
 * This function gets the current date from the RTC, formats it in DD.MM.YY, and stores it in the global variable `Date`.
 *
 * @note The RTC date is retrieved in binary format (RTC_FORMAT_BIN) and converted to a string format.
 */
void get_date(void)
{
    RTC_DateTypeDef gDate;

    // Retrieve the current date from the RTC
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

    // Format the date as "DD.MM.YY" and store it in the Date variable
    snprintf(Date, sizeof(Date), "%02d.%02d.%02d", gDate.Date, gDate.Month, gDate.Year);
}
