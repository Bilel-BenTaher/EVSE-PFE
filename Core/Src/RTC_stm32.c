/*
 * RTC_stm32.c
 *
 *  Created on: Jul 22, 2024
 *      Author: hp
 */

#include "stm32u5xx.h"
#include "RTC_stm32.h"
#include <stdio.h>
#include <string.h>


void get_time (void)
{
	 RTC_TimeTypeDef gTime;
	/* Get the RTC current Time */
	 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Display time Format: hh:mm:ss */
	 sprintf((char*)Time,"%02d:%02d",gTime.Hours, gTime.Minutes);
}
void get_date(void)
{
	RTC_DateTypeDef gDate;
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	/* Display date Format: dd-mm-yy */
	sprintf((char*)date,"%02d.%02d.%2d",gDate.Date, gDate.Month,gDate.Year);
}

