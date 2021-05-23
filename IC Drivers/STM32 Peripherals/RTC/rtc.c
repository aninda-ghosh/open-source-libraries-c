/* Includes ------------------------------------------------------------------*/
#include "rtc.h"
#include <time.h>

RTC_HandleTypeDef hrtc;

#define EPOCH_TIME_OFF 946684800 // This is 1st January 2000, 00:00:00 in epoch time
#define EPOCH_TIME_YEAR_OFF 100	 // years since 1900


static uint8_t _hours;
static uint8_t _minutes;
static uint8_t _seconds;
static uint8_t _year;
static uint8_t _month;
static uint8_t _day;
static uint8_t _wday;

/* RTC initialization function */
void RTC_Init(void) {
	/** Initialize RTC Only	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
}

uint8_t RTC_IsTimeSet(void) {
	return HAL_RTCEx_BKUPRead(&hrtc, RTC_BACKUP_INDEX) == RTC_BACKUP_VALUE ? 1 : 0;
}

/* Epoch Functions */
uint32_t RTC_GetEpoch(void) {
	struct tm tm;
	RTC_GetTime(&_hours, &_minutes, &_seconds);
	RTC_GetDate(&_year, &_month, &_day, &_wday);
	tm.tm_isdst = -1;
	/*
	 * mktime ignores the values supplied by the caller in the
	 * tm_wday and tm_yday fields
	 */
	tm.tm_yday = 0;
	tm.tm_wday = 0;
	tm.tm_year = _year + EPOCH_TIME_YEAR_OFF;
	tm.tm_mon = _month - 1;
	tm.tm_mday = _day;
	tm.tm_hour = _hours;
	tm.tm_min = _minutes;
	tm.tm_sec = _seconds;
	return mktime(&tm);
}

void RTC_SetEpoch(uint32_t ts) {
	if (ts < EPOCH_TIME_OFF) {
		ts = EPOCH_TIME_OFF;
	}
	time_t t = ts;
	struct tm *tmp = gmtime(&t);
	_year = tmp->tm_year - EPOCH_TIME_YEAR_OFF;
	_month = tmp->tm_mon + 1;
	_day = tmp->tm_mday;
	if (tmp->tm_wday == 0) {
		_wday = RTC_WEEKDAY_SUNDAY;
	} else {
		_wday = tmp->tm_wday;
	}
	_hours = tmp->tm_hour;
	_minutes = tmp->tm_min;
	_seconds = tmp->tm_sec;
	RTC_SetDate(_year, _month, _day, _wday);
	RTC_SetTime(_hours, _minutes, _seconds);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BACKUP_INDEX, RTC_BACKUP_VALUE);
}

void RTC_Backup(void) {
	/* Store the date in the backup registers */
	uint32_t dateToStore;
	memcpy(&dateToStore, &hrtc.DateToUpdate, 4);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, dateToStore >> 16);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2 + 1, dateToStore & 0xffff);
}

void RTC_Restore(void) {
	// Copy date data back out of the BackUp registers
	RTC_DateTypeDef BackupDate;
	RTC_TimeTypeDef DummyTime;
	uint32_t dateMem;
	dateMem = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2 + 1);
	dateMem |= HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) << 16;
	memcpy(&BackupDate, &dateMem, sizeof(uint32_t));
	if (IS_RTC_YEAR(BackupDate.Year) && IS_RTC_MONTH(BackupDate.Month) && IS_RTC_DATE(BackupDate.Date))
	{
		/* Change the date retrieved from the backup registers */
		hrtc.DateToUpdate.Year = BackupDate.Year;
		hrtc.DateToUpdate.Month = BackupDate.Month;
		hrtc.DateToUpdate.Date = BackupDate.Date;
		// Check for valid weekday separately so that if there is a problem we have still at least set the date
		hrtc.DateToUpdate.WeekDay = BackupDate.WeekDay;
		/* Read the time so that the date is rolled over if required */
		HAL_RTC_GetTime(&hrtc, &DummyTime, RTC_FORMAT_BIN);
		/* Store the date or it will revert again if we lose power before manually updating. */
		if (BackupDate.Date != hrtc.DateToUpdate.Date)
		{
			RTC_Backup();
		}
	}
}

void RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second) {
	RTC_TimeTypeDef sTime = { 0 };
	sTime.Hours = hour;
	sTime.Minutes = minute;
	sTime.Seconds = second;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void RTC_SetDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday) {
	RTC_DateTypeDef DateToUpdate = { 0 };
	DateToUpdate.WeekDay = weekday;
	DateToUpdate.Month = month;
	DateToUpdate.Date = day;
	DateToUpdate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void RTC_GetTime(uint8_t* hour, uint8_t* minute, uint8_t* second) {
	RTC_TimeTypeDef sTime = { 0 };
	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	*hour = sTime.Hours;
	*minute = sTime.Minutes;
	*second = sTime.Seconds;
}

void RTC_GetDate(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday) {
	RTC_DateTypeDef DateToUpdate = { 0 };
	if (HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	*year = DateToUpdate.Year;
	*month = DateToUpdate.Month;
	*day = DateToUpdate.Date;
	*weekday = DateToUpdate.WeekDay;
}

void HAL_RTC_MspInit(RTC_HandleTypeDef *rtcHandle) {
	if (rtcHandle->Instance == RTC) {
		HAL_PWR_EnableBkUpAccess();
		/* Enable BKP CLK enable for backup registers */
		__HAL_RCC_BKP_CLK_ENABLE();
		/* RTC clock enable */
		__HAL_RCC_RTC_ENABLE();
	}
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *rtcHandle) {
	if (rtcHandle->Instance == RTC) {
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE();
	}
}
