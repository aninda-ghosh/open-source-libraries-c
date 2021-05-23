/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern RTC_HandleTypeDef 	rtc_handle;

#define CURRENT_CENTURY 	2000
#define RTC_BACKUP_INDEX 	RTC_BKP_DR1
#define RTC_BACKUP_VALUE	0x32F2

void 		RTC_Init(void);
uint8_t 	RTC_IsTimeSet(void);
uint32_t 	RTC_GetEpoch(void);
void 		RTC_SetEpoch(uint32_t ts);
void 		RTC_Backup(void);
void 		RTC_Restore(void);
void 		RTC_SetTime(uint8_t hour, uint8_t minute, uint8_t second);
void 		RTC_SetDate(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday);
void 		RTC_GetTime(uint8_t* hour, uint8_t* minute, uint8_t* second);
void 		RTC_GetDate(uint8_t* year, uint8_t* month, uint8_t* day, uint8_t* weekday);

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
