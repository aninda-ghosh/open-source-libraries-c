#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern IWDG_HandleTypeDef hiwdg;

void IWDGInit(void);

#ifdef __cplusplus
}
#endif

#endif
