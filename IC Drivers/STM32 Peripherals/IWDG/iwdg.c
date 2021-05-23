/* Includes ------------------------------------------------------------------*/
#include "iwdg.h"

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void IWDGInit(void) {
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
}
