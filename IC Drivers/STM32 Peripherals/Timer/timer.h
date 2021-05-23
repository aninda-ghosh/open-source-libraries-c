/*
 * internal_timer.h
 *
 *  Created on: 01-May-2021
 *      Author: anind
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim2;

void timer2Init(void);
void timer2Enable(void);
void timer2Disable(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIMER_H_ */
