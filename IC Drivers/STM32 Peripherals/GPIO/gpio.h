#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void gpioInit(GPIO_TypeDef  *port, uint16_t pin);
void gpioInputInit(GPIO_TypeDef  *port, uint16_t pin);
void gpioHigh(GPIO_TypeDef  *port, uint16_t pin);
void gpioLow(GPIO_TypeDef  *port, uint16_t pin);
void gpioToggle(GPIO_TypeDef  *port, uint16_t pin);
uint8_t gpioRead(GPIO_TypeDef  *port, uint16_t pin);

void MULTIPLEXER_Init(void);
void MULTIPLEXER_SwitchState(uint8_t state);

void AZOTEQ_Swipe_Init(void);
void TP4056_Init(void);

void AZOTEQ_HelmetWear_Init(void);
uint8_t AZOTEQ_HelmetWear_Status(void);

#ifdef __cplusplus
}
#endif
#endif
