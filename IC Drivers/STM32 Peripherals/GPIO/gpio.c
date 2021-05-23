/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

void gpioInit(GPIO_TypeDef  *port, uint16_t pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if(port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if(port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if(port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void gpioInputInit(GPIO_TypeDef  *port, uint16_t pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if(port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if(port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if(port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void gpioHigh(GPIO_TypeDef  *port, uint16_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void gpioLow(GPIO_TypeDef  *port, uint16_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void gpioToggle(GPIO_TypeDef  *port, uint16_t pin) {
	HAL_GPIO_TogglePin(port, pin);
}

uint8_t gpioRead(GPIO_TypeDef  *port, uint16_t pin) {
	return HAL_GPIO_ReadPin(port, pin);
}

void MULTIPLEXER_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pins : PAPin PAPin */
	GPIO_InitStruct.Pin = ANALOGSWITCH1_Pin | ANALOGSWITCH2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void MULTIPLEXER_SwitchState(uint8_t state) {
	switch(state) {
		case UARTDEBUG:{
			HAL_GPIO_WritePin(ANALOGSWITCH1_GPIO_Port, ANALOGSWITCH1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANALOGSWITCH2_GPIO_Port, ANALOGSWITCH2_Pin, GPIO_PIN_RESET);
			break;
		}
		case STLINK:{
			HAL_GPIO_WritePin(ANALOGSWITCH1_GPIO_Port, ANALOGSWITCH1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ANALOGSWITCH2_GPIO_Port, ANALOGSWITCH2_Pin, GPIO_PIN_RESET);
			break;
		}
		case BM62PROG:{
			HAL_GPIO_WritePin(ANALOGSWITCH1_GPIO_Port, ANALOGSWITCH1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ANALOGSWITCH2_GPIO_Port, ANALOGSWITCH2_Pin, GPIO_PIN_SET);
			break;
		}
	}
}


void AZOTEQ_Swipe_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = SWIPELEFTPAD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWIPELEFTPAD_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = SWIPERIGHTPAD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWIPERIGHTPAD_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void TP4056_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = STDBY_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(STDBY_GPIO_Port, &GPIO_InitStruct);

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = CHRG_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(CHRG_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void AZOTEQ_HelmetWear_Init(void) {
	gpioInputInit(HELMETWEARPAD_GPIO_Port, HELMETWEARPAD_Pin);
}

uint8_t AZOTEQ_HelmetWear_Status(void) {
	return !gpioRead(HELMETWEARPAD_GPIO_Port, HELMETWEARPAD_Pin);
}
