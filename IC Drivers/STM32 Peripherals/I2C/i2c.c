/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/*
 * I2C handle
 */

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (i2cHandle->Instance == I2C1) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_RCC_I2C1_CLK_ENABLE();
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle) {

	if (i2cHandle->Instance == I2C1) {
		__HAL_RCC_I2C1_CLK_DISABLE();
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB7     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
	}
}

/* I2C1 init function */
void I2C_Init(uint32_t frequency) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = frequency;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
}

int8_t i2cIsDevReady(uint8_t devAddress) {
	if (HAL_I2C_IsDeviceReady(&hi2c1, devAddress, 1, 1000) != HAL_OK) {
		return -1;
	}
	return 0;
}

int8_t i2cReadRegisters(uint8_t devAddress, uint8_t subAddress, uint8_t count,
		uint8_t *data) {
	int8_t ret;
	uint8_t memaddr[1];
	memaddr[0] = subAddress;
	ret = HAL_I2C_Master_Transmit(&hi2c1, devAddress, memaddr, sizeof(memaddr),
			100);
	if (ret != HAL_OK) {
		return -1;
	} else {
		ret = HAL_I2C_Master_Receive(&hi2c1, devAddress, data, count, 100);
		if (ret != HAL_OK) {
			return -1;
		}
	}
	return 0;
}

int8_t i2cWriteRegister(uint8_t devAddress, uint8_t subAddress, uint8_t count,
		uint8_t data) {
	int8_t ret;
	uint8_t memaddr_data[2];
	memaddr_data[0] = subAddress;
	memaddr_data[1] = data;
	ret = HAL_I2C_Master_Transmit(&hi2c1, devAddress, memaddr_data,
			sizeof(memaddr_data), 100);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

