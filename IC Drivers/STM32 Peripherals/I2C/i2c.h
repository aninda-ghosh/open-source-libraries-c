#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle);


void I2C_Init(uint32_t frequency);
int8_t i2cReadRegisters(uint8_t devAddress, uint8_t subAddress, uint8_t count, uint8_t *data);
int8_t i2cWriteRegister(uint8_t devAddress, uint8_t subAddress, uint8_t count, uint8_t data);
int8_t i2cIsDevReady(uint8_t devAddress);

#ifdef __cplusplus
}
#endif

#endif
