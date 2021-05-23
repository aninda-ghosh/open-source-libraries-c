/*
 * eeprom.h
 *
 *  Created on: 11-May-2021
 *      Author: aninda
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"

class EEPROM {
private:
	I2C_HandleTypeDef* eeprom_handle;
	uint16_t devAddress;
public:
	uint16_t chipSize;
	uint16_t pageSize;
	uint16_t noOfPages;

	EEPROM(I2C_HandleTypeDef *_handle, uint8_t _address, uint8_t _size, uint8_t _pagesize);
	int8_t pageWrite(uint8_t pageNumber, uint8_t offset,  uint8_t* data, uint8_t datalen);
	int8_t pageRead(uint8_t pageNumber, uint8_t offset, uint8_t* data, uint8_t datalen);
	int8_t byteWrite(uint16_t memoryAddress, uint8_t data);
	int8_t byteRead(uint16_t memoryAddress, uint8_t* data);
	int8_t eraseChip(void);
};

#endif /* INC_EEPROM_H_ */
