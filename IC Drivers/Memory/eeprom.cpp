/*
 * eeprom.cpp
 *
 *  Created on: 11-May-2021
 *      Author: aninda
 */

#include "eeprom.h"

EEPROM::EEPROM(I2C_HandleTypeDef *_handle, uint8_t _address, uint8_t _size,
		uint8_t _pagesize) {
	eeprom_handle = _handle;
	devAddress = _address;
	chipSize = (_size*1024)/8;
	pageSize = _pagesize;
	noOfPages = chipSize / pageSize;
}

int8_t EEPROM::pageWrite(uint8_t pageNumber, uint8_t offset, uint8_t *data,
		uint8_t datalen) {
	if (datalen + offset <= pageSize && pageNumber < noOfPages) {
		uint16_t memoryAddress = (pageNumber * pageSize) + offset;
		uint8_t _data[32] = { 0 };
		_data[0] = uint8_t(memoryAddress >> 8) & 0xFF;
		_data[1] = uint8_t(memoryAddress) & 0xFF;
		memcpy(_data+2, data, datalen);
		return HAL_I2C_Master_Transmit(eeprom_handle, (devAddress << 1), _data,
				datalen+2, 1000);
	} else
		return -1;
}

int8_t EEPROM::pageRead(uint8_t pageNumber, uint8_t offset, uint8_t *data,
		uint8_t datalen) {
	if (datalen + offset <= pageSize && pageNumber < noOfPages) {
		uint16_t memoryAddress = (pageNumber * pageSize) + offset;
		uint8_t _data[2] = { 0 };
		_data[0] = uint8_t(memoryAddress >> 8) & 0xFF;
		_data[1] = uint8_t(memoryAddress) & 0xFF;
		HAL_I2C_Master_Transmit(eeprom_handle, (devAddress << 1), _data, 2,
				1000);
		return HAL_I2C_Master_Receive(eeprom_handle, (devAddress << 1) + 0x01,
				data, datalen, 1000);
	} else
		return -1;
}

int8_t EEPROM::byteWrite(uint16_t memoryAddress, uint8_t data) {
	uint8_t _data[3] = { 0 };
	_data[0] = uint8_t(memoryAddress >> 8) & 0xFF;
	_data[1] = uint8_t(memoryAddress) & 0xFF;
	_data[2] = data;
	return HAL_I2C_Master_Transmit(eeprom_handle, (devAddress << 1), _data, 3,
			1000);
}

int8_t EEPROM::byteRead(uint16_t memoryAddress, uint8_t *data) {
	uint8_t _data[2] = { 0 };
	_data[0] = uint8_t(memoryAddress >> 8) & 0xFF;
	_data[1] = uint8_t(memoryAddress) & 0xFF;
	HAL_I2C_Master_Transmit(eeprom_handle, (devAddress << 1), _data, 2, 100);
	return HAL_I2C_Master_Receive(eeprom_handle, (devAddress << 1) + 0x01, data,
			1, 1000);
}

int8_t EEPROM::eraseChip(void) {

}
