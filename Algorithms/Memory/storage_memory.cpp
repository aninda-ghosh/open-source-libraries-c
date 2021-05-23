/*
 * storage_memory.cpp
 *
 *  Created on: 22-May-2021
 *      Author: aninda
 */

#include "storage_memory.h"

STORAGE::STORAGE(EEPROM *StorageHandle, uint16_t ConfigPartion_Size, uint16_t LogPartition_Size, uint32_t ThresholdWear) {
	Storage = StorageHandle;
	configPartition_size = ConfigPartion_Size;
	logPartition_size = LogPartition_Size;
	wearThreshold = ThresholdWear;

	/**
	 * 25% of the entire memory will be used for config.
	 * 75% of rest of the memory will be used for logs.
	 * This functions does all the partitioning.
	 */
	configPartition_start = 0x0000;
	configPartition_stop  = configPartition_start + (Storage->chipSize/4) - 1;
	configPartition_Count = (configPartition_stop - configPartition_start + 1) / configPartition_size;

	logPartition_start = configPartition_stop + 1;	//Just the next memory where the configuration partition ends.
	logPartition_stop  = logPartition_start + ((Storage->chipSize*3)/4) - 1;
	logPartition_Count = (logPartition_stop - logPartition_start + 1) / logPartition_size;
}

void STORAGE::setup(void) {
	//Settle the config memory erase counts, initially the storage units generally have 0xFF in their bytes
	uint16_t currentAddress = configPartition_start;
	while(currentAddress < configPartition_stop) {
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}erase_cycle;
		Storage->pageRead((uint8_t)(currentAddress/Storage->pageSize), 0, erase_cycle._8bit, 4);
		virtualDelay();
		EEPROM_DEBUG("CurrentAddress[%d], Read Data[%ld], PartitionDetails[%d, %d]\n", currentAddress, erase_cycle._32bit, configPartition_start, configPartition_stop);
		if(erase_cycle._8bit[0] == 0xFF && erase_cycle._8bit[1] == 0xFF && erase_cycle._8bit[2] == 0xFF && erase_cycle._8bit[3] == 0xFF) {
			EEPROM_DEBUG("Found Virgin Page[%d]\n", (uint8_t)(currentAddress/Storage->pageSize));
			//If all of the memory is 0xFF then it's a virgin chip or not formatted properly
			erase_cycle._32bit = 1;
			Storage->pageWrite((uint8_t)(currentAddress/Storage->pageSize), 0, erase_cycle._8bit, 4);
			virtualDelay();
		}
		currentAddress += Storage->pageSize;
	}
}

void STORAGE::format(void) {
	static uint16_t currentAddress = configPartition_start;
	uint8_t formatData[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	while(currentAddress < logPartition_stop) {
		//Delete page by page
		uint8_t pgNo = (uint8_t)(currentAddress/Storage->pageSize);
		Storage->pageWrite(pgNo, 0, (uint8_t*)formatData, 4);
		virtualDelay();
		currentAddress += Storage->pageSize;
	}
}

int8_t STORAGE::writeConfig(uint8_t *data, uint8_t datalen) {
	if(datalen <= Storage->pageSize) { //Check if the config data length is less than the mentioned size. e.g. 1 page
		/**
		 * Steps involved in writing the config data
		 * Scan the first 4 bytes of every page and start writing where the Erase cycle < Wear threshold
		 * Write the config in the same page and increment the Erase cycle.
		 */
		int8_t ret=0;
		uint16_t error = 0;
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}erase_cycle;
		uint16_t currentAddress = configPartition_start;
		uint16_t perfectAddress = currentAddress;
		erase_cycle._32bit = wearThreshold+1;
		//This is where we are searching for our perfect page
		while(erase_cycle._32bit > wearThreshold && currentAddress <= (configPartition_stop + 1 - Storage->pageSize)){
			ret = Storage->pageRead((uint8_t)(currentAddress/Storage->pageSize), 0, erase_cycle._8bit, 4);
			virtualDelay();
			if(ret<0){
				//The Page's memory is corrupted
				erase_cycle._32bit = wearThreshold+1;	// Just set the erase cycle to ignore the corrupted page
				error ++;
			}else{
				perfectAddress = currentAddress;
				error = 0;
			}
			EEPROM_DEBUG("(Config Write) Current Erase Cycle [%ld] of Page Number [%d]\n", erase_cycle._32bit, (uint8_t)(currentAddress/Storage->pageSize));
			currentAddress += Storage->pageSize;	//Increment the address for reading by Pagesize for the given Storage chip
		}

		if(!error && perfectAddress <= (configPartition_stop + 1 - Storage->pageSize) && erase_cycle._32bit < wearThreshold) { //Now it's time to write the data if there are no pending errors
			//Increase the erase cycle by 2
			uint8_t pgNo = (uint8_t)(perfectAddress/Storage->pageSize);
			if(erase_cycle._32bit+2 > wearThreshold) {	//We need to change the page for next write
				erase_cycle._32bit += 2;
				Storage->pageWrite(pgNo, 0, erase_cycle._8bit, 4);
				virtualDelay();
				erase_cycle._32bit = 1;	//Set the base for the next page
				pgNo++;	//Write to the next page
			}
			erase_cycle._32bit += 2;
			ret = Storage->pageWrite(pgNo, 4, data, datalen);
			virtualDelay();
			Storage->pageWrite(pgNo, 0, erase_cycle._8bit, 4);
			virtualDelay();
			return 0;
		}else{
			return -1;	//Error Occurred
		}
	}else{
		return -1;
	}
}

int8_t STORAGE::readConfig(uint8_t *data, uint8_t datalen) {
	if(datalen <= Storage->pageSize) { //Check if the config data length is less than the mentioned size. e.g. 1 page
		/**
		 * Steps involved in writing the config data
		 * Scan the first 4 bytes of every page and start reading where the Erase cycle < Wear threshold
		 * Read the config from the same page
		 */
		int8_t ret=0;
		uint16_t error = 0;
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}erase_cycle;
		uint16_t currentAddress = configPartition_start;
		uint16_t perfectAddress = currentAddress;
		erase_cycle._32bit = wearThreshold+1;
		//This is where we are searching for our perfect page
		while(erase_cycle._32bit > wearThreshold && currentAddress <= (configPartition_stop + 1 - Storage->pageSize)){
			Storage->pageRead((uint8_t)(currentAddress/Storage->pageSize), 0, erase_cycle._8bit, 4);
			virtualDelay();
			if(ret<0){
				//The Page's memory is corrupted
				erase_cycle._32bit = wearThreshold+1;	// Just set the erase cycle to ignore the corrupted page
				error ++;
			}else{
				perfectAddress = currentAddress;
				error = 0;
			}
			EEPROM_DEBUG("(Config Read) Current Erase Cycle [%ld] of Page Number [%d]\n", erase_cycle._32bit, (uint8_t)(currentAddress/Storage->pageSize));
			currentAddress += Storage->pageSize;	//Increment the address for reading by Pagesize for the given Storage chip
		}
		if(perfectAddress <= (configPartition_stop + 1 - Storage->pageSize) && erase_cycle._32bit < wearThreshold) { //Now it's time to read the data
			Storage->pageRead((uint8_t)(perfectAddress/Storage->pageSize), 4, data, datalen);
		}
	}
	return 0;
}

//int8_t STORAGE::readLogs(uint8_t *data, uint8_t noOfLogs) {
//
//}

//int8_t STORAGE::writeLogs(uint8_t *data, uint8_t noOfLogs) {
//
//}
