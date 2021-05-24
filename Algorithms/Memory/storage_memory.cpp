/*
 * storage_memory.cpp
 *
 *  Created on: 22-May-2021
 *      Author: aninda
 */

#include "storage_memory.h"

STORAGE::STORAGE(EEPROM *StorageHandle, uint16_t ConfigPartion_Size, uint16_t LogPartition_Size, uint32_t ThresholdWear) {
	Storage = StorageHandle;
	configPartition_pagesInPartition = ConfigPartion_Size*Storage->pageSize;
	logPartition_pagesInPartition = LogPartition_Size*Storage->pageSize;
	wearThreshold = ThresholdWear;

	/**
	 * 25% of the entire memory will be used for config.
	 * 75% of rest of the memory will be used for logs.
	 * This functions does all the partitioning.
	 */
	configPartition_start = 0x0000;
	configPartition_end  = configPartition_start + (Storage->chipSize/4) - 1;
	configPartition_partitionCount = (configPartition_end - configPartition_start + 1) / configPartition_pagesInPartition;

	logPartition_start = configPartition_end + 1;	//Just the next memory where the configuration partition ends.
	logPartition_end  = logPartition_start + ((Storage->chipSize*3)/4) - 1;
	logPartition_partitionCount = (logPartition_end - logPartition_start + 1) / logPartition_pagesInPartition;
}

void STORAGE::setup(void) {
	EEPROM_DEBUG("\n\nStorage Details [%ld wear cycles]\n", wearThreshold);
	EEPROM_DEBUG("Config:\t[0x%04X, 0x%04X, %d pages, %d partitions]\n", configPartition_start, configPartition_end, configPartition_pagesInPartition, configPartition_partitionCount);
	EEPROM_DEBUG("Logs:\t[0x%04X, 0x%04X, %d pages, %d partitions]\n\n", logPartition_start, logPartition_end, logPartition_pagesInPartition, logPartition_partitionCount);
	int8_t error = -1;
	//Settle the config memory erase counts, initially the storage units generally have 0xFF in their bytes
	uint16_t currentpartition = 0;
	while(currentpartition < configPartition_partitionCount) {
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}partition_erase_cycle;

		uint16_t currAddress = currentpartition*configPartition_pagesInPartition;

		error = Storage->pageRead(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
		virtualDelay(5);
		EEPROM_DEBUG("CurrentPartition[%d, %d], PartitionDetails[%ld]\n", currentpartition, currAddress, partition_erase_cycle._32bit);

		//This step is necessary to identify a virgin chip
		if(partition_erase_cycle._8bit[0] == 0xFF && partition_erase_cycle._8bit[1] == 0xFF && partition_erase_cycle._8bit[2] == 0xFF && partition_erase_cycle._8bit[3] == 0xFF) {
			EEPROM_DEBUG("Found Virgin Partition[%d]\n", currAddress);
			//If all of the memory is 0xFF then it's a virgin chip or not formatted properly
			partition_erase_cycle._32bit = 1;
			Storage->pageWrite(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);
		}
		currentpartition ++;
	}

	currentpartition = 0;
	while(currentpartition < logPartition_partitionCount) {
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}partition_erase_cycle;

		uint16_t currAddress = (currentpartition*logPartition_pagesInPartition) + logPartition_start;

		error = Storage->pageRead(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
		virtualDelay(5);
		EEPROM_DEBUG("CurrentPartition[%d, %d], PartitionDetails[%ld]\n", currentpartition, currAddress, partition_erase_cycle._32bit);

		//This step is necessary to identify a virgin chip
		if(partition_erase_cycle._8bit[0] == 0xFF && partition_erase_cycle._8bit[1] == 0xFF && partition_erase_cycle._8bit[2] == 0xFF && partition_erase_cycle._8bit[3] == 0xFF) {
			EEPROM_DEBUG("Found Virgin Partition[%d]\n", currAddress);
			//If all of the memory is 0xFF then it's a virgin chip or not formatted properly
			partition_erase_cycle._32bit = 1;
			Storage->pageWrite(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);
		}
		currentpartition ++;
	}
}

void STORAGE::format(void) {
	static uint16_t currentAddress = configPartition_start;
	uint8_t formatData[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	while(currentAddress < logPartition_end) {
		//Delete page by page
		uint16_t pgNo = (currentAddress/Storage->pageSize);
		Storage->pageWrite(pgNo, 0, (uint8_t*)formatData, 4);
		virtualDelay(5);
		currentAddress += Storage->pageSize;
	}
}

int8_t STORAGE::writeConfig(uint8_t *data, uint8_t datalen) {
	if(datalen <= Storage->pageSize) { //Check if the config data length is less than the mentioned size. e.g. 1 page
		/**
		 * Steps involved in writing the config data
		 * Figure out which partition shall I choose for writing
		 * Write the config in the same page and increment the Erase cycle.
		 */

		//Figuring out the partition to choose from
		uint16_t currentPartition = 0;
		int8_t error = -1;
		uint8_t partitionFound = 0;
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}partition_erase_cycle;

		while(currentPartition < configPartition_partitionCount) {
			uint16_t currAddress = currentPartition*configPartition_pagesInPartition;
			error = Storage->pageRead(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);

			EEPROM_DEBUG("Before Writing Status [%d, %d, %ld]\n\n", currentPartition, currAddress, partition_erase_cycle._32bit);


			if(partition_erase_cycle._32bit < wearThreshold){
				partitionFound = 1;
				break;
			}

			currentPartition++;
		}

		//Prepare the data and the respective erase cycle for writing
		if(!error && partitionFound) {
			partition_erase_cycle._32bit += 2;
			uint16_t currAddress = currentPartition*configPartition_pagesInPartition;
			error = Storage->pageWrite(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);
			error = Storage->pageRead(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);

			currAddress += 4;
			for(uint16_t ptr=0; ptr < datalen; ptr++) {
				error = Storage->byteWrite(currAddress++, data[ptr]);
				virtualDelay(3);
			}
			EEPROM_DEBUG("After Writing Status [%d, %ld]\n\n", currentPartition, partition_erase_cycle._32bit);
			return 0;
		}else{
			return -1;
		}
	}else{
		return -1;
	}
}

int8_t STORAGE::readConfig(uint8_t *data, uint8_t datalen) {
	if(datalen <= Storage->pageSize) { //Check if the config data length is less than the mentioned size. e.g. 1 page
		/**
		 * Steps involved in writing the config data
		 * Figure out which partition shall I choose for reading
		 * Write the config from the same page
		 */

		//Figuring out the partition to choose from
		uint16_t currentPartition = 0;
		int8_t error = -1;
		uint8_t partitionFound = 0;
		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}partition_erase_cycle;

		union {
			uint32_t _32bit;
			uint8_t	_8bit[4];
		}next_partition_erase_cycle;

		while(currentPartition < configPartition_partitionCount) {
			uint16_t currAddress = currentPartition*configPartition_pagesInPartition;
			error = Storage->pageRead(currAddress/Storage->pageSize, 0, partition_erase_cycle._8bit, 4);
			virtualDelay(5);

			//Get the next partition erase count
			if(currentPartition+1 < configPartition_partitionCount) {
				error = Storage->pageRead((currAddress/Storage->pageSize)+1, 0, next_partition_erase_cycle._8bit, 4);
				virtualDelay(5);
			}

			EEPROM_DEBUG("Before Reading Status Curr Partition[%d, %d, %ld], Next Partition[%ld]\n\n", currentPartition, currAddress, partition_erase_cycle._32bit, next_partition_erase_cycle._32bit);


			if(partition_erase_cycle._32bit < wearThreshold || (partition_erase_cycle._32bit == wearThreshold && next_partition_erase_cycle._32bit == 1)){
				partitionFound = 1;
				break;
			}else if((partition_erase_cycle._32bit == wearThreshold && next_partition_erase_cycle._32bit == 1 && next_partition_erase_cycle._32bit < wearThreshold)){
				currentPartition+=1;
				partitionFound = 1;
				break;
			}
			currentPartition++;
		}
		if(!error && partitionFound) {
			EEPROM_DEBUG("Read From Partition [%d]\n", currentPartition);
			uint16_t currAddress = currentPartition*configPartition_pagesInPartition;
			error = Storage->pageRead(currAddress/Storage->pageSize, 4, data, datalen);
			virtualDelay(5);
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
