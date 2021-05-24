/*
 * storage_memory.h
 *
 *  Created on: 22-May-2021
 *      Author: aninda
 */

#ifndef INC_STORAGE_MEMORY_H_
#define INC_STORAGE_MEMORY_H_

#include "main.h"
#include "usart.h"
#include "eeprom.h"

class STORAGE {
public:
	STORAGE(EEPROM *StorageHandle, uint16_t ConfigPartion_Size, uint16_t LogPartition_Size, uint32_t ThresholdWear);
	void setup(void);
	void format(void);
	void dump(void);
	int8_t writeConfig(uint8_t *data, uint8_t datalen);
	int8_t readConfig(uint8_t *data, uint8_t datalen);
private:
	uint32_t wearThreshold;			//No of erase cycles a page should sustain.

	EEPROM *Storage;

	uint16_t configPartition_start;
	uint16_t configPartition_end;
	uint16_t configPartition_pagesInPartition;		//Number of pages in an inner partition.
	uint16_t configPartition_partitionCount;		//Number of inner partitions that can exist.

	uint16_t logPartition_start;
	uint16_t logPartition_end;
	uint16_t logPartition_pagesInPartition;			//Number of pages in an inner partition.
	uint16_t logPartition_partitionCount;			//Number of inner partitions that can exist.
};



#endif /* INC_STORAGE_MEMORY_H_ */
