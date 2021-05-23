## Description

This library includes libraries for a simple wear levelling algorithm for writing config and logs in a EEPROM based storage device.

Algorithm:

- The entire memory is divided into 25% and 75% segment
- 25% of the memory will be used for config storage. 
- 75% of the memory will be used for logs storage.
- Each memory segment will be partitioned further to evenly distribute the memory in it's entire lifetime.
  - Each subpartition will contain the current erase cycle of the partition and the data.
  - With any kind of write operation to the partition will increase the erase cycle accordingly.
  - Whenever the erase cycle crosses the wea threshold set by user the entire data will be shifted to next partition and the cycle continues till all the allocated partitions are worn out.

## Things to keep in mind while using

- Currently the library supports 4 functions
  - Format memory
  - Write config
  - Read config
  - Write logs
  - Read logs

## Usage

- Initialization
  
  ```
  STORAGE memory(&eeprom, 1, 4, 600000);   
  
  <Parameters : Storage Type Handle, No of Pages for each Config partition, No of pages for each Logs partition, Wear Threshold>
  ```

- Config Read Write
  
  ```
  <Parameters : Data, Data len>
  memory.writeConfig(byteseq, 10);

  <Parameters : Data, Data len>
  memory.readConfig(pageData, 10);
  ```
