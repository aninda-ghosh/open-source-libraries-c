## Description

This library includes libraries for AT24C32 EEPROM based on the I2C write functions of STM32 MCUs. The library can be utilized for other AT series of chips as well.

## Things to keep in mind while using

- Currently the library supports 4 functions
  - Byte Read (Individual addresses)
  - Byte Write (Individual addresses)
  - Page Write (Individual Pages) based on page Number
  - Page Read (Individual Pages) based on page Number

## Usage

- Initialization

  ```
  EEPROM eeprom(&hi2c1, 0x50, 32, 32)   
  
  <Parameters : i2c Handle, Address, Chip size in Kb, Page Size in Bytes>
  ```

- Byte Read Write

  ```
  <Parameters : Memory address, Data>
  eeprom.byteWrite(0x00, 0xAB);

  uint8_t data = 0x00;
  <Parameters : Memory address, Data>
  eeprom.byteRead(0x00, &data);
  ```

- Page Read Write

  ```
  uint8_t byteseq[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  <Parameters : Page Number, Offset, Data, Datalen>
  eeprom.pageWrite(1, 0, byteseq, 10);          
  
  uint8_t pageData[10] = {0x00};
  <Parameters : Page Number, Offset, Data, Datalen>
  eeprom.pageRead(1, 0, pageData, 10);
  ```
