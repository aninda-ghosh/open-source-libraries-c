/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ringBuffer.h"

#define DBG_BUF_LEN     512

extern char DBG_BUFFER[];
extern DebugEnable debugenable;

extern uint8_t UART3_rxBuffer[];
extern uint16_t writePosition;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void USART1_Init(void);
void USART1_ITReceive(void);
void USART3_Init(void);
void USART3_DMAInit(void);
void USART3_Write(uint8_t* data, uint16_t datalen);
void USART3_ITReceive(void);
void USART3_DMAReceive(void);
uint32_t USART3_CNDTR(void);
void USART3_DMADisable(void);
void USART3_DMAEnable(void);


//! Debug definitions
//! System Debug
#define SYSTEM_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[SYSTEM]: "); \
    sprintf(DBG_BUFFER + strlen("[SYSTEM]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.system)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! MPU Debug
#define MPU_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[MPU]: "); \
    sprintf(DBG_BUFFER + strlen("[MPU]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.mpu)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! BM62 Debug
#define BM62_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[BM62]: "); \
    sprintf(DBG_BUFFER + strlen("[BM62]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.bm62)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! RTC Debug
#define RTC_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[RTC]: "); \
    sprintf(DBG_BUFFER + strlen("[RTC]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.rtc)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! TP4056 Debug
#define TP4056_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[TP4056]: "); \
    sprintf(DBG_BUFFER + strlen("[TP4056]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.tp4056)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! EEPROM Debug
#define EEPROM_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    strcpy(DBG_BUFFER, "[EEPROM]: "); \
    sprintf(DBG_BUFFER + strlen("[EEPROM]: "),FORMAT,##__VA_ARGS__); \
	if(debugenable.eeprom)\
		HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

//! Simple Debug
#define SIMPLE_DEBUG(FORMAT,...) {\
    memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
	HAL_UART_Transmit(&huart1, (uint8_t*)(DBG_BUFFER), strlen((const char *)(DBG_BUFFER)), 2000);\
}

#ifdef __cplusplus
}
#endif

#endif
