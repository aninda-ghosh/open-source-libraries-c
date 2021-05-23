#include "usart.h"

extern RINGBUFFER ringBuf;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

#define UART3_rxBuffer_size 20
uint8_t UART3_rxBuffer[UART3_rxBuffer_size];
uint8_t UART3_recv;

#define UART1_rxBuffer_size 12
char UART1_rxBuffer[UART1_rxBuffer_size];
uint8_t UART1_recv;

/* USART1 Initialization function */
void USART1_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

/* USART3 Initialization function */
void USART3_Init(void) {
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
}

void USART3_DMAInit(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @fn void USART3_Write(uint8_t*, uint16_t)
 * @brief This function is used to transmit data to BM62 via UART3
 * @param data
 * @param datalen
 */
void USART3_Write(uint8_t *data, uint16_t datalen) {
	HAL_UART_Transmit(&huart3, data, datalen, 2000);
}

/**
 * UART MSP Initialization Routine
 */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (uartHandle->Instance == USART1) {
		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
	} else if (uartHandle->Instance == USART3) {
		__HAL_RCC_USART3_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**USART3 GPIO Configuration
		 PB10     ------> USART3_TX
		 PB11     ------> USART3_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USART3 DMA Init */
		/* USART3_RX Init */
		hdma_usart3_rx.Instance = DMA1_Channel3;
		hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
		hdma_usart3_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
		if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
			Error_Handler();
		}

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);

		USART3_DMAInit();
	}
}

/**
 * UART MSP De-Initialization Routine
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle) {

	if (uartHandle->Instance == USART1) {
		__HAL_RCC_USART1_CLK_DISABLE();
		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
	} else if (uartHandle->Instance == USART3) {
		__HAL_RCC_USART3_CLK_DISABLE();
		/**USART3 GPIO Configuration
		 PB10     ------> USART3_TX
		 PB11     ------> USART3_RX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
		HAL_NVIC_DisableIRQ(USART3_IRQn);
		HAL_DMA_DeInit(uartHandle->hdmarx);
	}
}

void USART3_ITReceive(void) {
	HAL_UART_Receive_IT(&huart3, &UART3_recv, 1);
}

void USART1_ITReceive(void) {
	HAL_UART_Receive_IT(&huart1, &UART1_recv, 1);
}

void USART3_DMAReceive(void) {
	HAL_UART_Receive_DMA(&huart3, (uint8_t *)&UART3_rxBuffer, UART3_rxBuffer_size);
}

void USART3_DMADisable(void) {
	__HAL_DMA_DISABLE(&hdma_usart3_rx);
}

void USART3_DMAEnable(void) {
	__HAL_DMA_ENABLE(&hdma_usart3_rx);
}

// USART Error Handler
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		HAL_UART_DeInit(&huart3);
		USART3_Init();
		USART3_DMAReceive();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		SYSTEM_DEBUG("Received [%d]\n", UART1_recv);
		UART1_recv -= 48;
		if(UART1_recv == 0)			//0
			debugenable.rtc = 0;
		else if(UART1_recv == 1) 	//1
			debugenable.rtc = 1;
		else if(UART1_recv == 2)  	//2
			debugenable.bm62 = 0;
		else if(UART1_recv == 3)  	//3
			debugenable.bm62 = 1;
		else if(UART1_recv == 4)  	//4
			debugenable.mpu = 0;
		else if(UART1_recv == 5)  	//5
			debugenable.mpu = 1;
		else if(UART1_recv == 6)  	//6
			debugenable.mpu = 0;
		else if(UART1_recv == 7)  	//7
			debugenable.mpu = 1;
		else if(UART1_recv == 8)  	//8
			debugenable.tp4056 = 0;
		else if(UART1_recv == 9)  	//9
			debugenable.tp4056 = 1;

		USART1_ITReceive();
	} else if (huart->Instance == USART3) {
		ringBuf.enqueue_arr(UART3_rxBuffer+(UART3_rxBuffer_size/2),UART3_rxBuffer_size/2);
		USART3_DMAReceive();
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		ringBuf.enqueue_arr(UART3_rxBuffer,UART3_rxBuffer_size/2);
	}
}
