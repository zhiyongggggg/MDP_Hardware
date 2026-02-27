#include <UART_Comm.h>
#include "stdlib.h"
#include <oled.h>
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef* huart3;
static DMA_HandleTypeDef* hdma_usart3_rx;
#define RxBuf_SIZE 100
#define MainBuf_SIZE 100

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];


/*

void UART_Comm_Init(UART_HandleTypeDef *huart){
	huart3 = huart;
	HAL_UART_Receive_IT(huart3,(uint8_t *)MainBuf, 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//prevent unused argument(s)compilation warning
	UNUSED(huart);

	HAL_UART_Receive_IT(huart3,(uint8_t *)MainBuf,30);
	//HAL_UART_Transmit(huart3,(uint8_t *)RxBuf,10,0xFFFF);
}
*/

void UART_Comm_Init(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma_usart_rx){
	huart3 = huart;
	hdma_usart3_rx = hdma_usart_rx;

	HAL_UARTEx_ReceiveToIdle_DMA(huart3, RxBuf, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_usart3_rx, DMA_IT_HT);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART3)
	{


		//clear the MainBuffer
		//Clear_Buffer();
		//HAL_Delay(10);
		//memcpy ((uint8_t *)MainBuf, RxBuf, Size);


		// start the DMA again
		//HAL_UARTEx_ReceiveToIdle_DMA(huart3, (uint8_t *) RxBuf, RxBuf_SIZE);
		//__HAL_DMA_DISABLE_IT(hdma_usart3_rx, DMA_IT_HT);


	        // Copy received data to MainBuf and null terminate
	        memcpy((uint8_t *)MainBuf, RxBuf, Size);
	        MainBuf[Size] = '\0';  // Null terminate

	        // Clear RxBuf for next reception
	        memset(RxBuf, 0, RxBuf_SIZE);

	        // Restart DMA reception
	        HAL_UARTEx_ReceiveToIdle_DMA(huart3, (uint8_t *) RxBuf, RxBuf_SIZE);
	        __HAL_DMA_DISABLE_IT(hdma_usart3_rx, DMA_IT_HT);

	}
}


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//prevent unused argument(s)compilation warning
	UNUSED(huart);

	HAL_UART_Receive_IT(huart3,(uint8_t *)MainBuf,10,0xFFFF);
}*/

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//prevent unused argument(s)compilation warning
	UNUSED(huart);

	HAL_UART_Transmit(huart3,(uint8_t *)MainBuf,10,0xFFFF);
}*/


void UART_send_string(const uint8_t *p){
	//HAL_UART_Transmit(huart3,(uint8_t *)p, 30,0xFFFF); *
	HAL_UART_Transmit(huart3,(uint8_t *)p, strlen(p),0xFFFF);

}

void UART_send_char(char c){
	HAL_UART_Transmit(huart3,(uint8_t *)&c,1,0xFFFF);
}

//void Get_Buffer(uint8_t* buffer){
//
//	sprintf(buffer,"%s", RxBuf);
//	memset(RxBuf, 0, RxBuf_SIZE);
//	RxBuf[0] = '\0';
//	//Clear_Buffer();
//
//}


void Get_Buffer(uint8_t* buffer)
{
    sprintf((char*)buffer, "%s", MainBuf);
    memset(MainBuf, 0, MainBuf_SIZE);  // Clear MainBuf after reading
    MainBuf[0] = '\0';
}

void Clear_Buffer(){
	//clear the MainBuffer
	memset(MainBuf, 0, MainBuf_SIZE);
	MainBuf[0] = '\0';
}
