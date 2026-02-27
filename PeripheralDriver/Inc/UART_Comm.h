/*
 * UART
 *
 *  Created on: Aug 30, 2023
 *      Author: DKER
 */

#ifndef INC_UART_Comm_H_
#define INC_UART_Comm_H_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

void UART_Comm_Init(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma_usart_rx);
//void UART_Comm_Init(UART_HandleTypeDef* huart);
void UART_send_string(const uint8_t *p);
void UART_send_char(char s);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void Get_Buffer();
void Clear_Buffer();
void Get_Buffer(uint8_t* buffer);  // Add parameter


#endif /* INC_UART_Comm_H_ */
