/*
 * HCSR04.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Xcoga
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_
#include "stm32f4xx.h"



void HCSR04_Init(TIM_HandleTypeDef* htim, uint32_t timer_channel, GPIO_TypeDef * GPIO_TrigPort, uint16_t GPIO_TrigPin, GPIO_TypeDef * GPIO_EchoPort, uint16_t GPIO_EchoPin);
void delay (uint16_t time);
void HCSR04_Read (void);
uint32_t HCSR04_getDistance(void);
uint32_t HCSR04_getDifference(void);


#endif /* INC_HCSR04_H_ */
