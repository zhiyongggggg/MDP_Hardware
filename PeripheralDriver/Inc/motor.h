/*
 * UART
 *
 *  Created on: Aug 30, 2023
 *      Author: DKER
 */

#ifndef INC_motor_H_
#define INC_motor_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"

void Motor_Init(TIM_HandleTypeDef* htiml, TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* encoderL_timer, TIM_HandleTypeDef* encoderR_timer);
void Motor_Stop();
void Motor_Forward(int PWM_L, int PWM_R);
void Motor_Reverse(int PWM_L, int PWM_R);
void Motor_LF_RB(int PWM_L, int PWM_R);
void Motor_RF_LB(int PWM_L, int PWM_R);

void Motor_RturnRight(int PWM_L, int PWM_R);
void Motor_turnRight(int PWM_L, int PWM_R);
void Get_Encoder();



#endif /* INC_motor_H_ */
