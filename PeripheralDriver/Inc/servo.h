/*
 * servo.h
 *
 *  Created on: Aug 22, 2023
 *      Author: Xcoga
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_


//Right is 144

//mid is 100
//#define MAX_SERVO_DUTY 195 // max to the right //160
//#define MIN_SERVO_DUTY 90 // max to the left //50

#define RIGHT_SERVO_DUTY 194  //24 degrees/s to right //218
#define LEFT_SERVO_DUTY 110 //100 //120
#define CENTER_SERVO_DUTY 147// 0 deg (To be adjusted once set) //147
#define R_RIGHT_SERVO_DUTY 197  //24 degrees/s to right //218


#define FIX_LEFT_SERVO_DUTY 140 //60
#define FIX_RIGHT_SERVO_DUTY 157 //152

#define MILD_RIGHT 148 //148
//#define MILD_CENTER_RIGHT 147 //110
#define MILD_LEFT 146 //146
//
//#define MAX_SERVO_DUTY_ANGLE -16
//#define MIN_SERVO_DUTY_ANGLE 21



#include "stm32f4xx_hal.h"

//-----------------Motor Control Functions----------------
void Servo_Init(TIM_HandleTypeDef* htim);
void Servo_turnLeft();
void Servo_turnRight();
void Servo_Centre();
void Servo_mildRight();
void Servo_mildLeft();
//void Servo_mildCenterRight();
void Servo_quiteLeft();

void Servo_fixLeft();
void Servo_fixRight();

void Servo_R_turnRight();
#endif /* INC_SERVO_H_ */

