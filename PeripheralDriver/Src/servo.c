/*
 * servo.c
 *
 *  Created on: Aug 22, 2023
 *      Author: Xcoga
 */



#include <servo.h>
#include <math.h>

static TIM_HandleTypeDef* pwm_timer;

void Servo_Init(TIM_HandleTypeDef* htim)
{
	//set the pwm_timer to the htim we get from the main (which should be timer 1)
	pwm_timer = htim;
	//In our case, our servo motor is set to Timer 1 channel 1
	HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_2);
	//why are they assigning register an angle? Because u want it to go straight first.
	(*pwm_timer).Instance->CCR2 = CENTER_SERVO_DUTY;
}

void Servo_turnRight(){
	(*pwm_timer).Instance->CCR2 = RIGHT_SERVO_DUTY;
}

void Servo_turnLeft(){
	(*pwm_timer).Instance->CCR2 = LEFT_SERVO_DUTY;
}

void Servo_fixLeft(){
	(*pwm_timer).Instance->CCR2 = FIX_LEFT_SERVO_DUTY;
}

void Servo_fixRight(){
	(*pwm_timer).Instance->CCR2 = FIX_RIGHT_SERVO_DUTY;
}

void Servo_Centre(){
	(*pwm_timer).Instance->CCR2 = CENTER_SERVO_DUTY; //-10.9;
}

void Servo_mildRight(){
	(*pwm_timer).Instance->CCR2 = MILD_RIGHT;
}

//void Servo_mildCenterRight(){
//	(*pwm_timer).Instance->CCR1 = MILD_CENTER_RIGHT;
//}

void Servo_mildLeft(){
	(*pwm_timer).Instance->CCR2 = MILD_LEFT;
}

void Servo_R_turnRight(){
	(*pwm_timer).Instance->CCR2 = R_RIGHT_SERVO_DUTY;

}





