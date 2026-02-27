/*
 * HCSR04.c
 *
 *  Created on: Aug 29, 2023
 *      Author: Xcoga
 */
#include "stdlib.h"
#include <HCSR04.h>


//define Ultrasound_Echo_Pin GPIO_PIN_14
//define Ultrasound_Echo_GPIO_Port GPIOD
//define Ultrasound_Trigger_Pin GPIO_PIN_15
//define Ultrasound_Trigger_GPIO_Port GPIOD


static TIM_HandleTypeDef* timer;
static GPIO_TypeDef * Ultrasound_Trigger_GPIO_Port;
static uint16_t Ultrasound_Trigger_Pin;
static GPIO_TypeDef * Ultrasound_Echo_GPIO_Port;
static uint16_t Ultrasound_Echo_Pin;
static uint32_t TIM_CHANNEL;


uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;



void HCSR04_Init(TIM_HandleTypeDef* htim, uint32_t timer_channel, GPIO_TypeDef * GPIO_TrigPort, uint16_t GPIO_TrigPin,
		GPIO_TypeDef * GPIO_EchoPort, uint16_t GPIO_EchoPin )
{
	timer = htim;
	TIM_CHANNEL = timer_channel;
	HAL_TIM_IC_Start_IT(timer, TIM_CHANNEL);
	Ultrasound_Trigger_GPIO_Port = GPIO_TrigPort;
	Ultrasound_Trigger_Pin = GPIO_TrigPin;
	Ultrasound_Echo_GPIO_Port = GPIO_EchoPort;
	Ultrasound_Echo_Pin = GPIO_EchoPin;
}

//Delay function to create microseconds delay for Ultrasound echo
void delay (uint16_t time){

	__HAL_TIM_SET_COUNTER(timer,0);
	while (__HAL_TIM_GET_COUNTER(timer)< time);

}




// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//TODO Please change the below if statement if we change the channel!
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				//Do note that difference is in microseconds as per our clock configuration fo 160MHz/160 = 1 Mhz
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			//Distance formula is used as per sensor's datasheet
			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false


			//





			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(timer, TIM_IT_CC2);
		}
	}

}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(Ultrasound_Trigger_GPIO_Port, Ultrasound_Trigger_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us as per ultrasound's datasheet
	HAL_GPIO_WritePin(Ultrasound_Trigger_GPIO_Port, Ultrasound_Trigger_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(timer, TIM_IT_CC2);
}

uint32_t HCSR04_getDistance(void){
	return Distance;
}

uint32_t HCSR04_getDifference(void){
	return Difference;
}
