#include <motor.h>
static TIM_HandleTypeDef* pwmL_timer_s;
static TIM_HandleTypeDef* pwmR_timer_s;
static TIM_HandleTypeDef* encoderL_timer_s;
static TIM_HandleTypeDef* encoderR_timer_s;

#define MAX_MOTOR_DUTY 8400 // 50%
#define MIN_MOTOR_DUTY 220 // 2.4%
#define COUNTER_RESET_VALUE 2147483647

static uint32_t time_elapsed = 0x00;
static uint32_t prev_time_elapsed = 0x00;
static uint32_t time_difference = 0x00;

void Motor_Init(TIM_HandleTypeDef* htiml, TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* encoderL_timer, TIM_HandleTypeDef* encoderR_timer)
{
	pwmL_timer_s = htiml;
	pwmR_timer_s = htim2;
	encoderL_timer_s = encoderL_timer;
	encoderR_timer_s = encoderR_timer;
	HAL_TIM_PWM_Start(pwmL_timer_s, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwmL_timer_s, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(pwmR_timer_s, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwmR_timer_s, TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(encoderL_timer_s, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(encoderR_timer_s, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(encoderL_timer_s, COUNTER_RESET_VALUE);
	__HAL_TIM_SET_COUNTER(encoderR_timer_s, COUNTER_RESET_VALUE);
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,0);
}

void Motor_Stop()
{
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_4,0);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,0);
}

void Motor_Forward(int PWM_L, int PWM_R)
{
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_3,PWM_L);	//	IN2 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_4,0);	    //	IN1 = 0
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,PWM_R);	//	IN1 = PWM_R
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);		//	IN2 = 0
	(*pwmL_timer_s).Instance->CCR3 = PWM_L;
	(*pwmR_timer_s).Instance->CCR4 = PWM_R;
}

void Motor_LF_RB(int PWM_L, int PWM_R)
{
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set Motor A (Left Motor) to move forward
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_3,PWM_L);	//IN2 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_4,0);	    //IN1 = 0

	// Set Motor B (Right Motor) to move backward
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_4, 0); 	  // IN1 = 0
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_3, PWM_R); // IN2 = PWM_R
	(*pwmL_timer_s).Instance->CCR3 = PWM_L;
	(*pwmR_timer_s).Instance->CCR3 = PWM_R;
}

void Motor_RF_LB(int PWM_L, int PWM_R)
{
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set Motor A (Left Motor) to move backward
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_4, PWM_L);  // IN1 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_3, 0);      // IN2 = 0

	// Set Motor B (Right Motor) to move forward
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,PWM_R);	//	IN1 = PWM_R
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);		//	IN2 = 0
	(*pwmL_timer_s).Instance->CCR4 = PWM_L;
	(*pwmR_timer_s).Instance->CCR4 = PWM_R;
}

void Motor_Reverse(int PWM_L, int PWM_R)
{
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set PWM for Motor A (Left Motor)
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_4, PWM_L);  // IN1 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_3, 0);      // IN2 = 0
	// Set PWM for Motor B (Right Motor)
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_4, 0);  	  // IN1 = 0
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_3, PWM_R); // IN2 = PWM_R
	(*pwmL_timer_s).Instance->CCR4 = PWM_L;
	(*pwmR_timer_s).Instance->CCR3 = PWM_R;
}

void Motor_turnLeft(int PWM_L, int PWM_R){
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set Motor A (Left Motor) to move backward
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_4, PWM_L);  // IN1 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_3, 0);      // IN2 = 0

	// Set Motor B (Right Motor) to move forward
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,PWM_R);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);
	(*pwmL_timer_s).Instance->CCR4 = PWM_L;
	(*pwmR_timer_s).Instance->CCR4 = PWM_R;
}

void Motor_turnRight(int PWM_L, int PWM_R){
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set Motor A (Left Motor) to move forward
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_3,PWM_L);
	__HAL_TIM_SetCompare(pwmL_timer_s,TIM_CHANNEL_4,0);

	// Set Motor B (Right Motor) to move backward
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_4, 0); 	  // IN1 = 0
	__HAL_TIM_SetCompare(pwmR_timer_s, TIM_CHANNEL_3, PWM_R); // IN2 = PWM_R
	(*pwmL_timer_s).Instance->CCR3 = PWM_L;
	(*pwmR_timer_s).Instance->CCR3 = PWM_R;
}

void Motor_RturnRight(int PWM_L, int PWM_R){
	PWM_L = PWM_L > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_L < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_L;
	PWM_R = PWM_R > MAX_MOTOR_DUTY ? MAX_MOTOR_DUTY : PWM_R < MIN_MOTOR_DUTY ? MIN_MOTOR_DUTY : PWM_R;
	// Set Motor A (Left Motor) to move backward
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_4, PWM_L);  // IN1 = PWM_L
	__HAL_TIM_SetCompare(pwmL_timer_s, TIM_CHANNEL_3, 0);      // IN2 = 0

	// Set Motor B (Right Motor) to move forward
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_4,PWM_R);
	__HAL_TIM_SetCompare(pwmR_timer_s,TIM_CHANNEL_3,0);
	(*pwmL_timer_s).Instance->CCR4 = PWM_L;
	(*pwmR_timer_s).Instance->CCR4 = PWM_R;
}

void Get_Encoder(double* Data){
	uint32_t counter_l = __HAL_TIM_GET_COUNTER(encoderL_timer_s);
	uint32_t counter_r = __HAL_TIM_GET_COUNTER(encoderR_timer_s);
	int32_t dir_cnt_l, dir_cnt_r;
	time_elapsed = HAL_GetTick();
	//overflow
	time_difference = time_elapsed - prev_time_elapsed;
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoderL_timer_s))
	{
		dir_cnt_l = (int32_t)(COUNTER_RESET_VALUE - counter_l);
	}
	else
	{
		dir_cnt_l = -(int32_t)(counter_l - COUNTER_RESET_VALUE);
	}
	__HAL_TIM_SET_COUNTER(encoderL_timer_s, COUNTER_RESET_VALUE);

	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(encoderR_timer_s))
	{
		dir_cnt_r = -(int32_t)(COUNTER_RESET_VALUE - counter_r);
	}
	else
	{
		dir_cnt_r = (int32_t)(counter_r - COUNTER_RESET_VALUE);
	}
	__HAL_TIM_SET_COUNTER(encoderR_timer_s, COUNTER_RESET_VALUE);

	Data[0] = (double) dir_cnt_l / (double) time_difference * 1000.0 / 360.0 / 2.0;
	Data[1] = (double) dir_cnt_r / (double) time_difference * 1000.0 / 360.0 / 2.0;
	Data[4] = (Data[0]+Data[1])/2 * 13.816 * (double) time_difference / (double)1000.0;
	prev_time_elapsed = time_elapsed;

}
