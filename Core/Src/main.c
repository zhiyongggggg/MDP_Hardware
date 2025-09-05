/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <icm20948.h>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "oled.h"
#include "HCSR04.h"
#include "UART_Comm.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static double PWM_L = 0.0, PWM_R = 0.0;
static double P_L = 0.0f, I_L = 0.0, D_L = 0.0, P_R = 0.0, I_R = 0.0, D_R = 0.0;
static double Encoder_Data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
static int16_t IMU_Data[] = {0, 0, 0, 0, 0, 0, 0};
static int imu_init_debugValue = -1;
static int imu_read_debugValue = -1;
float corrected_gyroVal = 0;
char command_buffer[30] = "stop";
char Task_buffer[30] = "stop";
char temp_buffer[30] = "stop";
char movement_finish[30] = "movement finished!";
double orientation = 0;
double x_pos = 0;
double y_pos = 0;
int checkSemaphore = 0;
static int uart_call_counter = 0;
static int uart_acquire_counter = 0;
static int buffer_data_counter = 0;



//for Motor
char right[10] = "right";
char left[10] = "left";
char forward[10] = "forward";
char reverse[10] = "reverse";
char stop[10] = "stop";
char reverse_left[10] = "r_left";
char reverse_right[10] = "r_right";

char motor_command[40] = "stop";


int obstacle_len  = 0;

//for ServoMotor
//commands format #TurnDirection # TurnAngle #Movement Direction #vertical displacement
//uint8_t dummy_command_forward[30] = "center,0,forward,50";
//uint8_t dummy_command_reverse[30] = "center,0,reverse,50";
//uint8_t dummy_command_left[30] = "left,90,forward,50";
//uint8_t dummy_command_right[30] = "right,90,forward,50";
//uint8_t dummy_command_right[30] = "right,90,reverse,50";
//uint8_t dummy_command_right[30] = "left,90,reverse,50";


//Stop moving.
char dummy_command_stop[20] = "stop";

//commands format for Spot Turns
//uint8_t dummy_command_left_spot[30] = "spot,90,left";
//uint8_t dummy_command_right_spot[30] = "spot,90,right";


/*
 *indoor values
rf 61,62,62
lf 61,61,64,61
rr 58,58,60
lr 56,58,57
 *
 */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for LED_Blink */
osThreadId_t LED_BlinkHandle;
const osThreadAttr_t LED_Blink_attributes = {
  .name = "LED_Blink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Show */
osThreadId_t ShowHandle;
const osThreadAttr_t Show_attributes = {
  .name = "Show",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoMotor */
osThreadId_t ServoMotorHandle;
const osThreadAttr_t ServoMotor_attributes = {
  .name = "ServoMotor",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UltrasoundRead */
osThreadId_t UltrasoundReadHandle;
const osThreadAttr_t UltrasoundRead_attributes = {
  .name = "UltrasoundRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Comm */
osThreadId_t UART_CommHandle;
const osThreadAttr_t UART_Comm_attributes = {
  .name = "UART_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CaculatePositio */
osThreadId_t CaculatePositioHandle;
const osThreadAttr_t CaculatePositio_attributes = {
  .name = "CaculatePositio",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for orientationCal1 */
osThreadId_t orientationCal1Handle;
const osThreadAttr_t orientationCal1_attributes = {
  .name = "orientationCal1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InfraredTask */
osThreadId_t InfraredTaskHandle;
const osThreadAttr_t InfraredTask_attributes = {
  .name = "InfraredTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for EncoderSemaphore */
osSemaphoreId_t EncoderSemaphoreHandle;
const osSemaphoreAttr_t EncoderSemaphore_attributes = {
  .name = "EncoderSemaphore"
};
/* Definitions for IMUSemaphore */
osSemaphoreId_t IMUSemaphoreHandle;
const osSemaphoreAttr_t IMUSemaphore_attributes = {
  .name = "IMUSemaphore"
};
/* Definitions for C_BufferSemaphore */
osSemaphoreId_t C_BufferSemaphoreHandle;
const osSemaphoreAttr_t C_BufferSemaphore_attributes = {
  .name = "C_BufferSemaphore"
};
/* USER CODE BEGIN PV */
uint32_t distanceBuffer;
uint32_t calibrationBuffer = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
void BlinkLED(void *argument);
void show(void *argument);
void servo_motor(void *argument);
void motor(void *argument);
void UltrasoundRea(void *argument);
void UART_comm_task(void *argument);
void Caculate_Pos(void *argument);
void orientationCal(void *argument);
void MovementTask(void *argument);
void IRread(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IR_buffer[2]; //to store the IR values for both IR
uint32_t left_IR = 0;
uint32_t right_IR = 0;
uint8_t adc_conv_complete_flag = 0;

uint32_t Adc_0_Data = 0;
uint32_t Adc_1_Data = 0;
uint32_t Channel_0_Volt = 0;
uint32_t Channel_1_Volt = 0;

//uint32_t d[2];
uint32_t d1 = 0;
uint32_t d2 = 0;


//int ADC_Read_Distance_Right(void){
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1,100);
//		right_IR = HAL_ADC_GetValue(&hadc1);
//		HAL_ADC_Stop(&hadc1);
//
//		return (float)(right_IR/4095)*3.3;
//}
//
//int ADC_Read_Distance_Left(void){
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1,100);
//	left_IR = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);
//
//	return ((float)left_IR/4095)*3.3 ;
//}
//
//uint16_t ADC_Read_Channel(uint32_t channel)
//{
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  // Configure the channel to read
//  sConfig.Channel = channel;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  // Start the ADC conversion
//  if (HAL_ADC_Start(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  // Poll for conversion completion
//  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  // Get the ADC value
//  uint16_t adc_value = HAL_ADC_GetValue(&hadc1);
//
//  // Stop the ADC conversion
//  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  //return ((float)adc_value/4095)*3.3;
//  return adc_value;
//}

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	//interrupt mode
	right_IR = HAL_ADC_GetValue(&hadc1);

	//Start ADC again
	HAL_ADC_Start_IT(&hadc1);


}*/
int16_t getRequestedAngle(char *p) {
    char* token;
    char* saveptr;
    int16_t angle = 0;
    //field index is index of angle within the command
    uint8_t field_index = 1;
    char temp_string[30];

    sprintf(temp_string,"%s",p);

    // Tokenize the command using ',' as the delimiter
    token = strtok_r(temp_string, ",", &saveptr);

    //skip the tokens that is before Displacement's field index
    for (int i=0;i<field_index;i++){
        token = strtok_r(NULL, ",", &saveptr);
    }

    // retrieve the angle value
    angle = atoi(token);
    return angle;

}

int16_t getRequestedVerticalDisplacement(char *p){

    char* token;
    char* saveptr;
    int16_t verticalDisplacement = 0;
    uint8_t field_index = 3;
    char temp_string[30];

    sprintf(temp_string,"%s",p);

    // Tokenize the command using ',' as the delimiter
    token = strtok_r(temp_string, ",", &saveptr);

    // Iterate through the tokens to find the last field
    //last field is at index 3
    for (int i=0;i<field_index;i++){
        token = strtok_r(NULL, ",", &saveptr);
    }

    verticalDisplacement = atoi(token);

    return verticalDisplacement;

}

uint8_t getRequestedMultiplier(char *p){
    char* token;
    char* saveptr;
    char temp_string[30];
    uint8_t Multiplier = 0;
    uint8_t field_index = 1;

    sprintf(temp_string,"%s",p);

    // Tokenize the command using ',' as the delimiter
    token = strtok_r(temp_string, ",", &saveptr);

    // Iterate through the tokens to find the last field
    //last field is at index 3
    for (int i=0;i<field_index;i++){
        token = strtok_r(NULL, ",", &saveptr);
    }

    Multiplier = atoi(token);

    return Multiplier;
}

double euclideanDis(double x1, double x2, double y1, double y2){
	return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

void sendPosOrienData(){
    char pos_orien_data[30] = "\0";

    sprintf(pos_orien_data, "ACK %d,%d,%d,%d\0", (int)orientation, (int)x_pos, (int)y_pos, (int)distanceBuffer);

    char newline[] = "\n";
    UART_send_string((uint8_t*)strcat(pos_orien_data, newline));
}

void askForDirection(){
	char takePicture[5] = "TP";

	UART_send_string((uint8_t*)takePicture);
}

void sendDistanceFromObstacle(){
	char distance_from_obstacle[30] = "\0"; // Initialize with an empty string.

    sprintf(distance_from_obstacle, "%d\0", (int)distanceBuffer);
	UART_send_string((uint8_t*)distance_from_obstacle);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  UART_Comm_Init(&huart3, &hdma_usart3_rx);
  //UART_Comm_Init(&huart3);
//  HCSR04_Init(&htim4, TIM_CHANNEL_3, Ultrasound_Trigger_GPIO_Port, Ultrasound_Trigger_Pin ,Ultrasound_Echo_GPIO_Port,Ultrasound_Echo_Pin );
  Motor_Init(&htim4, &htim1, &htim2, &htim5);
  //HAL_ADC_Start_DMA(&hadc1,&IR_buffer,2); //start in DMA mode and we are reading only 2 channel or 2 word
  //HAL_ADC_Start_IT(&hadc1);

  HAL_Delay(500);
  char hello_msg[] = "Starting...";
  OLED_Clear();
  OLED_ShowString(10, 10, (uint8_t*)hello_msg);
  OLED_Refresh_Gram();
  HAL_Delay(1000);

  // Initialize IMU (store result for tasks to use)
  imu_init_debugValue = IMU_Init(&hi2c2);

  Servo_Init(&htim12);
//
//  HAL_Delay(500);
//  char hello_msg[] = "hello";
//  OLED_Clear();
//  OLED_ShowString(10, 10, (uint8_t*)hello_msg);
//  OLED_Refresh_Gram();
//  HAL_Delay(2000);
//
//  // Initialize IMU
//  int imu_init_result = IMU_Init(&hi2c2);
//
//  // Display init result
//  char init_msg[20];
//  sprintf(init_msg, "IMU Init: %d", imu_init_result);
//  OLED_Clear();
//  OLED_ShowString(10, 10, (uint8_t*)init_msg);
//  OLED_Refresh_Gram();
//  HAL_Delay(2000);
//
//  // Test IMU reading
//  int16_t IMU_Data[7] = {0};
//  char imu_msg[50];
//


//  while (1)
//  {
//      int read_result = IMU_Read(IMU_Data);
//
//      OLED_Clear();
//      sprintf(imu_msg, "Read: %d", read_result);
//      OLED_ShowString(10, 10, (uint8_t*)imu_msg);
//
//      // Display gyro Z (used for orientation)
//      sprintf(imu_msg, "GyroZ: %d", IMU_Data[6]);
//      OLED_ShowString(10, 20, (uint8_t*)imu_msg);
//
//      // Display accel data (if working)
//      sprintf(imu_msg, "AccX: %d", IMU_Data[0]);
//      OLED_ShowString(10, 30, (uint8_t*)imu_msg);
//
//      OLED_Refresh_Gram();
//      HAL_Delay(200);
//  }

  /* USER CODE END 2 */

  /* Init scheduler */

  OLED_Clear();
  OLED_ShowString(10, 10, (uint8_t*)"OS Init...");
  OLED_Refresh_Gram();
  osKernelInitialize();

  OLED_Clear();
  OLED_ShowString(10, 10, (uint8_t*)"Creating tasks");
  OLED_Refresh_Gram();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of EncoderSemaphore */
  EncoderSemaphoreHandle = osSemaphoreNew(1, 1, &EncoderSemaphore_attributes);

  /* creation of IMUSemaphore */
  IMUSemaphoreHandle = osSemaphoreNew(1, 0, &IMUSemaphore_attributes);

  /* creation of C_BufferSemaphore */
  C_BufferSemaphoreHandle = osSemaphoreNew(1, 1, &C_BufferSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LED_Blink */
 // LED_BlinkHandle = osThreadNew(BlinkLED, NULL, &LED_Blink_attributes);

  /* creation of Show */
  ShowHandle = osThreadNew(show, NULL, &Show_attributes);

  /* creation of ServoMotor */
  ServoMotorHandle = osThreadNew(servo_motor, NULL, &ServoMotor_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of UltrasoundRead */
  //UltrasoundReadHandle = osThreadNew(UltrasoundRea, NULL, &UltrasoundRead_attributes);

  /* creation of UART_Comm */
  UART_CommHandle = osThreadNew(UART_comm_task, NULL, &UART_Comm_attributes);

  /* creation of CaculatePositio */
  CaculatePositioHandle = osThreadNew(Caculate_Pos, NULL, &CaculatePositio_attributes);

  /* creation of orientationCal1 */
  orientationCal1Handle = osThreadNew(orientationCal, NULL, &orientationCal1_attributes);

  /* creation of Task2 */
 // Task2Handle = osThreadNew(MovementTask, NULL, &Task2_attributes);

  /* creation of InfraredTask */
 // InfraredTaskHandle = osThreadNew(IRread, NULL, &InfraredTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  OLED_Clear();
  OLED_ShowString(10, 10, (uint8_t*)"Starting OS");
  OLED_Refresh_Gram();
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	//  Motor_Forward(4000, 5000);
    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 7199;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 160;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 1000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DC_Pin OLED_RST_Pin OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_BlinkLED */
/**
  * @brief  Function implementing the LED_Blink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BlinkLED */
void BlinkLED(void *argument)
{
  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
////uint8_t ch  = 'A';
//  for(;;)
//  {
//	  //We are transmitting the characters to raspberry pi
//	  //HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//	  //HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//	  /*if (ch<'Z')
//	  	  ch++;
//	  else ch = 'A';*/
//	  HAL_GPIO_TogglePin(LED_Blinking_GPIO_Port, LED_Blinking_Pin);
//	  osDelay(2000);
//  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the Show thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
//void show(void *argument)
//{
  /* USER CODE BEGIN show */

//	char gyroValStr[30];
//
//  /* Infinite loop */
//
//  for(;;)
//  {
//	  //To squeeze as much as possible on the OLED screen, we can set x as 10, y as 10,20,30,40,50
//	  OLED_Clear();
//	  //sprintf(gyroValStr,"%d,%d\0", ADC_Read_Distance_Left(),ADC_Read_Distance_Right());
//	  sprintf(gyroValStr, "%d\0", (int) distanceBuffer);
//	 OLED_ShowString(10,10,(uint8_t*)gyroValStr);
////	 sprintf(gyroValStr, "IR_left: %d\0", ADC_Read_Channel(11)); //ADC_Read_Distance_Left()*
//	 OLED_ShowString(10,20,(uint8_t*)gyroValStr);
//	 //OLED_ShowString(10,20,(uint8_t*)command_buffer);
//	  //sprintf(gyroValStr, "%d,%d\0", (int) x_pos,(int)y_pos);
////	  sprintf(gyroValStr, "IR_right: %d\0", ADC_Read_Channel(12)); //ADC_Read_Distance_Right()*
//	  OLED_ShowString(10,30,(uint8_t*)gyroValStr);
//	  sprintf(gyroValStr, "%d", (int) orientation);
//	  OLED_ShowString(10, 40, (uint8_t*)gyroValStr);
//	  sprintf(gyroValStr, "%s", motor_command);
//	  OLED_ShowString(10, 50, (uint8_t*)gyroValStr);
//
//
//	//refresh the ram in the OLED so taht the OLED display can be updated
//	OLED_Refresh_Gram();
//    osDelay(100);
//  }
//}

void show(void *argument)
{
    char displayStr[30];

    for(;;)
    {
        OLED_Clear();

        // Display current command
        sprintf(displayStr, "Cmd: %.12s", command_buffer);
        OLED_ShowString(10, 10, (uint8_t*)displayStr);

        // Display position
        sprintf(displayStr, "X:%d Y:%d", (int)x_pos, (int)y_pos);
        OLED_ShowString(10, 20, (uint8_t*)displayStr);

        // Display orientation
        sprintf(displayStr, "Orient: %d", (int)orientation);
        OLED_ShowString(10, 30, (uint8_t*)displayStr);

        OLED_Refresh_Gram();
        osDelay(100);
    }
}

/* USER CODE BEGIN Header_servo_motor */
/**
* @brief Function implementing the ServoMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_motor */
void servo_motor(void *argument)
{
  /* USER CODE BEGIN servo_motor */
  Servo_Init(&htim12);

  //for servo

  double target_Orientation = 0;
  double reverse_target_Orientation = 0;
  double initial_Orientation = 0;
  double requested_angle = 0;
  uint8_t delay_time = 10;
  //for 100 delay_time, use 12 compensation_delay
  //for 200 delay_time, use 15 compensation_delay
  //for 40 delay_time, use 5 compensation_delay
  //TODO TUNE THESE PARAMETERS ACCORDING TO OUR NEEDS
  double compensation_delay = 35;
  //right turn revers comp set to 15 (for delay of 200)
  //left side reverse comp set to 15 (for delay of 200)
  double reverse_compensation_delay = 15;
  int16_t offset_from_center;
  int16_t requested_vertical_displacement = 0;
  int multiplier = 100;
  int offset = 0;
  int distance = 0;
  int IR_value = 0;
  int body_len_car_exclude_IR = 16;
  int x_pos_obstacle_1= 0 ;
  double temp_orientation = 0;

  double initial_x_pos;
  double initial_y_pos;

  double starting_x_pos = 0;
  double starting_y_pos = 0;
  int flag = 0;
  double flag_y_pos = 0;

  Servo_Centre(); //center
  osDelay(5000);

   //Infinite loop
  for(;;)
  {

	  multiplier = 100;

	  //Make sure command_buffer is not changed by UART while code is running
	  if ((strncmp("stop", (char*)command_buffer, 4) == 0)){
		  Servo_Centre();
		  sprintf(motor_command, "%s", stop);
		  continue;
	  }



	  if (osSemaphoreAcquire(C_BufferSemaphoreHandle, 100)!=0){
		  continue;
	  }


	  if ((strncmp("stop", (char*)command_buffer, 4) == 0)){
		  Servo_Centre();
		  sprintf(motor_command, "%s", stop);
		  osDelay(500);

	  }

	  // SPOT TURN LEFT
	  else if((strncmp("spot", (char*)command_buffer, 4) == 0) && (strstr((char *)command_buffer, "left") != NULL)){

		  requested_angle = getRequestedAngle(command_buffer);
          target_Orientation = orientation + requested_angle;
          reverse_target_Orientation = orientation + requested_angle/2;
          multiplier = 70;


          sprintf(motor_command, "%s", stop);
          osDelay(100);
		  Servo_turnRight();
		  osDelay(200);
		  sprintf(motor_command, "%s,%d", reverse_left, multiplier);



          while(orientation < reverse_target_Orientation){
        	  osDelay(delay_time);

          }
          sprintf(motor_command,"%s",stop);
          osDelay(100);
          Servo_turnLeft();
          osDelay(200);
          sprintf(motor_command, "%s,%d", left, multiplier);

          while (orientation < target_Orientation){
        	  osDelay(delay_time);
          }
          sprintf(motor_command,"%s",stop);

          osDelay(200);
          Servo_Centre();
          osDelay(200);
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",dummy_command_stop);
		  UART_send_string(movement_finish);
		  sendPosOrienData();
		  osDelay(200);
	  }

	  else if ((strncmp("spot", (char*)command_buffer, 4) == 0) && (strstr((char *)command_buffer, "right") != NULL)){
		  requested_angle = -getRequestedAngle(command_buffer);
          target_Orientation = orientation + requested_angle;
          reverse_target_Orientation = orientation + requested_angle/2;
          multiplier = 70;

          sprintf(motor_command, "%s", stop);
          osDelay(100);
		  Servo_turnLeft();
		  osDelay(200);
		  sprintf(motor_command, "%s,%d", reverse_right, multiplier);


          while(orientation > reverse_target_Orientation){
        	  osDelay(delay_time);
          }
          sprintf(motor_command,"%s",stop);
          osDelay(100);
          Servo_turnRight();
          osDelay(200);
          sprintf(motor_command, "%s,%d", right, multiplier);

          while (orientation > target_Orientation){
        	  osDelay(delay_time);
          }
          sprintf(motor_command,"%s",stop);

          osDelay(200);
          Servo_Centre();
          osDelay(200);
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",dummy_command_stop);
		  UART_send_string(movement_finish);
		  sendPosOrienData();
		  osDelay(200);

	  }

	  //Forward Right turn
	  else if ((strncmp("right", (char*)command_buffer, 5) == 0) && (strstr((char*)command_buffer, "forward") != NULL)){

            requested_angle = -getRequestedAngle(command_buffer);
            offset = -2.98;
            multiplier  = 200;

			target_Orientation = (orientation + requested_angle + offset);
			sprintf(motor_command, "%s", stop);
			Servo_turnRight();
			osDelay(10);

			sprintf(motor_command, "%s,%d", right, multiplier);

			while (orientation > target_Orientation ){ //>*
				if (orientation < target_Orientation + 20){
					multiplier = 20; //40*
					sprintf(motor_command, "%s,%d", right, multiplier);
				}
				osDelay(3);
			}
			Servo_fixLeft();
			sprintf(motor_command,"%s",stop);
			sprintf(command_buffer,"%s",dummy_command_stop);

			osDelay(200);
			sendPosOrienData();
			osDelay(100);
	  }

	  else if ((strncmp("left", (char*)command_buffer, 4) == 0) && (strstr((char*)command_buffer, "forward") != NULL)){

		    requested_angle = getRequestedAngle(command_buffer);
		    multiplier  = 200;
		    offset = 0.08333;
			target_Orientation = orientation + requested_angle + offset;

			sprintf(motor_command, "%s", stop);
			Servo_turnLeft();
			osDelay(10);
			sprintf(motor_command, "%s,%d", left, multiplier);

			while (orientation < target_Orientation){
				//this delay is necessary or there will be scheduling issues
				if (orientation > target_Orientation +35 ){ //-20* //+40 //+35
					//add a multiplier. 20  gives exact angle. 30 to be faster
					multiplier = 20; //40* 20**
					sprintf(motor_command, "%s,%d", left, multiplier);
				}
				osDelay(3);
			}

			Servo_fixRight();
			sprintf(motor_command, "%s", stop);
			sprintf(command_buffer,"%s",dummy_command_stop);
			osDelay(200);
			sendPosOrienData();
			osDelay(100);

	  }

	  //Reverse right Turn
	  else if((strncmp("right", (char*)command_buffer, 5) == 0) && (strstr((char*)command_buffer, "reverse") != NULL)){
		  requested_angle = getRequestedAngle(command_buffer);
		  Servo_Centre();

		  offset = 0.58333;
		  //
		  target_Orientation = orientation + requested_angle + offset;

		  sprintf(motor_command, "%s", stop);
		  Servo_turnRight();
		  sprintf(motor_command, "%s,%d", reverse_right, multiplier);

		  while (orientation < target_Orientation){ //<*
				if (orientation > target_Orientation + 40){ //- 20*
					multiplier = 20; //30*
					sprintf(motor_command, "%s,%d", reverse_right, multiplier);
				}
			  osDelay(3);
		  }
		  Servo_fixLeft();
		  sprintf(motor_command, "%s", stop);

		  sprintf(command_buffer,"%s",dummy_command_stop);
		  osDelay(200);
		  sendPosOrienData();
		  osDelay(100);

	  }

	  //Reverse left Turn
	  else if((strncmp("left", (char*)command_buffer, 4) == 0) && (strstr((char*)command_buffer, "reverse") != NULL)){
		  requested_angle = -getRequestedAngle(command_buffer);
		  offset = -3.3333333;
		  Servo_Centre();

		  target_Orientation = orientation + requested_angle + offset;

		  sprintf(motor_command, "%s", stop);
		  Servo_turnLeft();

		  sprintf(motor_command, "%s,%d", reverse_left, multiplier);

		  while (orientation > target_Orientation){
				if (orientation < target_Orientation + 20){
					multiplier = 20;
					sprintf(motor_command, "%s,%d", reverse_left, multiplier);
				}
			  osDelay(3);
		  }
		  Servo_fixRight();
		  sprintf(motor_command, "%s", stop);

		  sprintf(command_buffer,"%s",dummy_command_stop);
		  osDelay(200);
		  sendPosOrienData();
		  osDelay(100);

	  }

	  else if((strncmp("center", (char*)command_buffer, 6) == 0) && (strstr((char *)command_buffer, "forward") != NULL)){

		  char right[30] = "right";
		  char left[30] = "left";
		  char offset_str[20];
		  int16_t center = 0;
		  requested_angle = 0;
		  target_Orientation = orientation + requested_angle;
		  requested_vertical_displacement = getRequestedVerticalDisplacement(command_buffer);
		  target_Orientation = orientation + requested_angle +  0.65/100*requested_vertical_displacement;

		  if(requested_vertical_displacement>50){
			  requested_vertical_displacement = requested_vertical_displacement-1;
		  }else{
			  requested_vertical_displacement = requested_vertical_displacement-1;
		  }
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  if (requested_vertical_displacement<=10) Servo_Centre();
		  else Servo_Centre();
		  sprintf(motor_command, "%s,%d", forward, multiplier);

		  center = orientation - target_Orientation;
		  while (euclideanDis(x_pos, initial_x_pos, y_pos,initial_y_pos) < requested_vertical_displacement){
				if (offset_from_center == 0){
					Servo_Centre();
				}
				offset_from_center = orientation - target_Orientation;

				if (offset_from_center > center){
					center++;
					Servo_mildRight();
					offset_from_center = 0;
				}
				if (offset_from_center < center){
					center--;
					Servo_mildLeft();
					offset_from_center = 0;
				}
				if (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) > requested_vertical_displacement - 20){
					multiplier = 40; //40*
					sprintf(motor_command, "%s,%d", forward, multiplier);
				}
				osDelay(6);
			}
			  sprintf(motor_command, "%s", stop);
			  sprintf(command_buffer,"%s",stop);
		  Servo_Centre();
		  osDelay(200);
		  sendPosOrienData();
		  osDelay(100);
	  }
	  else if((strncmp("center", (char*)command_buffer, 6) == 0) && (strstr((char *)command_buffer, "reverse") != NULL)){
		  //IF COMMAND IS GO STRAIGHT
		  Servo_Centre();
		  int16_t center = 0;
		  requested_angle = 0;
		  requested_vertical_displacement = getRequestedVerticalDisplacement(command_buffer);
		  target_Orientation = orientation + requested_angle + (-0.05)/100*requested_vertical_displacement;
		  if(requested_vertical_displacement>50){
			  requested_vertical_displacement = requested_vertical_displacement-2.5;
		  }
		  else{
			  requested_vertical_displacement = requested_vertical_displacement -0.40;
		  }
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  if (requested_vertical_displacement<=10)  Servo_Centre();
		  else Servo_Centre();
		  sprintf(motor_command, "%s,%d", reverse, multiplier);
		  center = orientation - target_Orientation;
		  while (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
				offset_from_center = orientation - target_Orientation;
				if (offset_from_center == 0){
					Servo_Centre();
				}
				offset_from_center = orientation - target_Orientation;
				if (offset_from_center > 0.05){
					center++;
					Servo_mildRight();
					offset_from_center = 0;
				}


				//means robot leaning to the right
				if (offset_from_center < -0.05){
					center--;
					Servo_mildLeft();
					offset_from_center = 0;
				}
				if (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) > requested_vertical_displacement - 20){
					//add a multiplier
					multiplier = 40;
					sprintf(motor_command, "%s,%d", reverse, multiplier);
				}
				osDelay(6);
			}
		  	  Servo_Centre();
			  sprintf(motor_command, "%s", stop);
			  sprintf(command_buffer,"%s",stop);

		  osDelay(100);
		  sendPosOrienData();
		  osDelay(100);

	  }
	  if (strncmp("0", (char*)command_buffer, 1) == 0){
		  //Below is original code
		  char word[5] = "ACK";
		  char newline[] = "\n";
		  starting_x_pos = x_pos;
		  starting_y_pos = y_pos;

		  initial_Orientation = orientation;
		  distance = 0;
		  int no_counts = 0;
		  multiplier = 200;
		  Servo_Centre();
		  osDelay(200);
		  d1 = 0;
		  d2 = 0;

		  static int i = 0;

		  while(no_counts < 5){
			  distance += distanceBuffer;
			  no_counts+=1;
			  osDelay(100);
		  }

		  distance = distance/5;

		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  char d[10] = "\0";


		  d1 = distance;
		  if (distance > 35){
			  requested_vertical_displacement = distance - 35;
			  multiplier = 100;
			sprintf(motor_command, "%s,%d", forward, multiplier);
			while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
				osDelay(3);
				if (orientation > initial_Orientation + 0.2){
					Servo_mildRight();
				}
				else if (orientation < initial_Orientation - 0.5){
					Servo_mildLeft();
				}
			}


			sprintf(motor_command, "%s", stop);
			sprintf(command_buffer,"%s",stop);
			no_counts = 0;
			while (no_counts<5){
				  distance += distanceBuffer;
				  no_counts+=1;
				  osDelay(100);
			}
			distance = distance/5;
			x_pos_obstacle_1 = x_pos + distance;
			d2 = distanceBuffer;
			sprintf(d, "ACK %d", (int)(d1-d2));
			UART_send_string((uint8_t*)strcat (d,newline));

		  }
		  else{

			  osDelay(500);
			  osDelay(500);
			  sprintf(command_buffer,"%s",stop);
			  sprintf(motor_command,"%s",stop);
			  d2 = distanceBuffer;
			  sprintf(d, "ACK %d", (int)(30));
			  UART_send_string((uint8_t*)strcat (d,newline));
		  }

		  if(distance <= 30 && distance > 25){
			sprintf(motor_command, "%s,%d", reverse, multiplier);
			osDelay(200);
			sprintf(command_buffer,"%s",stop);
			sprintf(motor_command,"%s",stop);
		  }
		  if(distance <= 25 && distance > 20){
			sprintf(motor_command, "%s,%d", reverse, multiplier);
			osDelay(300);
			sprintf(command_buffer,"%s",stop);
			sprintf(motor_command,"%s",stop);
		  }
	  }
	  //From obstacle 1 - Go to the left of obstacle 1
	  else if (strncmp("1,1", (char*)command_buffer, 3) == 0){
		  offset = 0;
		  requested_angle = 20; //40*
		  multiplier = 200;
		  delay_time = 200;
		  Servo_turnLeft();
		  sprintf(motor_command, "%s,%d", left, multiplier);
		  osDelay(700);
		  Servo_Centre();
		  osDelay(500);
		  multiplier=200;
		  Servo_turnRight();
		  sprintf(motor_command,"%s,%d",right,multiplier);
		  osDelay(1300);
		  Servo_Centre();
		  osDelay(500);
		  Servo_turnLeft();
		  sprintf(motor_command,"%s,%d",left,multiplier);
		  osDelay(300);
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
	  }
	  //Obstacle 1 ask to turn right
	  else if (strncmp("1,2", (char*)command_buffer, 3) == 0){
		  offset = 0;
		  requested_angle = -30;
		  multiplier = 200;
		  delay_time = 200;
		  Servo_turnRight();
		  multiplier = 150;
		  sprintf(motor_command, "%s,%d", right, multiplier);
		  while(orientation > initial_Orientation + requested_angle){
			  osDelay(3);
		  }
		  multiplier = 75;
		  Servo_Centre();
		  sprintf(motor_command,"%s,%d",forward,multiplier);
		  requested_vertical_displacement = 50;
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  temp_orientation = initial_Orientation;
		  while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
			  osDelay(3);
			  if (orientation > temp_orientation + 0.5){
				  Servo_mildRight();
			  }
			  else if (orientation < temp_orientation - 0.5){
				  Servo_mildLeft();
			  }
		  }
		  //reverse left
		  multiplier = 100;
		  Servo_turnRight();
		  sprintf(motor_command, "%s,%d", reverse_left, multiplier);
		  while(orientation < initial_Orientation){
		  	  osDelay(3);
		  }
		  Servo_Centre();
		  sprintf(motor_command, "%s,%d", forward, multiplier);
		  requested_vertical_displacement = 6;
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  temp_orientation = initial_Orientation;
		  while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
			  osDelay(3);
			  if (orientation > temp_orientation + 0.5){
				  Servo_mildRight();
			  }
			  else if (orientation < temp_orientation - 0.5){
				  Servo_mildLeft();
			  }
		  }
		  //We want to turn left
		  multiplier = 100;
		  requested_angle = 40;
		  Servo_turnLeft();
		  target_Orientation = initial_Orientation + requested_angle + offset;

		  sprintf(motor_command,"%s,%d", left, multiplier);
		  while(orientation < target_Orientation){
			  osDelay(3);
		  }
		  //Turn back right to set orientation to 0 degrees.
		  Servo_turnRight();
		  multiplier = 100;
		  sprintf(motor_command,"%s,%d", right, multiplier);
		  while(orientation > initial_Orientation){
			  osDelay(3);
		  }
		  Servo_Centre();
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  int no_counts = 0;
		  distance = 0;
		  while (no_counts <5){
			  distance += distanceBuffer;
			  no_counts++;
			  osDelay(100);
		  }
		  distance = distance/5;
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  if (distance > 30){
			  requested_vertical_displacement = distance - 30;
			  Servo_Centre();
			  sprintf(motor_command, "%s,%d",forward, multiplier);
			  temp_orientation = initial_Orientation;
			  while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement ){
				  osDelay(3);
				  if (orientation > temp_orientation + 0.5){
					  Servo_mildRight();
				  }
				  else if (orientation < temp_orientation - 0.5){
					  Servo_mildLeft();
				  }
			  }
		  }
		  else if (distance < 30){
			  requested_vertical_displacement = 30 - distance;
			  Servo_Centre();
			  sprintf(motor_command, "%s,%d",reverse, multiplier);
			  while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement ){
				  osDelay(3);
				  if (orientation > temp_orientation + 0.5){
					  Servo_mildLeft();
				  }
				  else if (orientation < temp_orientation - 0.5){
					  Servo_mildRight();
				  }
			  }
		  }
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  Servo_Centre();
		  UART_send_char('t');
		  osDelay(delay_time);
	  }
	  //if the RPI is unable to capture image, they send this command for us to inch forward.
	  else if (strncmp("1,3", (char*)command_buffer, 3) == 0){
		  int no_counts = 0;
		  multiplier = 100;
		  delay_time = 200;
		  distance = 0;
		  Servo_Centre();
		  multiplier = 75;
		  while(no_counts<5){
			  distance += distanceBuffer;
			  no_counts++;
			  osDelay(100);
		  }
		  distance = distance/5;
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;
		  requested_vertical_displacement = 10;
		  sprintf(motor_command,"%s,%d",reverse,multiplier);
		  while (euclideanDis(x_pos, initial_x_pos, y_pos,initial_y_pos) < requested_vertical_displacement){
			  osDelay(3);
		  }
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
	  }
	  else if (strncmp("1", (char*)command_buffer, 1) == 0){ //2,1*
		  char word[5] = "ACK";
		  char newline[] = "\n";
		  char IR[10] = "\0";
		  Servo_Centre();
		  osDelay(1000);
//		  IR_value = ADC_Read_Channel(11); //11 is left, 12 is right
		  osDelay(3);
		  while (IR_value > 1800){ //> 1800, there is an obstacle
//			  IR_value = ADC_Read_Channel(11);
			  multiplier = 200;
			  sprintf(motor_command, "%s,%d", forward, multiplier);
			  osDelay(3);
//			  IR_value = ADC_Read_Channel(11);
		  }
		  //if there is no obstacle, stop first
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  sprintf(IR, "ACK");
		  UART_send_string((uint8_t*)strcat (IR,newline));
	  }
	  else if (strncmp("2", (char*)command_buffer, 1) == 0){
	  //read right IR
		  char word[5] = "ACK";
		  char newline[] = "\n";
		  char IR[10] = "\0";
		  initial_Orientation = orientation;
		  Servo_Centre();
		  osDelay(1000);
//		  IR_value = ADC_Read_Channel(12); //11 is left, 12 is right
		  osDelay(3);
		  while (IR_value > 1800){ //> 1800, there is an obstacle
//			  IR_value = ADC_Read_Channel(12);
			  multiplier = 100; //200*
			  sprintf(motor_command, "%s,%d", forward, multiplier);
			  osDelay(3);
			  if (orientation > initial_Orientation + 0.2){
					Servo_mildRight();
				}
				else if (orientation < initial_Orientation - 0.5){
					Servo_mildLeft();
				}
//			  IR_value = ADC_Read_Channel(12);
		  }
		  //if there is no obstacle, stop first
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  sprintf(IR, "ACK");
		  UART_send_string((uint8_t*)strcat (IR,newline));
		  }
	  else if (strncmp("3", (char*)command_buffer, 1) == 0){
		  char word[5] = "ACK";
		  char newline[] = "\n";
		  char IR[10] = "\0";
		  starting_x_pos = x_pos;
		  starting_y_pos = y_pos;
		  initial_Orientation = orientation;
		  distance = 0;
		  int no_counts = 0;
		  multiplier = 200;
		  Servo_Centre();
		  osDelay(200);
		  while(no_counts < 5){
			  distance += distanceBuffer;
			  no_counts+=1;
			  osDelay(100);
		  }
		  distance = distance/5;
		  initial_x_pos = x_pos;
		  initial_y_pos = y_pos;

		  if (distance > 15){
			  requested_vertical_displacement = distance - 15;
			sprintf(motor_command, "%s,%d", forward, multiplier);
			while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
				osDelay(3);
				if (orientation > initial_Orientation + 0.2){
					Servo_mildRight();
				}
				else if (orientation < initial_Orientation - 0.5){
					Servo_mildLeft();
				}
			}
			sprintf(motor_command, "%s", stop);
			sprintf(command_buffer,"%s",stop);
			no_counts = 0;
			while (no_counts<5){
				  distance += distanceBuffer;
				  no_counts+=1;
				  osDelay(100);
			}
			distance = distance/5;
			x_pos_obstacle_1 = x_pos + distance;
			  sprintf(IR, "ACK");
			  UART_send_string((uint8_t*)strcat (IR,newline));
			  osDelay(500);
			  sprintf(command_buffer,"%s",stop);
			  sprintf(motor_command,"%s",stop);
		  }
		  else{
			  osDelay(500);
			  sprintf(IR, "ACK");
			  UART_send_string((uint8_t*)strcat (IR,newline));
			  osDelay(500);
			  sprintf(command_buffer,"%s",stop);
			  sprintf(motor_command,"%s",stop);
		  }
	  }
	  if (strncmp("4", (char*)command_buffer, 1) == 0){
	  		  //Below is original code
	  		  char word[5] = "ACK";
	  		  char newline[] = "\n";
	  		  starting_x_pos = x_pos;
	  		  starting_y_pos = y_pos;

	  		  initial_Orientation = orientation;
	  		  distance = 0;
	  		  int no_counts = 0;
	  		  multiplier = 200;
	  		  Servo_Centre();
	  		  osDelay(200);
	  		  d1 = 0;
	  		  d2 = 0;
	  		  static int i = 0;
	  		  while(no_counts < 5){
	  			  distance += distanceBuffer;
	  			  no_counts+=1;
	  			  osDelay(100);
	  		  }
	  		  distance = distance/5;
	  		  initial_x_pos = x_pos;
	  		  initial_y_pos = y_pos;
	  		  char d[10] = "\0";
	  		  //d1 = distance;
	  		  if (distance > 65){
	  			  requested_vertical_displacement = distance - 65;
	  			  multiplier = 100;
	  			sprintf(motor_command, "%s,%d", forward, multiplier);
	  			while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
	  				osDelay(3);
	  				if (orientation > initial_Orientation + 0.2){
	  					Servo_mildRight();
	  				}
	  				else if (orientation < initial_Orientation - 0.5){
	  					Servo_mildLeft();
	  				}
	  			}
	  			sprintf(motor_command, "%s", stop);
	  			sprintf(command_buffer,"%s",stop);
	  			no_counts = 0;
	  			while (no_counts<5){
	  				  distance += distanceBuffer;
	  				  no_counts+=1;
	  				  osDelay(100);
	  			}
	  			distance = distance/5;
	  			x_pos_obstacle_1 = x_pos + distance;
	  			//UART_send_string((uint8_t*)word);
	  			d2 = distanceBuffer;
	  			sprintf(d, "ACK");
	  			UART_send_string((uint8_t*)strcat (d,newline));
	  		  }
	  		  else{
	  			  osDelay(500);
	  			  osDelay(500);
	  			  sprintf(command_buffer,"%s",stop);
	  			  sprintf(motor_command,"%s",stop);
	  			  d2 = distanceBuffer;
	  			  sprintf(d, "ACK %d", (int)(30));
	  			  UART_send_string((uint8_t*)strcat (d,newline));
	  		  }
	  	  }
	  // Go to the left of second obstacle
	  else if (strncmp("2,2", (char*)command_buffer, 3) == 0){
		  offset = 0;
		  requested_angle = 20; //40*
		  multiplier = 200;
		  //delay_time = 200;
		  Servo_Centre();
		  osDelay(1000);
//		  IR_value = ADC_Read_Channel(11); //11 is left, 12 is right
		  osDelay(3);
		  while (IR_value > 1800){ //> 1800, there is an obstacle
			  multiplier = 200;
			  sprintf(motor_command, "%s,%d", forward, multiplier);
			  osDelay(3);
//			  IR_value = ADC_Read_Channel(11);
		  }
		  //if there is no obstacle, stop first
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  osDelay(1000);
		  multiplier=200;
		  Servo_turnLeft();
		  sprintf(motor_command,"%s,%d",left,multiplier);
		  osDelay(1300); //2300 180 degree, 1400 90 degree
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  Servo_Centre();
		  osDelay(1000);
		  sprintf(motor_command,"%s,%d",reverse,multiplier);
		  osDelay(400);
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
		  osDelay(1000);
		  Servo_turnLeft();
		  sprintf(motor_command,"%s,%d",left,multiplier);
		  osDelay(1200);
		  sprintf(motor_command,"%s",stop);
		  sprintf(command_buffer,"%s",stop);
	  }

	  else if (strncmp("scan obstacle", (char*)command_buffer, 13) == 0){
		  sendDistanceFromObstacle();
		  sprintf(motor_command, "%s", stop);
		  sprintf(command_buffer,"%s",stop);
	  }
	  else{
		  osDelay(100);
	  }

	  //Now, command_buffer starts to get values received from UART
	  osSemaphoreRelease(C_BufferSemaphoreHandle);
	  checkSemaphore = 1;
	  osDelay(100);
  }
  /* USER CODE END servo_motor */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  /* Infinite loop */
	//double maxDis = 80;
	const TickType_t xPeriod = pdMS_TO_TICKS(10); //100 hz
	TickType_t xLastWakeTime;
	//double wheel_C = M_PI * 0.0665; // wheel circumference in meters
	// All rotational speed of wheel is in revolution per seconds(RPS)
	double target_speed_l = 1;
	double target_speed_r = 1;
	double control = 50.0;
	double kP = 1 * control; // Proportional Gain
	double kI = 0 * control; // Integral Gain
	double kD = 2.5 * control; // Derivative Gain
	double dt = 10.0; // dT
	//float P = 0.0f, D = 0.0f, I_L = 0.0f, I_R = 0.0f;
	double err_l = 0.0, p_err_l = 0.0, err_r = 0.0, p_err_r = 0.0;
	uint8_t multiplier = 100; //50
	for(;;)
	{
	  xLastWakeTime = xTaskGetTickCount();
	  err_l = target_speed_l - fabs(Encoder_Data[0]);
	  		//Get_Encoder(Data);
	  P_L = kP * err_l;
	  I_L += err_l * kI * dt;
	  D_L = kD * (err_l - p_err_l) / dt;
	  p_err_l = err_l;
	  PWM_L += (P_L + I_L + D_L);
	  if(PWM_L>4000.0){
		  PWM_L=4000.0;
	  }
	  if(PWM_L<0.0){
		  PWM_L=0.0;
	  }
	  err_r = target_speed_r - fabs(Encoder_Data[1]);
	  P_R = kP * err_r;
	  I_R += err_r * kI * dt;
	  //D_R = kD * (p_err_r - err_r) / dt; //*
	  D_R = kD * (err_r - p_err_r) / dt;
	  p_err_r = err_r;
	  PWM_R += ((P_R + I_R + D_R));
	  if(PWM_R>4000.0){
		  PWM_R=4000.0;
	  }
	  if(PWM_R<0.0){
		  PWM_R=0.0;
	  }
	  //speed of turn 3, 0.5. Forward radius 26. 3,0.85 rr and lr is 20
	  if (strncmp((char *)motor_command,"r_right",7) == 0){ //reverse right
		  multiplier = getRequestedMultiplier(motor_command);
		  //prev values 3 and 1.5
		  target_speed_l = 1.5 * multiplier/100;
		  target_speed_r = 0 * multiplier/100;
		  Motor_Reverse((int)PWM_L, (int)PWM_R);
	  }
	  else if(strncmp((char *)motor_command, "r_left",6) == 0){ //reverse left
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 0 * multiplier/100;
		  target_speed_r = 2 * multiplier/100;

		  Motor_Reverse((int)PWM_L, (int)PWM_R);
	  }
	  else if(strncmp((char*)motor_command, "left", 4) == 0){
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 0 * multiplier/100; //task 2 new
		  target_speed_r = 2 * multiplier/100;

		  Motor_Forward((int)PWM_L, (int)PWM_R);

	  }
	  else if (strncmp((char*)motor_command, "right", 5) == 0){
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 1.5 * multiplier/100;	//task 2 new
		  target_speed_r = 0 * multiplier/100 ;

		  Motor_Forward((int)PWM_L, (int)PWM_R);

	  }
	  else if (strncmp((char*)motor_command, "reverse", 7) == 0){
		  multiplier = getRequestedMultiplier(motor_command);
		  target_speed_l = 3 * multiplier/100; //1.5*
		 target_speed_r = 3 * multiplier/100; //1.5*
		  Motor_Reverse((int)PWM_L, (int)PWM_R);
	  }

	  else if (strncmp((char*)motor_command, "forward", 7) == 0 ){

		  multiplier = getRequestedMultiplier(motor_command);
		  target_speed_l = 3 * multiplier/100;
		  target_speed_r = 3 * multiplier/100;
		  Motor_Forward((int)PWM_L, (int)PWM_R);
	  }

	  else if (strncmp((char*)motor_command, "stop", 4) == 0){
		  PWM_L = 0;
		  PWM_R = 0;
		  Motor_Forward((int)PWM_L, (int)PWM_R);
	  }
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
	  //osDelay(1);
	  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_UltrasoundRea */
/**
* @brief Function implementing the UltrasoundRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasoundRea */
void UltrasoundRea(void *argument)
{
  /* USER CODE BEGIN UltrasoundRea */
  /* Infinite loop */
  for(;;)
  {
	  HCSR04_Read();
	  distanceBuffer = HCSR04_getDistance();
	  osDelay(200);
  }
  /* USER CODE END UltrasoundRea */
}

/* USER CODE BEGIN Header_UART_comm_task */
/**
* @brief Function implementing the UART_Comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_comm_task */
//void UART_comm_task(void *argument)
//{
//  /* USER CODE BEGIN UART_comm_task */
//  /* Infinite loop */
//	  for(;;)
//	  {
//		  if (osSemaphoreAcquire(C_BufferSemaphoreHandle, 100)==0){
//			  Get_Buffer(temp_buffer);
//			  checkSemaphore = 0;
//
//			  if (temp_buffer[0]=='\0');
//			  else {
//
//				  sprintf(command_buffer,"%s",temp_buffer);
//			  }
//
//			  osSemaphoreRelease(C_BufferSemaphoreHandle);
//			  checkSemaphore = 1;
//		  }
//
//		  osDelay(100);
//	  }
//  /* USER CODE END UART_comm_task */
//}

void UART_comm_task(void *argument)
{
  for(;;)
  {
      uart_call_counter++;  // This should increment every 100ms

      if (osSemaphoreAcquire(C_BufferSemaphoreHandle, 100)==0){
          uart_acquire_counter++;  // This should increment when semaphore acquired
          Get_Buffer(temp_buffer);
          checkSemaphore = 0;

          if (temp_buffer[0]!='\0'){  // Changed condition
              buffer_data_counter++;  // This should increment when data received
              sprintf(command_buffer,"%s",temp_buffer);
          }

          osSemaphoreRelease(C_BufferSemaphoreHandle);
          checkSemaphore = 1;
      }
      osDelay(100);
  }
}

/* USER CODE BEGIN Header_Caculate_Pos */
/**
* @brief Function implementing the CaculatePositio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Caculate_Pos */
void Caculate_Pos(void *argument)
{
  /* USER CODE BEGIN Caculate_Pos */
	float offset = 105/100;
  /* Infinite loop */
  for(;;)
  {
	if (osSemaphoreAcquire(EncoderSemaphoreHandle, 100)!=0){
		  continue;
    }
	Get_Encoder(Encoder_Data);
	osSemaphoreRelease(EncoderSemaphoreHandle);
	x_pos += Encoder_Data[4]*cos(orientation*M_PI/180)*1.38 * offset;
	y_pos += Encoder_Data[4]*sin(orientation*M_PI/180)*1.38 * offset;
    osDelay(5);
  }
  /* USER CODE END Caculate_Pos */
}

/* USER CODE BEGIN Header_orientationCal */
/**
* @brief Function implementing the orientationCal1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_orientationCal */
void orientationCal(void *argument)
{
  /* USER CODE BEGIN orientationCal */
	//const TickType_t xPeriod = pdMS_TO_TICKS(10);
	const TickType_t xPeriod = 5;
	TickType_t xLastWakeTime;

  IMU_Init(&hi2c2);
  /* Infinite loop */
//  for(;;)
//  {
//	 xLastWakeTime = xTaskGetTickCount();
//    imu_read_debugValue = IMU_Read(IMU_Data);
//	orientation = (orientation+IMU_Data[6]/1000.0); //data[6]
//
//	vTaskDelayUntil(&xLastWakeTime, xPeriod);
//  }

  static double gyro_bias = -10.0; // Adjust this value

  for(;;)
  {
      xLastWakeTime = xTaskGetTickCount();
      imu_read_debugValue = IMU_Read(IMU_Data);

      // Apply bias correction before integration
      double corrected_gyro = IMU_Data[6] - gyro_bias;

      // Only integrate significant rotation (ignore small noise)
      if (abs(corrected_gyro) > 2) {
          orientation = orientation + (corrected_gyro / 1000.0);
      }

      vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
  /* USER CODE END orientationCal */
}

/* USER CODE BEGIN Header_MovementTask */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MovementTask */
void MovementTask(void *argument)
{
  /* USER CODE BEGIN MovementTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MovementTask */
}

/* USER CODE BEGIN Header_IRread */
/**
* @brief Function implementing the InfraredTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRread */
void IRread(void *argument)
{
  /* USER CODE BEGIN IRread */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IRread */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
