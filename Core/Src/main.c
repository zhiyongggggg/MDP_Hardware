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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <icm20948.h>
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
#define HARDCODE_MODE 1  // Change to 1 for hardcoded mode

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

// Juke task state variables
int movebackL = 0;
int movebackR = 0;
int moveBackLeftRun1 = 0;
int moveBackRightRun1 = 0;
int moveBackLeftRun2 = 0;
int moveBackRightRun2 = 0;
int obsTwoLength = 0;
int obsTwoFlag = 0;
char nexttask = 'Z';
int straightUS = 0;
int errorcorrection = 0;
int usTargetGLOBAL = 26;

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
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
  .priority = (osPriority_t) osPriorityNormal,
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
uint32_t IRDistBuffer[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
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
uint16_t ADC_Read_Channel(uint32_t channel)
{
 ADC_ChannelConfTypeDef sConfig = {0};

 // Configure the channel to read
 sConfig.Channel = channel;
 sConfig.Rank = 1;
 sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
 {
   Error_Handler();
 }

 // Start the ADC conversion
 if (HAL_ADC_Start(&hadc1) != HAL_OK)
 {
   Error_Handler();
 }

 // Poll for conversion completion
 if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
 {
   Error_Handler();
 }

 // Get the ADC value
 uint16_t adc_value = HAL_ADC_GetValue(&hadc1);

 // Stop the ADC conversion
 if (HAL_ADC_Stop(&hadc1) != HAL_OK)
 {
   Error_Handler();
 }

 //return ((float)adc_value/4095)*3.3;
 return adc_value;
}

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

int16_t getRepeatCount(char *p) {
    char* token;
    char* saveptr;
    int16_t repeat_count = 1;
    uint8_t field_index = 1;
    char temp_string[30];

    sprintf(temp_string,"%s",p);

    // Tokenize the command using ',' as the delimiter
    token = strtok_r(temp_string, ",", &saveptr);

    //skip the tokens that is before repeat count field index
    for (int i=0;i<field_index;i++){
        token = strtok_r(NULL, ",", &saveptr);
    }

    // retrieve the repeat count
    repeat_count = atoi(token);
    return repeat_count;
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


void sendStopAck(){
    char stop_ack[15] = "STOPPED\n";
    UART_send_string((uint8_t*)stop_ack);
}

void sendEndData(){
    char end_data[30] = "\0";
    sprintf(end_data, "END %d,%d,%d\0", (int)x_pos, (int)y_pos, (int)orientation);
    char newline[] = "\n";
    UART_send_string((uint8_t*)strcat(end_data, newline));
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
  MX_I2C2_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  UART_Comm_Init(&huart3, &hdma_usart3_rx);
  //UART_Comm_Init(&huart3);
  HCSR04_Init(&htim8, TIM_CHANNEL_2, us_Trig_GPIO_Port, us_Trig_Pin ,TIM8_Echo_GPIO_Port,TIM8_Echo_Pin );
  Motor_Init(&htim4, &htim1, &htim2, &htim5);
//  HAL_ADC_Start_DMA(&hadc1,&IR_buffer,2); //start in DMA mode and we are reading only 2 channel or 2 word
  HAL_ADC_Start_IT(&hadc1);

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
  OLED_Clear();
  OLED_ShowString(10, 10, (uint8_t*)"OS Init...");
  OLED_Refresh_Gram();
  /* USER CODE END 2 */

  /* Init scheduler */
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
//   LED_BlinkHandle = osThreadNew(BlinkLED, NULL, &LED_Blink_attributes);

  /* creation of Show */
  ShowHandle = osThreadNew(show, NULL, &Show_attributes);

  /* creation of ServoMotor */
  ServoMotorHandle = osThreadNew(servo_motor, NULL, &ServoMotor_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of UltrasoundRead */
  UltrasoundReadHandle = osThreadNew(UltrasoundRea, NULL, &UltrasoundRead_attributes);

  /* creation of UART_Comm */
  UART_CommHandle = osThreadNew(UART_comm_task, NULL, &UART_Comm_attributes);

  /* creation of CaculatePositio */
  CaculatePositioHandle = osThreadNew(Caculate_Pos, NULL, &CaculatePositio_attributes);

  /* creation of orientationCal1 */
  orientationCal1Handle = osThreadNew(orientationCal, NULL, &orientationCal1_attributes);

  /* creation of Task2 */
   Task2Handle = osThreadNew(MovementTask, NULL, &Task2_attributes);

  /* creation of InfraredTask */
  InfraredTaskHandle = osThreadNew(IRread, NULL, &InfraredTask_attributes);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED_DC_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(us_Trig_GPIO_Port, us_Trig_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : us_Trig_Pin */
  GPIO_InitStruct.Pin = us_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(us_Trig_GPIO_Port, &GPIO_InitStruct);

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
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
  char displayStr[30];

  for(;;)
  {
	OLED_Clear();
//	HCSR04_Read();

	// Display current command
	sprintf(displayStr, "Cmd: %.12s", command_buffer);
	OLED_ShowString(10, 00, (uint8_t*)displayStr);

	// Display position
	sprintf(displayStr, "X:%d Y:%d", (int)x_pos, (int)y_pos);
	OLED_ShowString(10, 10, (uint8_t*)displayStr);

	// Display orientation
	sprintf(displayStr, "Orient: %d", (int)orientation);
	OLED_ShowString(10, 20, (uint8_t*)displayStr);

//	// Display Echo
//	sprintf(displayStr, "Echo: %d", (int)HCSR04_getDifference());
//	OLED_ShowString(10, 40, (uint8_t*)displayStr);

	// Display Distance
	sprintf(displayStr, "Distance: %d", (int)distanceBuffer);
	OLED_ShowString(10, 30, (uint8_t*)displayStr);

	sprintf(displayStr, "IR(L): %d", (int)left_IR);
//	sprintf(displayStr, "IRDist: %d (L), %d (R)", (int)IR_buffer[0], (int)IR_buffer[1]);
	OLED_ShowString(10, 40, (uint8_t*)displayStr);
//

	// Display IR Dist
	sprintf(displayStr, "IR(R): %d", (int)right_IR);
//	sprintf(displayStr, "IRDist: %d (L), %d (R)", (int)IR_buffer[0], (int)IR_buffer[1]);
	OLED_ShowString(10, 50, (uint8_t*)displayStr);
//
//	// Display IR Dist
//	sprintf(displayStr, "IR: %d(L), %d(R)", (int)left_IR, (int)right_IR);
////	sprintf(displayStr, "IRDist: %d (L), %d (R)", (int)IR_buffer[0], (int)IR_buffer[1]);
//	OLED_ShowString(10, 20, (uint8_t*)displayStr);




	OLED_Refresh_Gram();
	osDelay(100);
  }
  /* USER CODE END show */
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
  double compensation_delay = 35;
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
		 // sendEndData();
		 // osDelay(100);
		//  sprintf(command_buffer, "%s", "");
		  continue;
	  }

	  if (osSemaphoreAcquire(C_BufferSemaphoreHandle, 100)!=0){
		  continue;
	  }

	  if ((strncmp("stop", (char*)command_buffer, 4) == 0)){
		  Servo_Centre();
		  sprintf(motor_command, "%s", stop);
		//  sendStopAck();
		  osDelay(500);
		//  sprintf(command_buffer, "%s", "");
	  }

	  else if (strncmp("end", (char*)command_buffer, 3) == 0){
	      // Stop all motors
	      Servo_Centre();
	      sprintf(motor_command, "%s", stop);

	      // Send END message with position coordinates
	      sendEndData();

	      // Clear command buffer
	      osDelay(100);
	      sprintf(command_buffer, "%s", dummy_command_stop);
	  }







	  //Forward Right turn - FIXED VERSION
	  else if ((strncmp("right", (char*)command_buffer, 5) == 0) && (strstr((char*)command_buffer, "forward") != NULL)){

            requested_angle = -getRequestedAngle(command_buffer);
            offset = 0.5;  // FIXED: Changed from -2.98 to 0.5 for consistent behavior
            multiplier = 200;

			target_Orientation = (orientation + requested_angle + offset);
			sprintf(motor_command, "%s", stop);
			Servo_turnRight();
			osDelay(10);

			sprintf(motor_command, "%s,%d", right, multiplier);

			while (orientation > target_Orientation ){
				// FIXED: Updated slow-down logic to match left turn behavior
				if (orientation < target_Orientation + 10){  // Changed from 20 to 10 for better precision
					multiplier = 30; // Changed from 20 to 30 for better completion
					sprintf(motor_command, "%s,%d", right, multiplier);
				}
				osDelay(3);
			}
			//Servo_fixLeft();
			Servo_Centre();
			sprintf(motor_command,"%s",stop);
			sprintf(command_buffer,"%s",dummy_command_stop);

			osDelay(200);
			sendPosOrienData();
			osDelay(100);
	  }

	  else if ((strncmp("left", (char*)command_buffer, 4) == 0) && (strstr((char*)command_buffer, "forward") != NULL)){

		    requested_angle = getRequestedAngle(command_buffer);
		    multiplier = 200;
		    offset = -1.5;
			target_Orientation = orientation + requested_angle + offset;

			sprintf(motor_command, "%s", stop);
			Servo_turnLeft();
			osDelay(10);
			sprintf(motor_command, "%s,%d", left, multiplier);

			while (orientation < target_Orientation){
				//this delay is necessary or there will be scheduling issues
				if (orientation > target_Orientation - 35 ){ 
					//add a multiplier. 20  gives exact angle. 30 to be faster
					multiplier = 20; 
					sprintf(motor_command, "%s,%d", left, multiplier);
				}
				osDelay(3);
			}

			//Servo_fixRight();
			Servo_Centre();
			sprintf(motor_command, "%s", stop);
			sprintf(command_buffer,"%s",dummy_command_stop);
			osDelay(200);
			sendPosOrienData();
			osDelay(100);

	  }

	  //Reverse right Turn - FIXED VERSION
	  else if((strncmp("right", (char*)command_buffer, 5) == 0) && (strstr((char*)command_buffer, "reverse") != NULL)){
		  requested_angle = getRequestedAngle(command_buffer);
		  Servo_Centre();

		  offset = 0.5;  // FIXED: Changed from 0.58333 to 0.5 for consistency
		  
		  target_Orientation = orientation + requested_angle + offset;

		  sprintf(motor_command, "%s", stop);
		  Servo_turnRight();
		  sprintf(motor_command, "%s,%d", reverse_right, multiplier);

		  while (orientation < target_Orientation){
				// FIXED: Updated slow-down logic for consistency
				if (orientation > target_Orientation - 10){ // Changed from + to - and adjusted threshold
					multiplier = 30; // Increased from 20 to 30
					sprintf(motor_command, "%s,%d", reverse_right, multiplier);
				}
			  osDelay(3);
		  }
		  //Servo_fixLeft();
		  Servo_Centre();
		  sprintf(motor_command, "%s", stop);

		  sprintf(command_buffer,"%s",dummy_command_stop);
		  osDelay(200);
		  sendPosOrienData();
		  osDelay(100);

	  }

	  //Reverse left Turn - FIXED VERSION
	  else if((strncmp("left", (char*)command_buffer, 4) == 0) && (strstr((char*)command_buffer, "reverse") != NULL)){
		  requested_angle = -getRequestedAngle(command_buffer);
		  offset = 9;  // FIXED: Changed from -3.3333333 to -0.5 for consistency
		  Servo_Centre();

		  target_Orientation = orientation + requested_angle + offset;

		  sprintf(motor_command, "%s", stop);
		  Servo_turnLeft();

		  sprintf(motor_command, "%s,%d", reverse_left, multiplier);

		  while (orientation > target_Orientation){
				// FIXED: Updated slow-down logic for consistency
				if (orientation < target_Orientation + 10){ // Adjusted threshold
					multiplier = 30; // Increased from 20 to 30
					sprintf(motor_command, "%s,%d", reverse_left, multiplier);
				}
			  osDelay(3);
		  }
		  //Servo_fixRight();
		  Servo_Centre();
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

				if (offset_from_center > 0.5){
					Servo_mildRight();
//		offset_from_center = 0;
				}
				else if (offset_from_center < -0.5){
					Servo_mildLeft();
//					offset_from_center = 0;
				}
				else {
				    Servo_Centre();      // Within acceptable range → go straight
				}

				if (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) > requested_vertical_displacement - 20){
					multiplier = 40; 
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

	      while (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
	          offset_from_center = orientation - target_Orientation;

	          if (offset_from_center == 0){
	              Servo_Centre();
	          }

	          // REVERSE STEERING: corrections are opposite of forward
	          if (offset_from_center > 0.5){
	              Servo_mildLeft();  // Reversed from forward
	          }
	          else if (offset_from_center < -0.5){
	              Servo_mildRight(); // Reversed from forward
	          }
	          else {
	              Servo_Centre();
	          }

	          if (euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) > requested_vertical_displacement - 20){
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

	  // Spot turn left
	  else if((strncmp("spot", (char*)command_buffer, 4) == 0) && (strstr((char*)command_buffer, "left") != NULL)){
	      requested_angle = getRequestedAngle(command_buffer);
	      offset = -1.5;  // Adjust as needed
	      target_Orientation = orientation + requested_angle + offset;

	      sprintf(motor_command, "%s", stop);
	      Servo_Centre();  // Keep servo centered for spot turn
	      osDelay(10);

	      sprintf(motor_command, "%s,%d", "spot_left", multiplier);

	      while (orientation < target_Orientation){
	          if (orientation > target_Orientation - 10){
	              multiplier = 30;
	              sprintf(motor_command, "%s,%d", "spot_left", multiplier);
	          }
	          osDelay(3);
	      }

	      sprintf(motor_command, "%s", stop);
	      sprintf(command_buffer, "%s", dummy_command_stop);
	      osDelay(200);
	      sendPosOrienData();
	      osDelay(100);
	  }

//	  // Obstacle movement sequence
//	  else if(strncmp("obstacle", (char*)command_buffer, 8) == 0){
//
//		    int repeat_count = getRepeatCount(command_buffer);
//		    int multiplier_obstacle = 200;
//
//		    for(int cycle = 0; cycle < repeat_count; cycle++){
//
//		        // Step 1: Move forward
//		        sprintf(motor_command, "%s,%d", forward, multiplier_obstacle);
//
//		        initial_x_pos = x_pos;
//		        initial_y_pos = y_pos;
//		        requested_vertical_displacement = 5; // Forward distance
//
//		        while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
//		            osDelay(10);
//		        }
//
//		        sprintf(motor_command, "%s", stop);
//		        osDelay(500);
//
//		        // Step 2: Reverse
//		        sprintf(motor_command, "%s,%d", reverse, multiplier_obstacle);
//
//		        initial_x_pos = x_pos;
//		        initial_y_pos = y_pos;
//		        requested_vertical_displacement = 100; // Reverse distance
//
//		        while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
//		            osDelay(10);
//		        }
//
//		        sprintf(motor_command, "%s", stop);
//		        osDelay(500);
//
//		        // Step 3: Forward turn left 90 degrees
//		        requested_angle = 90;
//		        offset = -1.5;
//		        target_Orientation = orientation + requested_angle + offset;
//
//		        sprintf(motor_command, "%s", stop);
//		        Servo_turnLeft();
//		        osDelay(10);
//		        sprintf(motor_command, "%s,%d", left, multiplier_obstacle);
//
//		        while (orientation < target_Orientation){
//		            if (orientation > target_Orientation - 35 ){
//		                multiplier_obstacle = 20;
//		                sprintf(motor_command, "%s,%d", left, multiplier_obstacle);
//		            }
//		            osDelay(3);
//		        }
//
//		        Servo_fixRight();
//		        sprintf(motor_command, "%s", stop);
//		        osDelay(500);
//		        multiplier_obstacle = 200; // Reset multiplier
//
//		        // Step 4: Forward turn right 90 degrees
//		        requested_angle = -90;
//		        offset = 0.5;
//		        target_Orientation = orientation + requested_angle + offset;
//
//		        sprintf(motor_command, "%s", stop);
//		        Servo_turnRight();
//		        osDelay(10);
//		        sprintf(motor_command, "%s,%d", right, multiplier_obstacle);
//
//		        while (orientation > target_Orientation ){
//		            if (orientation < target_Orientation + 10){
//		                multiplier_obstacle = 30;
//		                sprintf(motor_command, "%s,%d", right, multiplier_obstacle);
//		            }
//		            osDelay(3);
//		        }
//		        Servo_fixLeft();
//		        sprintf(motor_command,"%s",stop);
//		        osDelay(500);
//		        multiplier_obstacle = 200; // Reset multiplier
//
//		        // Step 5: Forward turn right 90 degrees again
//		        requested_angle = -90;
//		        offset = 0.5;
//		        target_Orientation = orientation + requested_angle + offset;
//
//		        sprintf(motor_command, "%s", stop);
//		        Servo_turnRight();
//		        osDelay(10);
//		        sprintf(motor_command, "%s,%d", right, multiplier_obstacle);
//
//		        while (orientation > target_Orientation ){
//		            if (orientation < target_Orientation + 10){
//		                multiplier_obstacle = 30;
//		                sprintf(motor_command, "%s,%d", right, multiplier_obstacle);
//		            }
//		            osDelay(3);
//		        }
//		        Servo_fixLeft();
//		        sprintf(motor_command,"%s",stop);
//		        osDelay(500);
//		        multiplier_obstacle = 200; // Reset multiplier
//
//		        // Step 1: Move forward
//		        sprintf(motor_command, "%s,%d", forward, multiplier_obstacle);
//
//		        initial_x_pos = x_pos;
//		        initial_y_pos = y_pos;
//		        requested_vertical_displacement = 15; // Forward distance
//
//		        while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
//		            osDelay(10);
//		        }
//
//		        sprintf(motor_command, "%s", stop);
//		        osDelay(500);
//
//		        // Delay between cycles if more repetitions needed
//		        if(cycle < repeat_count - 1){
//		            osDelay(1000);
//		        }
//		    }
//
//		    sprintf(motor_command, "%s", stop);
//		    sprintf(command_buffer, "%s", dummy_command_stop);
//		    UART_send_string(movement_finish);
//		    sendPosOrienData();
//		    osDelay(200);
//		}

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
		  osDelay(800);
		  Servo_Centre();
		  osDelay(100);
		  multiplier=500;
		  Servo_turnRight();
		  sprintf(motor_command,"%s,%d",right,multiplier);
		  osDelay(3000);
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
		  while (left_IR > 1800){ //> 1800, there is an obstacle
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
		  while (right_IR > 1800){ //> 1800, there is an obstacle
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
	double kI = 0.001 * control; // Integral Gain
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
	  if(PWM_L>3000.0){
		  PWM_L=3000.0;
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
	  if(PWM_R>3000.0){
		  PWM_R=3000.0;
	  }
	  if(PWM_R<0.0){
		  PWM_R=0.0;
	  }
	  //speed of turn 3, 0.5. Forward radius 26. 3,0.85 rr and lr is 20
	  if (strncmp((char *)motor_command,"r_right",7) == 0){ //reverse right
		  multiplier = getRequestedMultiplier(motor_command);
		  //prev values 3 and 1.5
		  target_speed_l = 1.2 * multiplier/100;
		  target_speed_r = 0 * multiplier/100;
		  Motor_Reverse(0.5*(int)PWM_L, 0.05*(int)PWM_R);
	  }
//	  else if(strncmp((char *)motor_command, "r_left",6) == 0){ //reverse left
//		  multiplier = getRequestedMultiplier(motor_command);
//
//		  target_speed_l = 0 * multiplier/100;
//		  target_speed_r = 1.2 * multiplier/100;
//
//		  Motor_Reverse(0.18*(int)PWM_L, 0.58*(int)PWM_R);
//	  }
//	  else if(strncmp((char*)motor_command, "left", 4) == 0){
//		  multiplier = getRequestedMultiplier(motor_command);
//
//		  target_speed_l = 0 * multiplier/100; //task 2 new
//		  target_speed_r = 1.2 * multiplier/100;
//
//		  Motor_Forward(0.3*(int)PWM_L, 0.68*(int)PWM_R);
//
//	  }
	  else if(strncmp((char *)motor_command, "r_left",6) == 0){ //reverse left
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 0 * multiplier/100;
		  target_speed_r = 1.2 * multiplier/100;

		  Motor_Reverse(0.05*(int)PWM_L, 0.5*(int)PWM_R);
	  }
	  else if(strncmp((char*)motor_command, "left", 4) == 0){
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 0 * multiplier/100; //task 2 new
		  target_speed_r = 1.2 * multiplier/100;

		  Motor_Forward(0.05*(int)PWM_L, 0.5*(int)PWM_R);

	  }
	  else if (strncmp((char*)motor_command, "right", 5) == 0){
		  multiplier = getRequestedMultiplier(motor_command);

		  target_speed_l = 1.2 * multiplier/100;	//task 2 new
		  target_speed_r = 0 * multiplier/100 ;


		  Motor_Forward(0.5*(int)PWM_L, 0.05*(int)PWM_R);

	  }
	  else if (strncmp((char*)motor_command, "reverse", 7) == 0){
		  multiplier = getRequestedMultiplier(motor_command);
		  target_speed_l = 3 * multiplier/100; //1.5*
		 target_speed_r = 3 * multiplier/100; //1.5*
		  Motor_Reverse((int)PWM_L, (int)PWM_R*0.93);
	  }

	  else if (strncmp((char*)motor_command, "forward", 7) == 0 ){

		  multiplier = getRequestedMultiplier(motor_command);
		  target_speed_l = 2.0 * multiplier/100;
		  target_speed_r = 2.0 * multiplier/100;
		  Motor_Forward((int)PWM_L*0.913, (int)PWM_R);
	  }

	  else if(strncmp((char*)motor_command, "spot_left", 9) == 0){
	      multiplier = getRequestedMultiplier(motor_command);

	      // Left wheel goes backward, right wheel goes forward
	      target_speed_l = 2.0 * multiplier/100;
	      target_speed_r = 2.0 * multiplier/100;

	      Motor_Reverse((int)PWM_L, 0);      // Left backward
	      Motor_Forward(0, (int)PWM_R);      // Right forward
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
void UART_comm_task(void *argument)
{
  /* USER CODE BEGIN UART_comm_task */
  /* Infinite loop */
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
  /* USER CODE END UART_comm_task */
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

  static double gyro_bias = -48.0; // Adjust this value

  orientation = orientation +17 ;

  for(;;)
  {
      xLastWakeTime = xTaskGetTickCount();
      imu_read_debugValue = IMU_Read(IMU_Data);

      // Apply bias correction before integration
      double corrected_gyro = IMU_Data[6] - gyro_bias;

      // Only integrate significant rotation (ignore small noise)
      if (fabs(corrected_gyro) > 8) {
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
  while(strncmp(command_buffer, "task2", 5) != 0){}


  // Wait for system initialization
  osDelay(2000);
  
  char ack_msg[] = "A\n";
  char done_msg[] = "D\n";
  
  // Reset encoder values
  if (osSemaphoreAcquire(EncoderSemaphoreHandle, 100) == 0) {
      Encoder_Data[2] = 0;  // Reset left encoder
      Encoder_Data[3] = 0;  // Reset right encoder
      osSemaphoreRelease(EncoderSemaphoreHandle);
  }
  
  /* ========== PHASE 1: APPROACH FIRST OBSTACLE ========== */
  
  #if HARDCODE_MODE
      // Hardcoded: Set command directly
      osDelay(100);
  #else
      // Wait for 'S' command from RPI (or command "0")
      while(strncmp(command_buffer, "0", 1) != 0) {
          osDelay(100);
      }
  #endif
  
  // Variables for obstacle approach
  char ack_response[20] = "\0";
  char newline[] = "\n";
  double starting_x_pos = x_pos;
  double starting_y_pos = y_pos;
  double initial_Orientation = orientation;
  int distance = 0;
  int no_counts = 0;
  int multiplier = 100;
  int d1 = 0;
  int d2 = 0;
  int requested_vertical_displacement = 0;
  char displayStr[30];
  
  Servo_Centre();
  osDelay(200);
  
  // Average 5 ultrasound readings
  distance = 0;
  no_counts = 0;
  while(no_counts < 5){
      distance += distanceBuffer;
      no_counts++;
      osDelay(100);
  }
  distance = distance / 5;
  
  double initial_x_pos = x_pos;
  double initial_y_pos = y_pos;
  
  d1 = distance;
  
  // Move forward if obstacle is far enough
  if (distance > 35){
      requested_vertical_displacement = distance - 35;
      multiplier = 100;
      
      sprintf(motor_command, "%s,%d", forward, multiplier);
      
      while(euclideanDis(x_pos, initial_x_pos, y_pos, initial_y_pos) < requested_vertical_displacement){
          osDelay(3);
          // Maintain straight orientation
          if (orientation > initial_Orientation + 0.2){
              Servo_mildRight();
          }
          else if (orientation < initial_Orientation - 0.5){
              Servo_mildLeft();
          }
      }
      
      sprintf(motor_command, "%s", stop);
      sprintf(command_buffer, "%s", stop);
      osDelay(200);
      
      // Re-measure distance after moving
      distance = 0;
      no_counts = 0;
      while (no_counts < 5){
          distance += distanceBuffer;
          no_counts++;
          osDelay(100);
      }
      distance = distance / 5;
      
      d2 = distanceBuffer;
      sprintf(ack_response, "ACK %d", (int)(d1 - d2));
      UART_send_string((uint8_t*)strcat(ack_response, newline));
  }
  else{
      // Already close enough
      osDelay(1000);
      sprintf(command_buffer, "%s", stop);
      sprintf(motor_command, "%s", stop);
      d2 = distanceBuffer;
      sprintf(ack_response, "ACK %d", (int)(30));
      UART_send_string((uint8_t*)strcat(ack_response, newline));
  }
  
  // Fine-tune position if too close
  if(distance <= 30 && distance > 25){
      sprintf(motor_command, "%s,%d", reverse, multiplier);
      osDelay(200);
      sprintf(command_buffer, "%s", stop);
      sprintf(motor_command, "%s", stop);
  }
  if(distance <= 25 && distance > 20){
      sprintf(motor_command, "%s,%d", reverse, multiplier);
      osDelay(300);
      sprintf(command_buffer, "%s", stop);
      sprintf(motor_command, "%s", stop);
  }
  
  // Save distance traveled for return journey
  moveBackLeftRun1 = (int)euclideanDis(x_pos, starting_x_pos, y_pos, starting_y_pos);
  moveBackRightRun1 = moveBackLeftRun1;
  
  // Display on OLED
  sprintf(displayStr, "Obs1: %dcm", (int)distance);
  OLED_ShowString(10, 10, (uint8_t*)displayStr);
  OLED_Refresh_Gram();
  osDelay(500);
  
  /* ========== PHASE 2: FIRST OBSTACLE MANEUVER ========== */
  
  #if HARDCODE_MODE
      // Hardcoded: Set first direction (L or R)
      nexttask = 'L';  // Change to 'R' for right turn
      osDelay(100);
  #else
      // Wait for direction command (L or R)
      nexttask = 'Z';
      while(nexttask == 'Z') {
          if(command_buffer[0] == 'L' || command_buffer[0] == 'R') {
              nexttask = command_buffer[0];
          }
          osDelay(50);
      }
  #endif
  
  // Execute first obstacle maneuver based on direction
  double initial_orient = orientation;
  
  if (nexttask == 'R') {
      // Right maneuver: S-curve right
      sprintf(command_buffer, "right,45,forward,20");
      osDelay(2000);  // Wait for turn completion
      
      sprintf(command_buffer, "left,95,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "right,45,forward,20");
      osDelay(2000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Orientation should now be approximately initial_orient - 5
      
  } else if (nexttask == 'L') {
      // Left maneuver: S-curve left
      sprintf(command_buffer, "left,50,forward,20");
      osDelay(2000);
      
      sprintf(command_buffer, "center,0,forward,100");
      osDelay(2000);

      sprintf(command_buffer, "right,95,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "left,45,forward,20");
      osDelay(2000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Orientation should now be approximately initial_orient + 5
  }
  
  // Send acknowledgment
  UART_send_string((uint8_t*)ack_msg);
  
  /* ========== PHASE 3: APPROACH SECOND OBSTACLE ========== */
  
  #if HARDCODE_MODE
      // Hardcoded: Proceed to second obstacle
      osDelay(500);
  #else
      // Wait for 'D' command (done with first obstacle)
      while(command_buffer[0] != 'D') {
          osDelay(50);
      }
  #endif
  
  // Reset encoders for second run
  if (osSemaphoreAcquire(EncoderSemaphoreHandle, 100) == 0) {
      Encoder_Data[2] = 0;
      Encoder_Data[3] = 0;
      osSemaphoreRelease(EncoderSemaphoreHandle);
  }
  
  // Move to second obstacle
  straightUS = 1;
  errorcorrection = 1;
  usTargetGLOBAL = 28;
  
  sprintf(command_buffer, "center,0,forward,100");
  
  while(distanceBuffer > usTargetGLOBAL + 2) {
      osDelay(50);
  }
  
  sprintf(command_buffer, "stop");
  osDelay(500);
  
  straightUS = 0;
  errorcorrection = 0;
  
  // Record second run movement
  if (osSemaphoreAcquire(EncoderSemaphoreHandle, 100) == 0) {
      movebackL += Encoder_Data[2];
      movebackR += Encoder_Data[3];
      moveBackLeftRun2 = Encoder_Data[2];
      moveBackRightRun2 = Encoder_Data[3];
      osSemaphoreRelease(EncoderSemaphoreHandle);
  }
  
  /* ========== PHASE 4: SECOND OBSTACLE MANEUVER ========== */
  
  #if HARDCODE_MODE
      // Hardcoded: Set second direction
      nexttask = 'R';  // Change to 'L' for left turn
      osDelay(100);
  #else
      // Wait for second direction command
      nexttask = 'Z';
      while(nexttask == 'Z') {
          if(command_buffer[0] == 'L' || command_buffer[0] == 'R') {
              nexttask = command_buffer[0];
          }
          osDelay(50);
      }
  #endif
  
  // Execute second obstacle maneuver
  initial_orient = orientation;
  
  if (nexttask == 'R') {
      // Turn right 90 degrees
      sprintf(command_buffer, "right,88,forward,30");
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Reverse slightly
      sprintf(command_buffer, "center,0,reverse,13");
      osDelay(2000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Move forward until IR sensor clears obstacle
      sprintf(command_buffer, "1");
      osDelay(100);
      
      // Turn around (180 degrees)
      sprintf(command_buffer, "left,180,forward,30");
      osDelay(5000);
      

      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Move forward measuring obstacle length
      obsTwoLength = 0;
      obsTwoFlag = 0;
      
      sprintf(command_buffer, "2");
      osDelay(100);
      
      // Turn left 90 degrees
      sprintf(command_buffer, "left,90,forward,30");
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
  } else if (nexttask == 'L') {
      // Similar logic for left turn (mirror of right)
      sprintf(command_buffer, "left,88,forward,30");
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      sprintf(command_buffer, "center,0,reverse,13");
      osDelay(2000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      sprintf(command_buffer, "2");
      osDelay(100);
      
      sprintf(command_buffer, "right,180,forward,30");
      osDelay(5000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      obsTwoLength = 0;
      obsTwoFlag = 0;
      
      sprintf(command_buffer, "1");
      osDelay(100);
      
      sprintf(command_buffer, "right,90,forward,30");
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
  }
  
  /* ========== PHASE 5: RETURN TO CARPARK ========== */
  
  // Calculate return distance
  int returnDistance = (moveBackRightRun2 + moveBackLeftRun2) / 2 + 13907;
  int distanceToMove = (returnDistance / 75.6) - 85;
  
  // Move back most of the way
  errorcorrection = 1;
  sprintf(command_buffer, "center,0,forward,%d", distanceToMove);
  
  // Wait for movement completion
  double start_x = x_pos;
  double start_y = y_pos;
  while(euclideanDis(x_pos, start_x, y_pos, start_y) < distanceToMove) {
      osDelay(50);
  }
  
  sprintf(command_buffer, "stop");
  osDelay(500);
  errorcorrection = 0;
  
  /* ========== PHASE 6: FINAL CARPARK POSITIONING ========== */
  
  if (nexttask == 'R') {
      // Slide turn left into carpark
      sprintf(command_buffer, "left,70,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Move forward to center
      int parkDistance = ((obsTwoLength * 14) / 2);
      if (parkDistance > 55) parkDistance = 55;
      
      sprintf(command_buffer, "center,0,forward,%d", parkDistance);
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      // Turn right to straighten
      sprintf(command_buffer, "right,70,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
  } else if (nexttask == 'L') {
      // Slide turn right into carpark
      sprintf(command_buffer, "right,70,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      int parkDistance = ((obsTwoLength * 14) / 2);
      if (parkDistance > 55) parkDistance = 55;
      
      sprintf(command_buffer, "center,0,forward,%d", parkDistance);
      osDelay(3000);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
      
      sprintf(command_buffer, "left,70,forward,20");
      osDelay(2500);
      
      sprintf(command_buffer, "stop");
      osDelay(500);
  }
  
  /* ========== PHASE 7: FINAL APPROACH ========== */
  
  // Final ultrasound approach to carpark wall
  straightUS = 1;
  errorcorrection = 1;
  usTargetGLOBAL = 22;
  
  sprintf(command_buffer, "center,0,forward,50");
  
  while(distanceBuffer > usTargetGLOBAL + 2) {
      osDelay(50);
  }
  
  sprintf(command_buffer, "stop");
  straightUS = 0;
  errorcorrection = 0;
  
  // Task complete - send final acknowledgment
  UART_send_string((uint8_t*)"TASK_COMPLETE\n");
  
  // Infinite loop - task complete
  for(;;) {
      osDelay(10000);
  }
  
  /* USER CODE END MovementTask */
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
	float leftDistance = 0.0f;
	float rightDistance = 0.0f;

	for(;;)
	{
		left_IR = ADC_Read_Channel(11);
		right_IR = ADC_Read_Channel(12);
//		IRDistBuffer[1] = 0;
		// Convert raw ADC values (0–4095) to voltages
		float leftVolt = (left_IR * 3.3f) / 4095.0f;
		float rightVolt = (right_IR * 3.3f) / 4095.0f;
//
//		// Convert voltage to approximate distance (cm)
//		// Based on Sharp GP2Y0A21YK curve (~27.86 / (V - 0.42))
		leftDistance = 27.86f / (leftVolt - 0.42f);
		rightDistance = 27.86f / (rightVolt - 0.42f);
//		rightDistance = 27.86f / (rightVolt - 0.42f);
//((float)adc_value/4095)*3.3;
//		// Optional: clamp values
		if(leftDistance > 80) leftDistance = 80;
		if(rightDistance > 80) rightDistance = 80;
//		if(rightDistance > 80) rightDistance = 80;
//
//		// You can now write these distances to a global struct or queue
//		// Example:
		 IRDistBuffer[0] = leftDistance;
		 IRDistBuffer[1] = rightDistance;

		osDelay(100); // Read every 100 ms
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
