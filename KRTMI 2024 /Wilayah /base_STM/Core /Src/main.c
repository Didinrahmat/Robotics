/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdlib.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
int32_t tim1_cnt = 0, direction1= 0; //kiri
int32_t tim2_cnt = 0, direction2= 0; //kanan

int32_t position1 = 0; //kiri
int32_t position2 = 0; //kanan

int32_t counter1 = 0, counter2 = 0;
int32_t pengali1 = 0, pengali2 = 0;

char posBuffer[4];

volatile uint32_t rise1= 0;
volatile float freq1 = 0;

volatile uint32_t rise2= 0;
volatile float freq2 = 0;

float current_rpm1 = 0.0;
float current_rpm2 = 0.0;

int sp1; //in RPM
int sp2; //in RPM

float kp1 = 10;
float ki1 = 0.2;
float kd1 = 2;
int error1 = 0;
int last_error1 = 0, delta_error1  =  0, total_error1 = 0;
int sum_error1 = 0;
float motorSpeed1 = 0;
uint32_t  lastTime1 = 0;

float kp2 = 10;
float ki2 = 0.2;
float kd2 = 2;
int error2 = 0;
int last_error2 = 0, delta_error2  =  0, total_error2 = 0;
int sum_error2 = 0;
float motorSpeed2 = 0;
uint32_t  lastTime2 = 0;

int T = 30; //sample time in ms
int max_control = 1050; //PG45
int min_control = 0;
uint8_t tx = 6;
//uint8_t rx[24],rxadd;
char rx_buffer[9];
char rx1[1];
volatile uint8_t data_received = 0;

int cmd = 0;

int32_t targetPos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void kananCW (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	sp1 = 40;
}
void kananCCW (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	sp1 = 40;
}
void kananStop (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	sp1 = 0;
}
void kiriCW (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	sp2 = 40;
}
void kiriCCW (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
	sp2 = 40;
}
void kiriStop (void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
	sp2 = 0;
}

void stop(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	sp1 = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
	sp2 = 0;
	TIM5 -> CCR1 = 0;
	TIM5 -> CCR2 = 0;
}

void kiri(void){
	sp1 = 45;
	sp2 = 45;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);

}

void kanan(void){
	sp1 = 45;
	sp2 = 45;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);

}



void motorProcess (void){
	tim1_cnt = htim1.Instance -> CNT;
	direction1 = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));
	if (tim1_cnt == 1000 && direction1 == 1) {
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		pengali1++;
	}
	else if (tim1_cnt == 1000 && direction1 == 0){
		__HAL_TIM_SET_COUNTER(&htim1, 999);
		pengali1--;
	}
	counter1 = (1000*pengali1) + tim1_cnt;
	position1 = (float) counter1 / 10.566666666666666666666666666667;

	tim2_cnt = htim2.Instance -> CNT;
	direction2 = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));
	if (tim2_cnt == 1000 && direction2 == 1) {
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		pengali2++;
	}
	else if (tim2_cnt == 1000 && direction2 == 0){
		__HAL_TIM_SET_COUNTER(&htim2, 999);
		pengali2--;
	}
	counter2 = (1000*pengali2) + tim2_cnt;
	position2 = (float) counter2 / 10.566666666666666666666666666667;

	current_rpm1 = (freq1 * 60.0) / (200.0 * 50.9);  //motor PG45
	current_rpm2 = (freq2 * 60.0) / (200.0 * 50.9); //Motor PG45

	TIM5 -> CCR1 = motorSpeed1;
	TIM5 -> CCR2 = motorSpeed2;

	//PID Motor Left
	uint32_t currentTime1 = HAL_GetTick();
	int deltaTime1 = currentTime1 - lastTime1;
	if (deltaTime1 >= T){
		error1 = sp1 - current_rpm1;
		total_error1 += error1;
		if (total_error1>=max_control) total_error1 = max_control;
		else if (total_error1<=min_control) total_error1 = min_control;
		int delta_error1 = error1-last_error1;
		motorSpeed1 = kp1*error1 + (ki1*T)*total_error1 + (kd1/T)*delta_error1;
		if(motorSpeed1 >= max_control)motorSpeed1 = max_control;
		else if(motorSpeed1 <=  min_control) motorSpeed1 = min_control;
		last_error1 = error1;
		lastTime1 = currentTime1;
	}
	//PID Motor Back
	uint32_t currentTime2 = HAL_GetTick();
	int deltaTime2 = currentTime2 - lastTime2;
	if (deltaTime2 >= T){
		error2 = sp2 - current_rpm2;
		total_error2 += error2;
		if (total_error2>=max_control) total_error2 = max_control;
		else if (total_error2<=min_control) total_error2 = min_control;
		int delta_error2 = error2-last_error2;
		motorSpeed2 = kp2*error2 + (ki2*T)*total_error2 + (kd2/T)*delta_error2;
		if(motorSpeed2 >= max_control)motorSpeed2 = max_control;
		else if(motorSpeed2 <=  min_control) motorSpeed2 = min_control;
		last_error2 = error2;
		lastTime2 = currentTime2;
	}
}
void remoteControl (void){
	while(rx_buffer[2]==1){ //berhenti
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		kananStop();
		kiriStop();
	}
	while(rx_buffer[5]==1){ //kiri
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		kananCW();
		kiriCCW();
	}
	while(rx_buffer[6]==1){ //kanan
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
		kananCCW();
		kiriCW();
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  //HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx_buffer, sizeof(rx_buffer)); //untuk Mode Remote
  HAL_UART_Receive_DMA(&huart2, (uint8_t*)rx1, sizeof(rx1)); //untuk Mode Autonomous

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ////BEGIN Call Main Process Function////
//	  motorProcess(); //Berisi kalkulasi encoder motor, posisi, dan PID motor
//	  kiri();
//	  kiriCW();
//	  HAL_Delay(1000);
//	  kiriCCW();
//	  HAL_Delay(1000);
//	  kananCW();
//	  kiriCW();
//	  if (position2 >= 2000){
//		  kiriStop();
//	  }
	  //remoteControl(); //Berisi perintah menjalankan motor dalam mode Remote (ESP32 to STM32)
	  ////END Call Main Process Function////

	  ////BEGIN Format Syntax Komunikasi, Transmit Data STM32 to ESP32////
	  //HAL_UART_Transmit(&huart2, (uint8_t *)"1", 1, HAL_MAX_DELAY); // Mengirim data char '1'
	  //HAL_Delay(1000);
	  ////END Format Syntax Komunikasi, Transmit Data STM32 to ESP32////

	  //BEGIN Command for Autonomous Mode, Receive Data from ESP32 to STM32//
	 while(rx1[0]==48){ // Pesan Posisi A
		  targetPos = -5489;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"A", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==49){ // Pesan Posisi B
		  targetPos = -3141;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"B", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==50){//Pesan Posisi C
		  targetPos = -824;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"C", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==51){//Pesan Posisi D
		  targetPos = 1685;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"D", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==52){//Pesan Posisi E
		  targetPos = 4113;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"E", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==53){//Pesan Posisi F
		  targetPos = 5098;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"F", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==54){//Pesa Posisi G
		  targetPos = 2647;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"G", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==55){//Pesan Posisi H
		  targetPos = 417;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"H", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==56){//Pesan Posisi I
		  targetPos = -2029;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"I", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==57){//Pesan Posisi J
		  targetPos = -4931;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"J", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==65){// Pesan Posisi K
		  targetPos = -1677;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"K", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==66){ // Pesan Posisi L
		  targetPos = -824;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"L", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==67){//Pesan Posisi M
		  targetPos = 0;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"M", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==68){ //Pesan Posisi N
		  targetPos = 694;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"N", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  while(rx1[0]==69){ // Pesan Posisi O
		  targetPos = 1685;
		  motorProcess();
		  if (position1 < targetPos){
			  kanan();
//			  kananCW();
//			  kiriCCW();
		  }
		  if (position1 > targetPos){
			  kiri();
//			  kananCCW();
//			  kiriCW();
		  }
		  if (position1 == targetPos){
			  stop();
			  HAL_UART_Transmit(&huart2, (uint8_t *)"O", 1, HAL_MAX_DELAY);
			  rx1[0] = 0;
		  }
	  }
	  //END Command for Autonomous Mode, Receive Data from ESP32 to STM32//
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 16-1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 16-1;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 16-1;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 16-1;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_TRC;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1050-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    data_received = 1;
  }
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
