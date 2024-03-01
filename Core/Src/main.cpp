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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AMT22.h"
#include "RoboArm.h"
//#include "TMC2209.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

//osThreadId defaultTaskHandle;
//osThreadId myAMT22TaskHandle;
//osThreadId myUARTTaskHandle;
/* USER CODE BEGIN PV */

uint8_t rx_index = 0;
uint8_t rx_data[20];
uint8_t rx_buffer[12];
uint8_t transfer_cplt = 0;

bool startFirstMove = false;
bool sendDataFlag = false;
bool stopHand = false;
bool setZeroFlag = false;

bool timerFT1 = false;
bool timerFT2 = false;
bool startCorrectPos = false;
bool posCorrected = false;

float accuracy = 0.05;

float recAngleF = 0.0;
uint16_t recDist = 0;
uint16_t recHold = 0;
bool flagReadEnc = 0;

int encMax = 16384;
int linMax = 250;

union UN {
	struct {
		float lin;
		float ang;
		int hold;
	} params;
	uint8_t bytes[sizeof(params)];
} un_get, un_send, un_to, un_now;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
//void StartDefaultTask(void const * argument);
//void StartAMT22Data(void const * argument);
//void StartUARTData(void const * argument);
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t cntImpulse1 = 0, cntImpulse2 = 0, cntImpulse3 = 0, step1 = 0, step2 = 0;
bool allowMove = true, gripperMoveFinished = true;

RoboArm arm(0, 124);
//RoboArm arm(0, 124);

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Init(&htim1);
  HAL_TIM_Base_Init(&htim2);

  arm.SetSettMotors(huart2,htim1, htim2, htim3, Dir1_GPIO_Port, Dir1_Pin,
		  Dir2_GPIO_Port, Dir2_Pin, Dir3_GPIO_Port, Dir3_Pin, En1_GPIO_Port,
		  En1_Pin, En2_GPIO_Port, En2_Pin, En3_GPIO_Port, En3_Pin);

  arm.SetSettEncoders(hspi1, CS1_GPIO_Port, CS1_Pin, CS2_GPIO_Port, CS2_Pin,
		  14);
  un_now.params.hold = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_UART_Receive_IT (&huart1, str, 1);
  HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
  arm.setPrintState(true);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (startFirstMove) {
		  startFirstMove = false;

		  if (un_now.params.hold != 0){
			  allowMove = false;
			  un_now.params.hold = 0;
			  arm.SetGripper(0);
		  }
		  while (!allowMove) {}
		  if (allowMove) {
			  arm.Move2Motors(un_to.params.ang, un_to.params.lin);
		  }
	  }

	  //з докаткою
//	  if (timerFT1 && timerFT2) {
//
//		  timerFT1 = false;
//		  timerFT2 = false;
//
//		  float lin = arm.GetLin();
//		  un_send.params.lin = lin; //для АМТ223В-V
////		  un_now.params.lin = arm.ShiftZeroLin(lin); //для АМТ223С-V
//
//		  float ang = arm.GetAng();
//		  un_send.params.ang = ang; //для АМТ223В-V
//		  un_now.params.ang = arm.ShiftZeroAng(ang); //для АМТ223С-V
//		  if (posCorrected) {
////			  timerFT1 = false;
////			  timerFT2 = false;
//			  posCorrected = false;
//
//			  if (abs(un_now.params.ang - un_to.params.ang) > accuracy ||
//					  abs(un_now.params.ang - un_to.params.ang) > accuracy) {
//				  startCorrectPos = true;
//				  arm.MoveCorrectPosition(un_to.params.ang, un_to.params.lin);
//			  } else {
//				  startCorrectPos = false;
//				  if (un_now.params.hold != un_to.params.hold) {
//					  gripperMoveFinished = false;
//					  un_now.params.hold = un_to.params.hold;
//					  arm.SetGripper(un_to.params.hold);
//				  }
//				  while (!gripperMoveFinished) {}
//				  if (gripperMoveFinished) {
//					  un_send.params.lin = 0;
//					  un_send.params.ang = 0;
//					  un_send.params.hold = 10;
//					  HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
//							  12);
//				  }
//			  }
//		  } else {
//			  if (abs(un_now.params.ang - un_to.params.ang) > accuracy ||
//			  					  abs(un_now.params.ang - un_to.params.ang) > accuracy) {
//				  startCorrectPos = true;
//				  arm.MoveCorrectPosition(un_to.params.ang, un_to.params.lin);
//			  }
////			  arm.MoveCorrectPosition(un_to.params.ang, un_to.params.lin);
//		  }
//	  }

	  if (timerFT1 && timerFT2) {
		  timerFT1 = false;
		  timerFT2 = false;
		  un_now.params.lin = un_to.params.lin;
		  un_now.params.ang = un_to.params.ang;

		  //виставляємо зачеп у потрібну позицію
		  if (un_now.params.hold != un_to.params.hold) {
			  gripperMoveFinished = false;
			  un_now.params.hold = un_to.params.hold;
			  arm.SetGripper(un_to.params.hold);
		  }
		  while (!gripperMoveFinished) {}
		  if (gripperMoveFinished) {
			  un_send.params.lin = 0;
			  un_send.params.ang = 0;
			  un_send.params.hold = 10;
			  HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					  12);
		  }

	  }

	  if (arm.getPrintState() && sendDataFlag) {
		  sendDataFlag = false;

		  float lin = arm.GetLin();
		  un_send.params.lin = lin;
//		  un_send.params.lin = arm.ShiftZeroLin(lin);

		  float ang = arm.GetAng();
		  un_send.params.ang = ang;
//		  un_send.params.ang = arm.ShiftZeroAng(ang);

		  un_send.params.hold = un_now.params.hold;

		  HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
				  12);

	  }

	  if (stopHand) {
		  stopHand = false;
		  arm.EmergencyStop();
		  un_send.params.lin = 0;
		  un_send.params.ang = 0;
		  un_send.params.hold = 10;
		  HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
				  12);
	  }

	  if (setZeroFlag) {
		  setZeroFlag = false;
		  arm.SetZeroEncoders();
//		  arm.SetSoftwareZero();
		  un_send.params.lin = 0;
		  un_send.params.ang = 0;
		  un_send.params.hold = 10;
		  HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
				  12);
	  }

	  HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
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
//
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
//
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
//
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, En2_Pin|Button_Pin|En1_Pin|Dir1_Pin
                          |Buser_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS2_Pin|Dir2_Pin|Led1_Pin
                          |S1_Pin|S2_Pin|En3_Pin|Dir3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : En2_Pin Button_Pin En1_Pin Dir1_Pin
                           Buser_Pin */
  GPIO_InitStruct.Pin = En2_Pin|Button_Pin|En1_Pin|Dir1_Pin
                          |Buser_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS2_Pin Dir2_Pin Led1_Pin
                           S1_Pin S2_Pin En3_Pin Dir3_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS2_Pin|Dir2_Pin|Led1_Pin
                          |S1_Pin|S2_Pin|En3_Pin|Dir3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EndCap1_Pin EndCap2_Pin EndCap3_Pin */
  GPIO_InitStruct.Pin = EndCap1_Pin|EndCap2_Pin|EndCap3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EndCap4_Pin */
  GPIO_InitStruct.Pin = EndCap4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EndCap4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//UNUSED(huart);
	if (huart == &huart1) {
//		if(!strcmp(rx_buffer,"TEXT")) {
//		}

//		uint8_t data[] = { '\\', 0x8f, 0xf8, 'B', 'q', '}', 0x16, 'C', 1, 1, 0, 0 };
		memcpy(un_get.bytes, rx_buffer, sizeof(rx_buffer));
		switch (un_get.params.hold) {
		case 0:
		case 1:
			startFirstMove = true;
			un_to.params.lin = un_get.params.lin;
			un_to.params.ang = un_get.params.ang;
			un_to.params.hold = un_get.params.hold;
//			arm.moveGripper = un_get.params.hold;
			break;
		case 25:
			stopHand = true;
			break;
		case 50:
			sendDataFlag = true;
			break;
		case 75:
			setZeroFlag = true;
			break;
		}
		memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	}
	HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == EndCap1_Pin) {
	    //gripper is fully opened, no move allowed
		allowMove = false;
		gripperMoveFinished = true;

		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_Base_Stop_IT(&htim3);
	  } else if (GPIO_Pin == EndCap2_Pin) {
		//gripper is fully closed, move allowed
		allowMove = true;
		gripperMoveFinished = true;
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_Base_Stop_IT(&htim3);
	  } else if (GPIO_Pin == EndCap3_Pin || GPIO_Pin == EndCap4_Pin) {
		  //linear motor is on the end of the hand, stop move
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
		  HAL_TIM_Base_Stop_IT(&htim2);
		  cntImpulse2 = 0;
		  arm.stateMoveM2 = false;
		  timerFT2 = true;
	  } else {
	      __NOP();
	  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM1)/*Проверяем от какого таймера пришёл CallBack тут надо проверить точность*/
	{
		cntImpulse1++;
		if (cntImpulse1 >= arm.anglePsteps) {

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
			arm.SetEnable(1, false);
			cntImpulse1 = 0;
			arm.stateMoveM1 = false;
			timerFT1 = true;
			if (startCorrectPos) {posCorrected = true;}
		}

	} else if (htim->Instance == TIM2) {

		cntImpulse2++;
		if (cntImpulse2 >= arm.distPsteps) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim2);
			arm.SetEnable(2, false);
			cntImpulse2 = 0;
			arm.stateMoveM2 = false;
			timerFT2 = true;
			if (startCorrectPos) {posCorrected = true;}
		}
	} else if (htim->Instance == TIM3) {
//		cntImpulse3++;
//		if (cntImpulse3 >= arm.gripperPsteps) {
//			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//			HAL_TIM_Base_Stop_IT(&htim3);
//			arm.SetEnable(3, false);
//			cntImpulse3 = 0;
//			arm.stateMoveM3 = false;
//
//			//TEMPORARY, should be removed when physical gripper with encaps will pass the tests
//			if (un_now.params.hold == 1) {
//				allowMove = false;
//				gripperMoveFinished = true;
//			} else {// if (un_now.params.hold == 0) {
//				allowMove = true;
//				gripperMoveFinished = true;
//			}
//		}
	}
  /* USER CODE END Callback 0 */
//  if (htim->Instance == TIM4) {
//    HAL_IncTick();
//  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == EndCap1_Pin) {
//	    //gripper is fully opened, no move allowed
//		allowMove = false;
//		gripperMoveFinished = true;
//	  } else if (GPIO_Pin == EndCap2_Pin) {
//		//gripper is fully closed, move allowed
//		allowMove = true;
//		gripperMoveFinished = true;
//	  } else if (GPIO_Pin == EndCap3_Pin || GPIO_Pin == EndCap4_Pin) {
//		  //linear motor is on the end of the hand, stop move
//		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		  HAL_TIM_Base_Stop_IT(&htim2);
//		  cntImpulse2 = 0;
//		  arm.stateMoveM2 = false;
//		  timerFT2 = true;
//	  } else {
//	      __NOP();
//	  }
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
