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
uint8_t rx_data[28];
uint8_t rx_buffer[28];
uint8_t transfer_cplt = 0;
//TODO сокріш за все треба прибрати вже непотрібні флаги але такого інструменту тут немає))
//bool startFirstMove = false;
//bool sendDataFlag = false;
bool stopHand = false;
//bool setZeroFlag = false;

bool timerFT1 = false;
bool timerFT2 = false;
bool timerFT3 = false;

//bool startCorrectPos = false;
//bool posCorrected = false;

bool gripIntFlag = false;

//bool allowMove = true;
bool gripperMoveFinished = true;
//bool startMove = false;
bool moveFinished = false;

bool getVersion = false;

bool stepsSetFlagSent = false;

float accuracy = 0.05;

float recAngleF = 0.0;
uint16_t recDist = 0;
uint16_t recHold = 0;
bool flagReadEnc = 0;

int encMax = 16384;
int linMax = 210;

//десь внизу я видалив використання UN_NOW взагалі
union UN {
	struct {
		float lin;
		float ang;
		int PoT_lin;
		int PoT_ang;
		float lin_2;
		float ang_2;
		int hold;
	} params;
	uint8_t bytes[sizeof(params)];
} un_get, un_send, un_to, un_now;

float lin_beforeCorrect, ang_beforeCorrect;

void debounce_check_pins_and_set_flag();
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

uint32_t cntImpulse1 = 0, cntImpulse2 = 0, cntImpulse3 = 0, step1 = 0,
		step2 = 0;

//TODO version naming

//15 - додано щось

int version = 15;
RoboArm arm(240.0, 124.0);

//RoboArm arm(0, 124); - перша рука
//RoboArm arm(120, 124); - друга
//RoboArm arm(240, 124); - третя

//RoboArm arm(0, 124);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	arm.SetSettMotors(huart2, htim1, htim2, htim3, Dir1_GPIO_Port, Dir1_Pin,
	Dir2_GPIO_Port, Dir2_Pin, Dir3_GPIO_Port, Dir3_Pin, En1_GPIO_Port,
	En1_Pin, En2_GPIO_Port, En2_Pin, En3_GPIO_Port, En3_Pin,
	Buser_GPIO_Port, Buser_Pin);
	arm.SetSettEncoders(hspi1, CS1_GPIO_Port, CS1_Pin, CS2_GPIO_Port, CS2_Pin,
			14);

	//Додано функція
	arm.SetSettGripper(EndCap1_GPIO_Port, EndCap1_Pin, EndCap2_GPIO_Port,
	EndCap2_Pin, EndCap3_GPIO_Port, EndCap3_Pin, EndCap4_GPIO_Port,
	EndCap4_Pin);

//	un_now.params.hold = arm.GetGripperState();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//  HAL_UART_Receive_IT (&huart1, str, 1);
	HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
	arm.setPrintState(true);

	arm.SetBuserState(8);
//	arm.SetBuserState(4);

//В останне запишемо поточне положення зачепа
	arm.lastGripState = arm.GetGripperState();

	arm.State = arm.ArmSTAND;

	for (int i=0; i<3; i++){
		arm.GetLin();
		HAL_Delay(10);
		arm.GetAng();
	}

	arm.SetLinAngMicrostepsAndParams(4);

//	debounce_check_pins_and_set_flag();

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);
		if (error_flag) {
			for (int i = 0; i <= 200; i++) {
				HAL_GPIO_TogglePin(Buser_GPIO_Port, Buser_Pin);
				HAL_Delay(1);
			}
		}

		//!!!!!!!!!ЯКЩО зараз йде рух зацепа то ми постійно в цій умові перевіряємо стан зацепа і виставляємо флаг
		if (arm.State == arm.ArmGripPreMOVE || arm.State == arm.ArmGripMOVE || arm.State == arm.ArmGripMOVERetry) {

			debounce_check_pins_and_set_flag();

		}

		// +2 початок руху якщо прийшли нові дані
		if (arm.State == arm.ArmSTART) {
			arm.State = arm.ArmSTAND;
			//перевіряємо статус зацепа якщо він зачеплений то ОПУСКАЄМО (це не вірна логіка)
			//+1
			int tempGripState = arm.GetGripperState();
			//Зберігли попередній стан

			if (tempGripState == 1 && un_to.params.hold == 0) { //якщо піднятий +1.1 і треба опустити
//			if (un_to.params.hold == 0) { //якщо піднятий +1.1 і треба опустити
				arm.State = arm.ArmGripPreMOVE;
				arm.lastGripState = tempGripState; //записали поточне положеня
				arm.SetGripper(0);
			} else { //if (tempGripState == 0) { //якщо опущенний то можна далі рухатись
				//+3
				arm.State = arm.ArmGripPreENDMOVE;
			}
		}

		//обробка кількість кроків та періоди.
		if (arm.State == arm.ArmStepSTART) {
			arm.State = arm.ArmSTAND;
			arm.Set2StepMotors(un_to.params.lin, un_to.params.PoT_lin,
					un_to.params.ang, un_to.params.PoT_ang);
			arm.State = arm.ArmStepWaitMOVE;
		}

		if (arm.State == arm.ArmStepWaitMOVE) {
//			stepsSetFlagSent = true;
			arm.State = arm.ArmSTAND;
			un_send.params.lin = 0.0;
			un_send.params.ang = 0.0;
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;
			un_send.params.lin_2 = 0;
			un_send.params.ang_2 = 0;
			un_send.params.hold = 10;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
		}

		if (arm.State == arm.ArmStepStartMOVE) {
			arm.State = arm.ArmStepMOVE;
			//В останне запишемо поточне положення зачепа
			arm.lastGripState = arm.GetGripperState();

//			int tempGripState = arm.GetGripperState();
//
//			if (tempGripState == 1 && un_to.params.hold == 0) { //якщо піднятий +1.1 і треба опустити
//				arm.State = arm.ArmGripPreMOVEStep;
//				arm.lastGripState = tempGripState; //записали поточне положеня
//				arm.SetGripper(0);
//			} else { //if (tempGripState == 0) { //якщо опущенний то можна далі рухатись
//				//+3
//				arm.State = arm.ArmGripPreENDMOVEStep;
//			}

			arm.Move2StepMotors();
//			stepsSetFlagSent = false;
		}

		//+4 опустили якщо треба було або починаємо одночасний рух моторів.
		if (arm.State == arm.ArmGripPreENDMOVE) {
			arm.State = arm.ArmMOVE;
			arm.Move2Motors(un_to.params.ang_2, un_to.params.lin_2);
		}

		if (arm.State == arm.ArmGripPreENDMOVEStep) {
			arm.State = arm.ArmMOVE;
			arm.Move2StepMotors();
		}

		if (arm.anglePsteps == 0 && (arm.State == arm.ArmMOVE || arm.State == arm.ArmStepMOVE || arm.State == arm.ArmCorrectPosition)){
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
			cntImpulse1 = 0;
			timerFT1 = true;
		}

		if (arm.distPsteps == 0 && (arm.State == arm.ArmMOVE || arm.State == arm.ArmStepMOVE || arm.State == arm.ArmCorrectPosition)){
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim2);
			cntImpulse2 = 0;
			timerFT2 = true;
		}

		if (timerFT1 && timerFT2 && arm.State == arm.ArmMOVE) {
			timerFT1 = false;
			timerFT2 = false;
			arm.State = arm.ArmCorrectPosition;

			lin_beforeCorrect = arm.GetLin();
//			un_send.params.lin = 0;
//			un_send.params.lin_2 = lin;
			HAL_Delay(10);
			ang_beforeCorrect = arm.GetAng();
//			un_send.params.ang = 0;
//			un_send.params.ang_2 = ang;
//			un_send.params.hold = arm.GetGripperState()+10;
//			un_send.params.PoT_lin = 0;
//			un_send.params.PoT_ang = 0;
//			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),12);
//			HAL_Delay(1);

//			arm.drvMicroSteps = 128;
//			arm.gripperPsteps = 523*arm.drvMicroSteps;
//			arm.steps4OneMM = motorStep * arm.drvMicroSteps / (beltRatio * spoolStep);

			arm.SetLinAngMicrostepsAndParams(7);

			// TODO change microsteps to 128
			// UPDATE VALUES FOR ARM, LIKE STEPS FOR 1 MM
//			arm.SetBuserState(2);
			arm.Move2Motors(un_to.params.ang_2, un_to.params.lin_2);
		}



		//+5 обидва мотори доїхали по статусам в таймерах
		if (timerFT1 && timerFT2 && arm.State == arm.ArmStepMOVE) {
			timerFT1 = false;
			timerFT2 = false;
			arm.State = arm.ArmCorrectPosition;

			lin_beforeCorrect = arm.GetLin();
			HAL_Delay(10);
			ang_beforeCorrect = arm.GetAng();

			arm.SetLinAngMicrostepsAndParams(7);

			arm.Move2Motors(un_to.params.ang_2, un_to.params.lin_2);
		}
		if (timerFT1 && timerFT2	&& arm.State == arm.ArmCorrectPosition) {
//			arm.SetBuserState(3);
			// TODO change microsteps to 32/16
			// UPDATE VALUES FOR ARM, LIKE STEPS FOR 1 MM
			timerFT1 = false;
			timerFT2 = false;
//			arm.SetLinAngMicrostepsAndParams(4);
			arm.State = arm.ArmGripPermit;
//			arm.drvMicroSteps = 16;
//			arm.gripperPsteps = 523*arm.drvMicroSteps;
//			arm.steps4OneMM = motorStep * arm.drvMicroSteps / (beltRatio * spoolStep);
			arm.SetLinAngMicrostepsAndParams(4);
		}

		if (arm.State == arm.ArmGripPermit) {
			//+6 Перевірка статуса зацепа чи він не посередині і встановлюємо потрібний опускаємо
//			arm.SetBuserState(1);
			int tempGripState = arm.GetGripperState();
			if ((tempGripState == 1 || tempGripState == 0)
					&& (tempGripState != un_to.params.hold)) {

				arm.lastGripState = tempGripState; //записали поточне положеня

				arm.State = arm.ArmGripMOVE;
				arm.SetGripper(un_to.params.hold);
			} else if (tempGripState == un_to.params.hold) {
				arm.State = arm.ArmGripENDMOVE;
			} else if (tempGripState == 3) {
				arm.State = arm.ArmGripMOVEError;  //!!!!!!!!!!!!!!!! УВАГА ТУТ
			}
		}

		//якщо кроки закінчились а кінцевік не спрацював
		if (arm.State == arm.ArmGripMOVEError) {
			arm.State = arm.ArmGripMOVERetry;
			//їдемо в протилежну сторону
			arm.SetGripper(0); // TODO to last stable position

//			if (arm.lastGripState == 1) {
//				arm.SetGripper(1);
//			} else {//if (arm.lastGripState == 0) {
//				arm.SetGripper(0);
//			}
		}

//		if (arm.GetGripperState()==un_to.params.hold && arm.State = arm.ArmGripENDMOVE){
//			arm.State = arm.ArmGripENDMOVE;
//		}

		// +7 закінчили рух зацепа
		if (arm.State == arm.ArmGripENDMOVE) {
			arm.State = arm.ArmSTAND;
			float lin = arm.GetLin();
//			un_send.params.lin = 0;
			un_send.params.lin = lin_beforeCorrect;
			un_send.params.lin_2 = lin;
			HAL_Delay(10);
			float ang = arm.GetAng();
//			un_send.params.ang = 0;
			un_send.params.ang = ang_beforeCorrect;
			un_send.params.ang_2 = ang;
			un_send.params.hold = arm.GetGripperState()+10;
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;

			gripperMoveFinished = false;
			moveFinished = false;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),12);

		}

		//запит на читання координат
		if (arm.getPrintState() && arm.State == arm.ArmGetData) {
			arm.State = arm.ArmSTAND;
			float lin = arm.GetLin();
			un_send.params.lin = 0;
			un_send.params.lin_2 = lin;
//		  un_send.params.lin = arm.ShiftZeroLin(lin); //це для АМТ223С-V
			HAL_Delay(10);
			float ang = arm.GetAng();
			un_send.params.ang = 0;
			un_send.params.ang_2 = ang;
//		  un_send.params.ang = arm.ShiftZeroAng(ang); //це для АМТ223С-V
			un_send.params.hold = arm.GetGripperState();
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
		}

		//екстренна зупинка
		if (arm.State == arm.ArmSTOP) {
			stopHand = false;
			arm.EmergencyStop();

			//відправляємо "все ок" до малини
			un_send.params.lin = 0;
			un_send.params.ang = 0;
			un_send.params.lin_2 = 0;
			un_send.params.ang_2 = 0;
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;
			un_send.params.hold = 10;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
			arm.State = arm.ArmSTAND;
		}

		//встановлення нуля
		if (arm.State == arm.ArmSetZero) {
			arm.State = arm.ArmSTAND;
			arm.SetZeroEncoders();
//		  arm.SetSoftwareZero();
			//відправляємо "все ок" до малини
			un_send.params.lin = 0;
			un_send.params.ang = 0;
			un_send.params.lin_2 = 0;
			un_send.params.ang_2 = 0;
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;
			un_send.params.lin_2 = 0;
			un_send.params.ang_2 = 0;
			un_send.params.hold = 10;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);

//			arm.State = arm.ArmSTAND;
		}

		if (arm.State == arm.ArmGetVers) {
			arm.State = arm.ArmSTAND;
			un_send.params.lin = 0;
			un_send.params.ang = 0;
			un_send.params.lin_2 = 0;
			un_send.params.ang_2 = 0;
			un_send.params.PoT_lin = 0;
			un_send.params.PoT_ang = 0;
			un_send.params.hold = version;
			HAL_UART_Transmit(&huart1, un_send.bytes, sizeof(un_send.bytes),
					12);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief SPI1 Initialization Function
 * @param None HAL_Delay(1);
 }
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
	HAL_GPIO_WritePin(GPIOA,
	En2_Pin | Button_Pin | En1_Pin | Dir1_Pin | Buser_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			CS1_Pin | CS2_Pin | Dir2_Pin | Led1_Pin | S1_Pin | S2_Pin | En3_Pin
					| Dir3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : Led_Pin */
	GPIO_InitStruct.Pin = Led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : En2_Pin Button_Pin En1_Pin Dir1_Pin
	 Buser_Pin */
	GPIO_InitStruct.Pin = En2_Pin | Button_Pin | En1_Pin | Dir1_Pin | Buser_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : CS1_Pin CS2_Pin Dir2_Pin Led1_Pin
	 S1_Pin S2_Pin En3_Pin Dir3_Pin */
	GPIO_InitStruct.Pin = CS1_Pin | CS2_Pin | Dir2_Pin | Led1_Pin | S1_Pin
			| S2_Pin | En3_Pin | Dir3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : EndCap1_Pin EndCap2_Pin EndCap3_Pin */
	GPIO_InitStruct.Pin = EndCap1_Pin | EndCap2_Pin | EndCap3_Pin;
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
		// копіюємо отримані дані у rx_buffer
		memcpy(un_get.bytes, rx_buffer, sizeof(rx_buffer));

		switch (un_get.params.hold) {
		case 0:
		case 1:
			// 0 або 1 у un_get.params.hold = прийшли нові координати
			//startFirstMove = true;

			//статус початку руху  +1
			arm.State = arm.ArmSTART;

			un_to.params.lin = un_get.params.lin;
			un_to.params.ang = un_get.params.ang;
			un_to.params.PoT_ang = 0;
			un_to.params.PoT_lin = 0;
			un_to.params.lin_2 = un_get.params.lin_2;
			un_to.params.ang_2 = un_get.params.ang_2;
			un_to.params.hold = un_get.params.hold;
//			arm.moveGripper = un_get.params.hold;
			break;

		case 25:
			//25 = екстренна зупинка

			arm.State = arm.ArmSTOP;
			//оце би на переривання якесь повісити, щоб якщо натиснули в дашборді - то остаточно вирубати.
			break;
		case 50:
			//50 = get-запит
			//sendDataFlag = true;
			arm.State = arm.ArmGetData;
			break;
		case 75:
			//75 = встановлення нуля
			//	setZeroFlag = true;
			arm.State = arm.ArmSetZero;
			break;
		case 80:
			arm.State = arm.ArmGetVers;
			break;

		case 31: //так робити погано але ладна)
		case 30:
			arm.State = arm.ArmStepSTART;
			un_to.params.lin = un_get.params.lin; //це кроки 1
			un_to.params.ang = un_get.params.ang; //це кроки 2
			un_to.params.PoT_ang = un_get.params.PoT_ang; //це період 2
			un_to.params.PoT_lin = un_get.params.PoT_lin; //це період 1
			un_to.params.lin_2 = un_get.params.lin_2;
			un_to.params.ang_2 = un_get.params.ang_2;
			un_to.params.hold = un_get.params.hold % 10; //парсимо hold 0 або 1

			break;

		case 40:
			//перевірка чи були налаштовані таймери до цього для руху по крокам
			arm.State = arm.ArmStepStartMOVE;

			break;

		}
		memset(rx_buffer, 0, sizeof(rx_buffer));
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	}
	HAL_UART_Receive_IT(&huart1, rx_buffer, sizeof(rx_buffer));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//Тут не використовується, треба почитати як без таймера не затримуючі переривання чимось зупинятись
	//можливо встановлювати флаг на перевірку статусу функцією debounce_check_pins_and_set_flag();
	if (GPIO_Pin == EndCap1_Pin) { //натиснутий нижній
//
//		HAL_Delay(1);
//
//		if (HAL_GPIO_ReadPin(arm.EndCap1_GPIO_PortG, arm.EndCap1_PinG) == GPIO_PIN_SET) {
//		            gripIntFlag = true;
//		}

	} else if (GPIO_Pin == EndCap2_Pin) {
//
//		HAL_Delay(1);
//
//		if(HAL_GPIO_ReadPin(arm.EndCap2_GPIO_PortG, arm.EndCap2_PinG) == GPIO_PIN_SET) {
//			gripIntFlag = true;
//		}

//		cntImpulse3= 0;
	} else if (GPIO_Pin == EndCap3_Pin || GPIO_Pin == EndCap4_Pin) {
		//лінійний мотор утикнувся в край руки, зупиняємо його рух
		//зупиняємо ШІМ на другому таймері
//		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
//		HAL_TIM_Base_Stop_IT(&htim2);
		//зануляємо лічильник імпульсів
		//	cntImpulse2 = 0;
		//вважаємо, що мотор доїхав до необхідної позиції
		//	arm.stateMoveM2 = false;
		//	timerFT2 = true;
	}
//	else {
//		__NOP();
//	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (huart->ErrorCode == HAL_UART_ERROR_ORE) {
			// переполнение
			arm.SetBuserState(2);
		}
		if (huart->ErrorCode == HAL_UART_ERROR_PE) {
			// Ошибка четности
			arm.SetBuserState(3);
		}
		if (huart->ErrorCode == HAL_UART_ERROR_NE) {
			// Ошибка зашумление
			arm.SetBuserState(4);
		}
		if (huart->ErrorCode == HAL_UART_ERROR_FE) {
			// Ошибка кадрирования
			arm.SetBuserState(5);
		}
	}
}

void debounce_check_pins_and_set_flag() {
	static uint32_t last_check_time = 0;
	static uint32_t debounce_delay = 15; // Затримка для віднімання дребезгу, в мілісекундах, насправді дребезг всього 120 мікросекунд!!!!
	static bool checkFlag1 = false;
	static bool checkFlag2 = false;
//	static uint8_t last_EndCap1_state = GPIO_PIN_RESET;
//	static uint8_t last_EndCap2_state = GPIO_PIN_RESET;
	static uint8_t last_EndCap1_state = HAL_GPIO_ReadPin(arm.EndCap1_GPIO_PortG,
			arm.EndCap1_PinG);
	static uint8_t last_EndCap2_state = HAL_GPIO_ReadPin(arm.EndCap2_GPIO_PortG,
			arm.EndCap2_PinG);
	uint32_t current_time = HAL_GetTick();

	// Перевірка чи пройшла достатня затримка для уникнення дребезгу
	if (current_time - last_check_time >= debounce_delay) {
		// Оновлення часу останньої перевірки
		last_check_time = current_time;

		// Перевірка стану піну EndCap1
		uint8_t current_EndCap1_state = HAL_GPIO_ReadPin(arm.EndCap1_GPIO_PortG,
				arm.EndCap1_PinG);

		if (current_EndCap1_state == GPIO_PIN_SET
				&& last_EndCap1_state == GPIO_PIN_RESET) {
			if (checkFlag1) {
				checkFlag1 = false;
				gripIntFlag = true;
//				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				HAL_GPIO_TogglePin(arm.Buser_GPIO_Port_Ind, arm.Buser_Pin_Ind);
			} else {
				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				checkFlag1 = true;
			}
		} else {
			if (checkFlag1) {
				checkFlag1 = false;
			}
		}
		if (!checkFlag1) {
			last_EndCap1_state = current_EndCap1_state;
		}

		// Перевірка стану піну EndCap2
		uint8_t current_EndCap2_state = HAL_GPIO_ReadPin(arm.EndCap2_GPIO_PortG,
				arm.EndCap2_PinG);
//		if (current_EndCap2_state == GPIO_PIN_SET
//				&& last_EndCap2_state == GPIO_PIN_RESET) {
//			gripIntFlag = true;
//		}
//		last_EndCap2_state = current_EndCap2_state;
		if (current_EndCap2_state == GPIO_PIN_SET
				&& last_EndCap2_state == GPIO_PIN_RESET) {
			if (checkFlag2) {
				checkFlag2 = false;
				gripIntFlag = true;
//				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				HAL_GPIO_TogglePin(arm.Buser_GPIO_Port_Ind, arm.Buser_Pin_Ind);
			} else {
				HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
				checkFlag2 = true;
			}
		} else {
			if (checkFlag2) {
				checkFlag2 = false;
			}
		}
		if (!checkFlag2) {
			last_EndCap2_state = current_EndCap2_state;
		}
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	if (htim->Instance == TIM1)/*Проверяем от какого таймера пришёл CallBack тут надо проверить точность*/
	{
		cntImpulse1++;
		if (cntImpulse1 >= arm.anglePsteps) {

			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim1);
//			arm.SetEnable(1, false);
			// COMMENT 10
//			arm.SetEnable(1, true);
			cntImpulse1 = 0;
			timerFT1 = true;

			//перевірка: якщо була докатка, виставляємо, що ми її відпрацювали
			//if (startCorrectPos) {posCorrected = true;}
		}

	} else if (htim->Instance == TIM2) {

		cntImpulse2++;
		if (cntImpulse2 >= arm.distPsteps) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim2);
//			arm.SetEnable(2, false);
			// COMMENT 11
//			arm.SetEnable(2, true);
			cntImpulse2 = 0;
			timerFT2 = true;

			//перевірка: якщо була докатка, виставляємо, що ми її відпрацювали
			//if (startCorrectPos) {posCorrected = true;}
		}
	} else if (htim->Instance == TIM3) {

		cntImpulse3++;

		// якщо кроки закінчилися але кінцевік не спрацював, то зупиняємось.
		if (cntImpulse3 >= arm.gripperPsteps && gripIntFlag == false) {
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim3);
			arm.State = arm.ArmGripMOVEError;
		} else if (cntImpulse3 >= arm.gripperPsteps || gripIntFlag == true) {
			gripIntFlag = false;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim3);

			if (arm.State == arm.ArmGripPreMOVE) {
				arm.State = arm.ArmGripPreENDMOVE;
			} else if (arm.State == arm.ArmGripPreMOVEStep) {
				arm.State = arm.ArmGripPreENDMOVEStep;
			} else if (arm.State == arm.ArmGripMOVE) {
				arm.State = arm.ArmGripENDMOVE;
			} else if (arm.State == arm.ArmGripMOVERetry) {
				arm.State = arm.ArmGripPermit;
			}

			cntImpulse3 = 0;
			timerFT3 = true;
		}
//		if (cntImpulse3 > arm.gripperPsteps) {
//			arm.SetBuserState(1);
//			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
//			HAL_TIM_Base_Stop_IT(&htim3);
//			arm.State = arm.ArmGripMOVEError;
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	//повинен пропищати при помилці; треба перевірити
	for (int t = 0; t <= 4; t++) {
		for (int i = 0; i <= 200; i++) {
			HAL_GPIO_TogglePin(Buser_GPIO_Port, Buser_Pin);
			HAL_Delay(1);
		}
		HAL_Delay(100);
	}
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
