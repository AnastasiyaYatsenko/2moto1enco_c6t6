/*
 * RoboArm.h
 *
 *  Created on: Oct 3, 2023
 *      Author: admin
 */

#ifndef ROBOARM_H_
#define ROBOARM_H_

#include "stm32f1xx_hal.h"
#include <math.h>
#include "AMT22.h"
#include "TMC2209.h"
// #include "TMC2209.h"

#define drvMicroSteps 128
// #define drvMicroSteps 16
#define spoolStep 20
#define motorStep 200
#define beltRatio 2

#define encAmendment 0
// #define encAmendment 5461
// #define encAmendment 10923

class RoboArm{
public:
	float linearStepsMil = motorStep * drvMicroSteps / (beltRatio * spoolStep);
	int steps4OneMM = 200 * 128 / (2 * 20);
	// Settings for moto/rs
	TIM_HandleTypeDef *htim1M1;
	TIM_HandleTypeDef *htim2M2;
	TIM_HandleTypeDef *htim3M3;

	UART_HandleTypeDef *huartTmc;

	GPIO_TypeDef *Dir1_GPIO_Port_M1;
	uint16_t Dir1_Pin_M1;
	GPIO_TypeDef *Dir2_GPIO_Port_M2;
	uint16_t Dir2_Pin_M2;

	GPIO_TypeDef *Dir3_GPIO_Port_M3;
	uint16_t Dir3_Pin_M3;

	GPIO_TypeDef *En1_GPIO_Port_M1;
	uint16_t En1_Pin_M1;
	GPIO_TypeDef *En2_GPIO_Port_M2;
	uint16_t En2_Pin_M2;
	GPIO_TypeDef *En3_GPIO_Port_M3;
	uint16_t En3_Pin_M3;

	GPIO_TypeDef *EndCap1_GPIO_PortG;
	uint16_t EndCap1_PinG;
	GPIO_TypeDef *EndCap2_GPIO_PortG;
	uint16_t EndCap2_PinG;
	GPIO_TypeDef *EndCap3_GPIO_PortG;
	uint16_t EndCap3_PinG;
	GPIO_TypeDef *EndCap4_GPIO_PortG;
	uint16_t EndCap4_PinG;


	GPIO_TypeDef *Buser_GPIO_Port_Ind;
	uint16_t Buser_Pin_Ind;

	uint32_t distPsteps = 0, anglePsteps = 0;
	uint32_t gripperPsteps = 67000;
	float distMax = 210.0;
	float distMin = 48.0;

	// ENCODERS
	uint8_t ResolutionEncoders = 14;
	SPI_HandleTypeDef *arm_hspi1;
	uint16_t CS_Pin_Enc1;
	GPIO_TypeDef *CS_GPIO_Port_Enc1;
	uint16_t CS_Pin_Enc2;
	GPIO_TypeDef *CS_GPIO_Port_Enc2;

	uint32_t posNowEnc1, posNowEnc2;

	// TMC2209 drivers
		TMC2209 tmcd_linear;
		TMC2209 tmcd_angle;
		TMC2209 tmcd_gripper;

	// 124 мм лыныйне перемышення   добавить к линейному перемщению при сбросе
	// Записать последние данные в память флеш

	// entity
	float posNowAngle;
	uint16_t posNowDistance;

	int moveGripper;

	bool stateMoveM1 = false, stateMoveM2 = false, stateMoveM3= false;
	float defaultAngle, defaultDistanse; // стандартний кут //0 120 240 та дистанція 124 мм
	bool stateMovement[2];
	bool inverseLinZero = true;

	enum StateSystem{
	ArmSTART, //перший рух
	ArmStepSTART, //коли прийшли дані у вигляді кроків та періодів налаштували таймери
	ArmStepWaitMOVE, //таймери налаштовані, чекаємо старту руху по крокам
	ArmStepStartMOVE, //починаємо рух по крокам
	ArmStepMOVE, // рух йде по крокам, із цього переходимо
	ArmMOVE,  // зараз рух моторів
	ArmSTAND,  //стоїть нічого не робить
	ArmERROR,  // випав в помилку
	ArmGripMOVE, //руха зацепа зараз йде
	ArmGripENDMOVE,  //зацеп закінчив зачеплятись
	ArmGripPreMOVE,  //Рух зацепа НА початку руху перед перемішенням руки в точку (опустити якщо зачеплено)
	ArmGripPreENDMOVE, //зачеп було опущено після PRE руху
	ArmEndMOVE,        //ми доїхали
	ArmSTOP,			//рука в екстренному стопі
	ArmGetData, 		//запит даних
	ArmSetZero,			//встановлено сет зеро
	ArmGetVers, 		//запит версії
	};

	StateSystem State;

	RoboArm(float, float);
	int OpenGripper();				   // Open Gripper
	int CloseGripper();				   // Close Gripper
	int SetGripper(int);			   // Set Gripper
	int SetSettGripper(GPIO_TypeDef *EndCap1_GPIO_PortT, uint16_t EndCap1_PinT,
			GPIO_TypeDef *EndCap2_GPIO_PortT, uint16_t EndCap2_PinT,
			GPIO_TypeDef *EndCap3_GPIO_PortT, uint16_t EndCap3_PinT,
			GPIO_TypeDef *EndCap4_GPIO_PortT, uint16_t EndCap4_PinT);
	int GetGripperState();				//true - close / false - open
//	int GetLastPosition();			   // set last positions to encoder value
	int Move2Motors(float, float); // move 2 mottors simultaneously
	int Set2StepMotors(float,int, float, int);
	int Move2StepMotors(); //start timers
	int SetBuserState(int State);
//	int MoveLinear(float);
//	int MoveAngle(float);
	int MoveCorrectPosition(float angle, float distance);
	//	int SetMicrosteps(uint16_t microsteps_per_step); //set microsteps per step
	int SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
						GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
						GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
						uint8_t ResolutionEncodersT); // settings for encoders
	uint32_t GetPosEncoders(uint8_t);				  // get actually position encoders 1 or 2
	int SetZeroEncoders();							  // set zero position all encoders
	int SetSoftwareZero();							  // memorize current position as zero position
	float UnshiftZeroAng(float);					  // converts user angle into actual angle
	float UnshiftZeroLin(float);					  // converts user distance into actual distance
	float ShiftZeroAng(float);				  		  // converts actual angle into user angle
	float ShiftZeroLin(float);				  		  // converts actual distance into user distance
	float GetAngleEncoders(uint32_t);				  // get calculated Angle - pos value
	float GetLinEncoders(float);				  	  // get calculated linear pos value from angle
	uint32_t GetPosTactEncoders(uint32_t);			  // get calculated position


	float GetLin();				  	  				  // get calculated linear position
	float GetAng();				  	  				  // get calculated angle position

	int setPrintState(bool); // flag to send status to uart
	bool getPrintState();
	int EmergencyStop();
	int SetSettMotors(UART_HandleTypeDef &huartTmc, TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2, TIM_HandleTypeDef &htim3,
					  GPIO_TypeDef *Dir1_GPIO_Port_M1T, uint16_t Dir1_Pin_M1T,
					  GPIO_TypeDef *Dir2_GPIO_Port_M2T, uint16_t Dir2_Pin_M2T,
					  GPIO_TypeDef *Dir3_GPIO_Port_M3T, uint16_t Dir3_Pin_M3T,
					  GPIO_TypeDef *En1_GPIO_Port_M1T, uint16_t En1_Pin_M1T,
					  GPIO_TypeDef *En2_GPIO_Port_M2T, uint16_t En2_Pin_M2T,
					  GPIO_TypeDef *En3_GPIO_Port_M3T, uint16_t En3_Pin_M3T,
					  GPIO_TypeDef *Buser_GPIO_Port_IndT, uint16_t Buser_Pin_IndT);
	//			UART_HandleTypeDef &huart_tmcT);

	int SetMicrosteps4All(uint8_t);
	int SetEnable(uint16_t numMotor, bool state);
	int factoryReset();

private:
	bool PrintAllState = false;

	// Данные о текущем положении моторов
	float lastPosLinear_Enc = 0;
	float lastPosAngle_Enc = 0;

	float lastPosLinear_Set = 0;
	float lastPosAngle_Set = 0;

	bool lastPosGripper = false;

	// zero position info
	float ang_zero = 0;
	float lin_zero = 0;
};

#endif /* ROBOARM_H_ */
