#include "RoboArm.h"
//#include "TMC2209.h"

RoboArm::RoboArm(float defaultAngleT, float defaultDistanseT) {
	defaultAngle = defaultAngleT;
	defaultDistanse = defaultDistanseT;

	startDWT();
}

int RoboArm::CloseGripper() {
	return 0;
}

int RoboArm::EmergencyStop() {

	HAL_GPIO_WritePin(En1_GPIO_Port_M1, En1_Pin_M1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(En2_GPIO_Port_M2, En2_Pin_M2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(En3_GPIO_Port_M3, En3_Pin_M3, GPIO_PIN_SET);

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htim3M3, TIM_CHANNEL_3);

	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);
	HAL_TIM_Base_Stop_IT(htim3M3);

	return 0;
}


int RoboArm::MoveCorrectPosition(float angle, float distance) {

	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim1M1);			// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT(htim2M2);

//	GetLastPosition(); //update -> lastPosAngle lastPosLinear from ENCODER
	lastPosAngle_Enc = GetAng();
	lastPosLinear_Enc = GetLin();

	float lastPosAngle = lastPosAngle_Enc;
	float lastPosLinear = lastPosLinear_Enc;

	float pos_ang = abs(lastPosAngle - angle);
	float inverse_pos_ang = abs(360.0 - pos_ang);
	float actualPosAngle;

	/* виставили в яку сторону ехать мотору*/
	if (inverse_pos_ang < pos_ang) {
		actualPosAngle = inverse_pos_ang;
		if (lastPosAngle < angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
			tmcd_angle.enableInverseMotorDirection();
		} else if (lastPosAngle > angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);

			tmcd_angle.disableInverseMotorDirection();
		}
	} else {
		actualPosAngle = pos_ang;
		if (lastPosAngle < angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
			tmcd_angle.disableInverseMotorDirection();
		} else if (lastPosAngle > angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
			tmcd_angle.enableInverseMotorDirection();
		}
	}

	if (lastPosLinear < distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
		tmcd_linear.enableInverseMotorDirection();
	} else if (lastPosLinear > distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
		tmcd_linear.disableInverseMotorDirection();
	}

	float actualPosDistance = abs(lastPosLinear - distance);

	//set microstepping
	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps

	distPsteps = actualPosDistance * steps4OneMM; //кількість кроків для такої дистанції

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

//	uint32_t periodM1 = 1200; //reduce to 600-400 mks for 32 microsteps
	uint32_t periodM1 = 30;
	uint32_t psc = 72 - 1;

	float delimiter = 1;
	float mnoj = 1;

	if (anglePsteps >= distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR1 = periodM1 / 2;

		delimiter = float(anglePsteps) / float(distPsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR2 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR2 = periodM1 / 2;

		delimiter = float(distPsteps) / float(anglePsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR1 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;



	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);


	State == ArmGripPermit;

	return 0;
}

int steps4OneMM = 200 * 128 / (2 * 20);

//float DistLast=0,DistLastT=0,DistNow=0;

//float distFenc(float encDt){
//
//    //0........16384
//
//    float angleEncoder = (encDt * 360.0 ) / 16384;  //кут енкодера
//    float distPsteps = (angleEncoder * 6.4516129 * 200 * 128) / 360;  //Кроки із кута
////    std::cout<<"== Поточна дистанція по енкодеру: "<<distPsteps/steps4OneMM<<" mm"<<std::endl; //поточна дистанція по енкодеру
//
//     return distPsteps/steps4OneMM;
// }

int RoboArm::Move2Motors(float angle, float distance) {

	// TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
	// TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim1M1);			// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT(htim2M2);

	// COMMENT 4
//	SetEnable(1, false);
//	SetEnable(2, false);

//	GetLastPosition(); //update -> lastPosAngle lastPosLinear from ENCODER
	lastPosAngle_Enc = GetAng();
	lastPosLinear_Enc = GetLin();

	float lastPosAngle = lastPosAngle_Enc;
	float lastPosLinear = lastPosLinear_Enc;

	float pos_ang = abs(lastPosAngle - angle);
	float inverse_pos_ang = abs(360.0 - pos_ang);
	float actualPosAngle;

	/* виставили в яку сторону ехать мотору*/
	if (inverse_pos_ang < pos_ang) {
		actualPosAngle = inverse_pos_ang;
		if (lastPosAngle < angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
			tmcd_angle.enableInverseMotorDirection();
		} else {//if (lastPosAngle > angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);

			tmcd_angle.disableInverseMotorDirection();
		}
	} else {
		actualPosAngle = pos_ang;
		if (lastPosAngle < angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
			tmcd_angle.disableInverseMotorDirection();
		} else {//if (lastPosAngle > angle) {
			//	HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
			tmcd_angle.enableInverseMotorDirection();
		}
	}

	if (lastPosLinear < distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
		tmcd_linear.enableInverseMotorDirection();
	} else {//if (lastPosLinear > distance) {
//			HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
		tmcd_linear.disableInverseMotorDirection();
	}

	float actualPosDistance = abs(lastPosLinear - distance);

	//set microstepping
	anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps

	distPsteps = actualPosDistance * steps4OneMM; //кількість кроків для такої дистанції

// 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

//	uint32_t periodM1 = 1200; //reduce to 600-400 mks for 32 microsteps
	uint32_t periodM1 = 30;
	if (State == ArmCorrectPosition){
		periodM1 = 600;
	}
	uint32_t psc = 72 - 1;

	float delimiter = 1;
	float mnoj = 1;

	if (anglePsteps >= distPsteps) {

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = periodM1;
		htim1M1->Instance->CCR1 = periodM1 / 2;

		delimiter = float(anglePsteps) / float(distPsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = mnoj;
		htim2M2->Instance->CCR2 = mnoj / 2;

	} else if (anglePsteps < distPsteps) {

		htim2M2->Instance->PSC = psc;
		htim2M2->Instance->ARR = periodM1;
		htim2M2->Instance->CCR2 = periodM1 / 2;

		delimiter = float(distPsteps) / float(anglePsteps);
		mnoj = ceil(periodM1 * delimiter);

		htim1M1->Instance->PSC = psc;
		htim1M1->Instance->ARR = mnoj;
		htim1M1->Instance->CCR1 = mnoj / 2;
	}

	stateMoveM1 = true;
	stateMoveM2 = true;

	// COMMENT 5
//	SetEnable(1, true);
//	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	return 0;
}

int RoboArm::Set2StepMotors(float stepLinT, int periodLinT, float stepAngleT,
		int periodAngleT) {

//	float trunc_val = stepAngleT;
//	float temp_val = stepAngleT;
//	if (stepAngleT < 0) {
//		temp_val = -1.0*stepAngleT;
//	}


	anglePsteps=abs(stepAngleT);
	distPsteps=abs(stepLinT);

	uint32_t psc = 72 - 1;

	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT (htim1M1);			// остановили прерывание таймеров
	HAL_TIM_Base_Stop_IT (htim2M2);

	//COMMENT 6
//	SetEnable(1, false);
//	SetEnable(2, false);

	float tempAngDir = 0, tempLinDir = 0;
	if (stepAngleT < 0) {
		tmcd_angle.enableInverseMotorDirection();
//		tempAngDir = -1;
	} else {//if (stepAngleT > 0) {
		tmcd_angle.disableInverseMotorDirection();
//		tempAngDir = 1;
	}

	if (stepLinT < 0) {
		tmcd_linear.enableInverseMotorDirection();
//		tempLinDir = -1;
	} else {//if (stepLinT > 0) {
		tmcd_linear.disableInverseMotorDirection();
//		tempLinDir = 1;
	}

	if (periodAngleT % 2 != 0) { periodAngleT += 1;}
	if (periodLinT % 2 != 0) { periodLinT += 1;}

	htim1M1->Instance->PSC = psc;
	htim1M1->Instance->ARR = periodAngleT;
	htim1M1->Instance->CCR1 = periodAngleT / 2;

	htim2M2->Instance->PSC = psc;
	htim2M2->Instance->ARR = periodLinT;
	htim2M2->Instance->CCR2 = periodLinT / 2;


	return 0;
}

int RoboArm::Move2StepMotors(){

	stateMoveM1 = true;
	stateMoveM2 = true;

	// COMMENT 7
//	SetEnable(1, true);
//	SetEnable(2, true);

	HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim1M1);
	HAL_TIM_Base_Start_IT(htim2M2);

	return 0;
}



int RoboArm::Move2CGripRollon(){
//відкат зацепа в протилежну сторону



return 0;
}



int RoboArm::SetBuserState(int State) {
	//State 1......9
	if (State > 0 && State < 10) {
		for (int t = 0; t <= State; t++) {
			for (int i = 0; i <= 200; i++) {
				HAL_GPIO_TogglePin(Buser_GPIO_Port_Ind, Buser_Pin_Ind);
				HAL_Delay(1);
			}
			HAL_Delay(100);
		}
	}
	return 0;
}

//int MoveLinear(float Dist) {
//
//
//
//	return 0;
//}
//int MoveAngle(float Angl){
//
//
//
//	return 0;
//}

/*

 int RoboArm::Move2MotorsSimu(float angle, uint16_t distance) {

 // TIM1 Х  enc1 -  угол 360  -  8 оборотов движка на 1 оборот энкодера
 // TIM2  Y  enc2 - линейный -  6,4516129 оборотов движка (это целое линейное перемещение с запасом) на 1 оборот энкодера

 HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);      //остановили PWM таймера
 HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);
 HAL_TIM_Base_Stop_IT(htim1M1);				// остановили прерывание таймеров
 HAL_TIM_Base_Stop_IT(htim2M2);

 SetEnable(1, false);
 SetEnable(2, false);

 GetLastPosition();


 if (lastPosAngle < angle) {
 HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_SET);
 } else if (lastPosAngle > angle) {
 HAL_GPIO_WritePin(Dir1_GPIO_Port_M1, Dir1_Pin_M1, GPIO_PIN_RESET);
 }
 if (lastPosLinear < distance) {
 HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_SET);
 } else if (lastPosLinear > distance) {
 HAL_GPIO_WritePin(Dir2_GPIO_Port_M2, Dir2_Pin_M2, GPIO_PIN_RESET);
 }

 float actualPosAngle = abs(lastPosAngle - angle);
 float actualPosDistance = abs(lastPosLinear - distance);

 //set microstepping
 anglePsteps = (actualPosAngle * (8 * motorStep * drvMicroSteps)) / 360; //angle to steps
 distPsteps = actualPosDistance * linearStepsMil; //steps to distanse

 float distPangle = ((distPsteps / (motorStep * drvMicroSteps)) * 360
 / 6.45);

 //	lastPosAngle = angle;
 //	lastPosLinear = distance;

 // 1, 2, 3, 4, 6, 8, 9, 12, 18, 24, 36 и 72 - Це можлива обрана максимальна швидкість для мотора з більшої кількістю кроків. Це дільник таймера

 float periodM1 = 1200;
 uint32_t psc = 72-1;

 float delimiter=1;
 float mnoj=1;

 if (anglePsteps > distPsteps) {

 htim1M1->Instance->PSC = psc;
 htim1M1->Instance->ARR = periodM1;
 htim1M1->Instance->CCR1 = periodM1/2;

 delimiter = anglePsteps / distPsteps;
 mnoj = ceil(periodM1 * delimiter);

 htim2M2->Instance->PSC = psc;
 htim2M2->Instance->ARR = mnoj;
 htim2M2->Instance->CCR2 = mnoj / 2;

 } else if (anglePsteps < distPsteps) {

 htim2M2->Instance->PSC = psc;
 htim2M2->Instance->ARR = periodM1;
 htim2M2->Instance->CCR2 = periodM1 / 2;

 delimiter = distPsteps / anglePsteps;
 mnoj = ceil(periodM1 * delimiter);

 htim1M1->Instance->PSC = psc;
 htim1M1->Instance->ARR = mnoj;
 htim1M1->Instance->CCR1 = mnoj / 2;
 }

 stateMoveM1 = true;
 stateMoveM2 = true;

 SetEnable(1, true);
 SetEnable(2, true);

 HAL_TIM_PWM_Start(htim1M1, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(htim2M2, TIM_CHANNEL_2);
 HAL_TIM_Base_Start_IT(htim1M1);
 HAL_TIM_Base_Start_IT(htim2M2);

 return 0;
 }
 */

int RoboArm::factoryReset() {
	SetZeroEncoders();
//	SetSoftwareZero();
	posNowAngle = defaultAngle;
	posNowDistance = defaultDistanse;

	return 0;
}

int RoboArm::GetGripperState() {
	//1 = UP
	//2 = DOWN
	//3 = ERROR

	//сюди додати цикл х3 перевірки
	//EndCap1_Pin - зацеп зацеплен - ВЕРХНИЙ НАЖАт
	//EndCap2_Pin - зацеп расцеплен, можно двигать рукой - НИЖНИЙ

	GPIO_PinState pin_UP, pin_DOWN;

	pin_UP = HAL_GPIO_ReadPin(EndCap1_GPIO_PortG, EndCap1_PinG);
	pin_DOWN = HAL_GPIO_ReadPin(EndCap2_GPIO_PortG, EndCap2_PinG);

	if (pin_UP == GPIO_PIN_SET && pin_DOWN == GPIO_PIN_RESET) {
		return 1;
	} else if (pin_DOWN == GPIO_PIN_SET && pin_UP == GPIO_PIN_RESET) {
		return 0;
	} else {
		return 3; //error
	}
}

int RoboArm::SetSettGripper(GPIO_TypeDef *EndCap1_GPIO_PortT,
		uint16_t EndCap1_PinT, GPIO_TypeDef *EndCap2_GPIO_PortT,
		uint16_t EndCap2_PinT, GPIO_TypeDef *EndCap3_GPIO_PortT,
		uint16_t EndCap3_PinT, GPIO_TypeDef *EndCap4_GPIO_PortT,
		uint16_t EndCap4_PinT) {

	EndCap1_GPIO_PortG = EndCap1_GPIO_PortT;
	EndCap1_PinG = EndCap1_PinT;
	EndCap2_GPIO_PortG = EndCap2_GPIO_PortT;
	EndCap2_PinG = EndCap2_PinT;
	EndCap3_GPIO_PortG = EndCap3_GPIO_PortT;
	EndCap3_PinG = EndCap3_PinT;
	EndCap4_GPIO_PortG = EndCap4_GPIO_PortT;
	EndCap4_PinG = EndCap4_PinT;

	return 0;
}

int RoboArm::SetGripper(int opcl) {

	//Встановити захват
	//TODO додати перевірку чи точно зупинились мотори
	HAL_TIM_PWM_Stop(htim1M1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htim2M2, TIM_CHANNEL_2);

	HAL_TIM_Base_Stop_IT(htim1M1);
	HAL_TIM_Base_Stop_IT(htim2M2);

	HAL_TIM_PWM_Stop(htim3M3, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(htim3M3);

	// COMMENT 8
//	SetEnable(3, false);

	//Зупинили все, 1 та 2й на утриманні
	//Обираэмо напрям
	if (opcl == 1) {
		HAL_GPIO_WritePin(Dir3_GPIO_Port_M3, Dir3_Pin_M3, GPIO_PIN_SET);
	} else if (opcl == 0) {
		HAL_GPIO_WritePin(Dir3_GPIO_Port_M3, Dir3_Pin_M3, GPIO_PIN_RESET);
	}

	//Підібрати кроки та швидкість
	float periodM3 = 30.00;
	uint32_t psc3 = 72 - 1;
	htim3M3->Instance->PSC = psc3;
	htim3M3->Instance->ARR = periodM3;
	htim3M3->Instance->CCR2 = periodM3 / 2;

	// COMMENT 9
//	SetEnable(3, true);
	HAL_TIM_PWM_Start(htim3M3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(htim3M3);
	return 0;
}

int RoboArm::setPrintState(bool state) {
	if (state) {
		PrintAllState = true;
	} else {
		PrintAllState = false;
	}
	return 0;
}

bool RoboArm::getPrintState() {
	if (PrintAllState) {
		return true;
	} else {
		return false;
	}
}

int RoboArm::SetMicrosteps4All(uint8_t microsteps_per_step) {

	tmcd_angle.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
	tmcd_gripper.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
	tmcd_linear.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);

	return 0;
}

int RoboArm::SetLinAngMicrostepsAndParams(uint8_t microsteps_per_step) {

	tmcd_angle.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
//	tmcd_gripper.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);
	tmcd_linear.setMicrostepsPerStepPowerOfTwo(microsteps_per_step);

	int steps = 1;
	for (int i=0; i<microsteps_per_step; i++){
		steps *= 2;
	}

	drvMicroSteps = steps;
//	gripperPsteps = 523*arm.drvMicroSteps;
	steps4OneMM = motorStep * drvMicroSteps / (beltRatio * spoolStep);

	return 0;
}

int RoboArm::SetSettEncoders(SPI_HandleTypeDef &arm_hspi1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc1T, uint16_t CS_Pin_Enc1T,
		GPIO_TypeDef *CS_GPIO_Port_Enc2T, uint16_t CS_Pin_Enc2T,
		uint8_t ResolutionEncodersT) {

	arm_hspi1 = &arm_hspi1T;
	CS_GPIO_Port_Enc1 = CS_GPIO_Port_Enc1T;
	CS_Pin_Enc1 = CS_Pin_Enc1T;
	CS_Pin_Enc2 = CS_Pin_Enc2T;
	CS_GPIO_Port_Enc2 = CS_GPIO_Port_Enc2T;
	ResolutionEncoders = ResolutionEncodersT;
	return 0;
}

float RoboArm::GetAngleEncoders(uint32_t encoderValue) {
	//	https://www.cuidevices.com/product/resource/amt22.pdf
	return calculateAngle(encoderValue, ResolutionEncoders);
}

float RoboArm::GetLinEncoders(float ang) {
	float pos;
	if (inverseLinZero) {
		ang = abs(360 - ang);
	}
	pos = ang * distMax / 360.0;
	return pos;
}

uint32_t RoboArm::GetPosEncoders(uint8_t numEnc) {
	switch (numEnc) {
	case 1:
		posNowEnc1 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
				ResolutionEncoders);
		return posNowEnc1;
		break;
	case 2:
		posNowEnc2 = getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
				ResolutionEncoders);
		return posNowEnc2;
		break;
	default:
		return 1;
		break;
	}
	return 0;
}

float RoboArm::GetLin() {
	int attempts = 0;

	uint32_t posnowT_2 = GetPosEncoders(2);
	while (posnowT_2 == 0xFFFF && ++attempts < 6) // +1.2 вдвічі збільшено кількість разів, можливо йому не вистачає. Обмежено, щоб не завис, якщо енкодери впали зовсім
		posnowT_2 = GetPosEncoders(2); //try again

	float pos = -1.0;
	float enc2mm = 16384.0 / (2.0 * 20.0 * 33.0 * 33.0 / (13.0 * 13.0)); //число одиниць енкодера на 1 мм лінійного руху

	// 0-5000 on encoder = 124mm-48mm;
	if (posnowT_2 <= 5000) {
		//the motor is somewhere between 48mm and 124mm
		pos = defaultDistanse - float(posnowT_2) / enc2mm;

	} else { //if (posnowT_2 <= 9000){ //приблизно по факту десь +- 8963
		pos = defaultDistanse + (16384.0 - float(posnowT_2)) / enc2mm;
		// the motor is somewhere 124...end of the hand

	}


	return pos;
}

float RoboArm::GetAng() {
	int attempts = 0;
	uint32_t posnowT_1 = GetPosEncoders(1);
	while (posnowT_1 == 0xFFFF && ++attempts < 6) // +1.2 вдвічі збільшено кількість разів
		posnowT_1 = GetPosEncoders(1); //try again

//	if (posnowT_1 == 0xFFFF) {
//			return -1;
//	}

	float ang_actual = GetAngleEncoders(posnowT_1);
	float ang = ang_actual + defaultAngle; //arm.ShiftZeroAng(ang_actual);
	if (ang > 360.0)
		ang -= 360.0;
	return ang;
}

int RoboArm::SetZeroEncoders() {
	HAL_Delay(300);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1,
			ResolutionEncoders);
	getPositionSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2,
			ResolutionEncoders);
	//	resetAMT22();
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc1, CS_Pin_Enc1);
	setZeroSPI(arm_hspi1, CS_GPIO_Port_Enc2, CS_Pin_Enc2);
	HAL_Delay(250);
	return 0;
}

//int RoboArm::SetSoftwareZero() {
//
//	int attempts = 0;
//	uint32_t posnowT_1 = GetPosEncoders(1);
//
//	while(posnowT_1 == 0xFFFF && ++attempts < 3)
//		posnowT_1 = GetPosEncoders(1); //try again
//
//	attempts = 0;
//
//	//			float ang = posnowT*360/16384;
//	ang_zero = GetAngleEncoders(posnowT_1);// - defaultAngle;// - defaultAngle; //0, 120, 240
//	if (ang_zero < 0.0)
//		ang_zero = 360.0 + ang_zero;
//	//			un_send.params.ang = angleT;
//
//	uint32_t posnowT_2 = GetPosEncoders(2);
//	while(posnowT_2 == 0xFFFF && ++attempts < 3)
//		posnowT_2 = GetPosEncoders(2); //try again
//	float ang_pos = GetAngleEncoders(posnowT_2);
//	lin_zero = GetLinEncoders(ang_pos);// - defaultDistanse;
//	if (lin_zero < 0.0)
//		lin_zero = distMax + lin_zero;
//
//	return 0;
//}

float RoboArm::UnshiftZeroAng(float angle) {
	float ang_actual = ang_zero + angle;
	if (ang_actual > 360.0)
		ang_actual -= 360.0;
	return ang_actual;
}

float RoboArm::UnshiftZeroLin(float distance) {
	float lin_actual = lin_zero + distance;
	if (lin_actual > distMax)
		lin_actual -= distMax;
	return lin_actual;
}

float RoboArm::ShiftZeroAng(float ang_actual) {
	float ang = ang_actual - ang_zero;
	if (ang < 0.0)
		ang = 360.0 + ang;
	return ang;
}

float RoboArm::ShiftZeroLin(float lin_actual) {
	float lin = lin_actual - lin_zero;
	if (lin < 0.0)
		lin = distMax + lin;
	return lin;
}

int RoboArm::SetSettMotors(UART_HandleTypeDef &huartTmc,
		TIM_HandleTypeDef &htim1, TIM_HandleTypeDef &htim2,
		TIM_HandleTypeDef &htim3, GPIO_TypeDef *Dir1_GPIO_Port_M1T,
		uint16_t Dir1_Pin_M1T, GPIO_TypeDef *Dir2_GPIO_Port_M2T,
		uint16_t Dir2_Pin_M2T, GPIO_TypeDef *Dir3_GPIO_Port_M3T,
		uint16_t Dir3_Pin_M3T, GPIO_TypeDef *En1_GPIO_Port_M1T,
		uint16_t En1_Pin_M1T, GPIO_TypeDef *En2_GPIO_Port_M2T,
		uint16_t En2_Pin_M2T, GPIO_TypeDef *En3_GPIO_Port_M3T,
		uint16_t En3_Pin_M3T, GPIO_TypeDef *Buser_GPIO_Port_IndT,
		uint16_t Buser_Pin_IndT) {
//		UART_HandleTypeDef &huart_tmcT) {
	htim1M1 = &htim1;
	htim2M2 = &htim2;
	htim3M3 = &htim3;

	Dir1_GPIO_Port_M1 = Dir1_GPIO_Port_M1T;
	Dir1_Pin_M1 = Dir1_Pin_M1T;
	Dir2_GPIO_Port_M2 = Dir2_GPIO_Port_M2T;
	Dir2_Pin_M2 = Dir2_Pin_M2T;
	Dir3_GPIO_Port_M3 = Dir3_GPIO_Port_M3T;
	Dir3_Pin_M3 = Dir3_Pin_M3T;

	En1_GPIO_Port_M1 = En1_GPIO_Port_M1T;
	En1_Pin_M1 = En1_Pin_M1T;
	En2_GPIO_Port_M2 = En2_GPIO_Port_M2T;
	En2_Pin_M2 = En2_Pin_M2T;
	En3_GPIO_Port_M3 = En3_GPIO_Port_M3T;
	En3_Pin_M3 = En3_Pin_M3T;

	En3_GPIO_Port_M3 = En3_GPIO_Port_M3T;
	En3_Pin_M3 = En3_Pin_M3T;
	Buser_GPIO_Port_Ind = Buser_GPIO_Port_IndT;
	Buser_Pin_Ind = Buser_Pin_IndT;

	SetEnable(1, true);
	SetEnable(2, true);
	SetEnable(3, true);

	tmcd_angle.setup(&huartTmc, 115200, tmcd_angle.SERIAL_ADDRESS_0);
	tmcd_gripper.setup(&huartTmc, 115200, tmcd_gripper.SERIAL_ADDRESS_2);
	tmcd_linear.setup(&huartTmc, 115200, tmcd_linear.SERIAL_ADDRESS_1);

	tmcd_angle.enable();
	tmcd_gripper.enable();
	tmcd_linear.enable();

	SetMicrosteps4All(7);
//	SetMicrosteps4All(4);

	// COMMENT 1
//	SetEnable(1, false);
//	SetEnable(2, false);
//	SetEnable(3, false);

	return 0;
}

int RoboArm::SetEnable(uint16_t numMotor, bool state) {

	GPIO_PinState pinSet;

	if (state) {
		pinSet = GPIO_PIN_RESET;
	} else {
		pinSet = GPIO_PIN_SET;
	}

	if (numMotor == 1) {
		HAL_GPIO_WritePin(En1_GPIO_Port_M1, En1_Pin_M1, pinSet);
	} else if (numMotor == 2) {
		HAL_GPIO_WritePin(En2_GPIO_Port_M2, En2_Pin_M2, pinSet);
	} else if (numMotor == 3) {
		HAL_GPIO_WritePin(En3_GPIO_Port_M3, En3_Pin_M3, pinSet);
	}

	return 0;
}
