/*
 * xavier.c
 *
 *  Created on: Nov 21, 2019
 *      Author: tdubuke
 */

#include "socket.h"
#include "xavier.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Actuator.h"
#include "XYZrobotServo.h"
#include "Dynamixelrobotservo.h"
#include "PWMServo.h"
#include "BrushedDCMotor.h"
#include "BrushlessMotor.h"

#include "Sensor.h"
#include "Encoder.h"
#include "AS5048A.h"

#include "Package.h"

#include "PWMBank.h"
#include "GPIOBank.h"

#include "ST7789.h"

extern struct ST7789 *lcd1;
extern struct ST7789 *lcd2;
extern uint8_t generic_st7789[];

extern uint16_t *lcd1_screen_buffer;
extern uint16_t *lcd2_screen_buffer;

extern SPI_HandleTypeDef hspi1;	// LCD1
extern SPI_HandleTypeDef hspi3; // LCD2
extern SPI_HandleTypeDef hspi5; // Other
extern UART_HandleTypeDef huart4; // Smart UART Servos
extern USART_HandleTypeDef husart2; //Dynamixel Servo
extern I2C_HandleTypeDef hi2c1; // GPIO Expanders
extern I2C_HandleTypeDef hi2c2; // PWM Expanders

extern LPTIM_HandleTypeDef hlptim2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern osThreadId_t lcdDisplaysTaskHandle;
extern osThreadId_t motorOutputTaskHandle;

extern struct Package *Packages[255];

extern struct GPIOBank *_GPIOBank;
extern struct PWMBank *_PWMBank;

extern uint8_t lcd1_x_t;
extern uint8_t lcd1_y_t;
extern uint8_t lcd2_x_t;
extern uint8_t lcd2_y_t;
extern uint8_t blink;

uint8_t currState = STATE_XAVIER_SYN;
uint8_t modsAvail[] = {
		BRUSHED_DC,      		BRUSHED_AVAIL,
		PWM_SERVO,       		PWM_AVAIL,
		XYZ_SMART_SERVO,    	XYZ_SMART_SERVO_AVAIL,
		DYNAMIXEL_SERVO,		DYNAMIXEL_SERVO_AVAIL,
		STEPPER_MOTOR,   		STEP_AVAIL,
		BRUSHLESS_MOTOR, 		BRUSHLESS_AVAIL,
		BINARY_OUTPUT,			BINARY_OUT_AVAIL,
		POTENTIOMETER,   		POT_AVAIL,
		QUAD_ENCODER,       	ENCODER_AVAIL,
		BINARY_INPUT,			BINARY_IN_AVAIL,
		LCD_DISPLAY,     		LCD_AVAIL
};

void resetStructs(){
	vPortFree(_PWMBank);
	vPortFree(_GPIOBank);
}

void initStructs(){
	PWMBank *PWMBank_ptr = (PWMBank *)pvPortMalloc(sizeof(PWMBank));
	PWMBank PWMBank;
	memcpy(PWMBank_ptr, &PWMBank, sizeof(PWMBank));

	_PWMBank = PWMBank_ptr;
	_PWMBank->addDriver(0x80, &hi2c1);
	_PWMBank->addDriver(0x82, &hi2c1);

	GPIOBank *GPIOBank_ptr = (GPIOBank *)pvPortMalloc(sizeof(GPIOBank));
	GPIOBank GPIOBank(0);
	memcpy(GPIOBank_ptr, &GPIOBank, sizeof(GPIOBank));

	_GPIOBank = GPIOBank_ptr;
	_GPIOBank->addDriver(0x42, &hi2c2);
	_GPIOBank->addDriver(0x40, &hi2c2);
}

uint8_t getResponse(uint8_t *buf, uint8_t len){
	uint8_t opcode = getOpCode(buf);

	switch(opcode){
	case XAVIER_SYN:
	{
		// BIG RESET HOURS
		resetStructs();
		initStructs();

		currState = STATE_XAVIER_MODULES;
		return buildACK(buf[XAVIER_SYN_SEQ], buf[XAVIER_SYN_SEQ] + 1, buf);

		break;
	}
	case XAVIER_ACK:
		break;
	case XAVIER_NACK:
		break;
	case XAVIER_FIN:
		if(currState == STATE_XAVIER_MODULES){
			// init all PIDs
			for(int i = 0; i < 255; i++){
				if(Packages[i] != 0){
					// Sensor and actuator, needs PID
					if(Packages[i]->getSensor() != 0 && Packages[i]->getActuator() != 0 && Packages[i]->needsPID){
						Packages[i]->initPID();
					}
				}
			}

			if(lcd1 != 0 && lcd2 != 0){
				osThreadResume(lcdDisplaysTaskHandle);
			}
			osThreadResume(motorOutputTaskHandle);

			currState = STATE_XAVIER_COMMAND;
			return buildACK(buf[XAVIER_SYN_SEQ], 0, buf);
		}else{
			return buildError(0, INVALID_ORDER, buf);
		}
		break;
	case XAVIER_INFO:
		return buildInfo(buf);
		break;
	case XAVIER_AVAIL:
		return buildAVAIL(buf);
		break;
	case XAVIER_MODCONF:
	{
		struct MODULECONFIG_t moduleCommand;
		parseMODCONF(buf, len, &moduleCommand);

		// If it does not exist, create it
		if(Packages[moduleCommand.nucleoPackage] == 0){
			Package *package_ptr = (Package *)pvPortMalloc(sizeof(Package));
			Package package(moduleCommand.nucleoPackage);

			memcpy(package_ptr, &package, sizeof(Package));

			Packages[moduleCommand.nucleoPackage] = package_ptr;
		}

		// Init module based on command
		switch(moduleCommand.modType){
		case BRUSHED_DC:
		{
			if(Packages[moduleCommand.nucleoPackage]->getActuator() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			BrushedDCMotor *mtr_ptr = (BrushedDCMotor *)pvPortMalloc(sizeof(BrushedDCMotor));
			BrushedDCMotor mtr(_PWMBank, _GPIOBank, moduleCommand.nucleoAddress);
			memcpy(mtr_ptr, &mtr, sizeof(BrushedDCMotor));

			uint16_t motorNum = moduleCommand.param[0];

			uint8_t layout[4];
			getMotorLayout(motorNum, layout);
			int retVal = mtr_ptr->begin(layout[0], layout[1], layout[2], layout[3]);

			if(retVal == -1){
				vPortFree(mtr_ptr);
				return buildError(0, INVALID_ORDER, buf);
			}

			Packages[moduleCommand.nucleoPackage]->addActuator(mtr_ptr);

			return buildModAck(&moduleCommand, buf);

			break;
		}
		case PWM_SERVO:
		{
			if(Packages[moduleCommand.nucleoPackage]->getActuator() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			uint8_t pwmPin = moduleCommand.param[0] + 16;
			uint32_t maxRot = (moduleCommand.param[1] << 24) | (moduleCommand.param[2] << 16) | (moduleCommand.param[3] << 8) | (moduleCommand.param[4]);

			float fmaxRot;
			memcpy(&fmaxRot, &maxRot, sizeof(uint32_t));

			PWMServo *servo_ptr = (PWMServo *)pvPortMalloc(sizeof(PWMServo));
			PWMServo servo(_PWMBank, fmaxRot, moduleCommand.nucleoAddress);

			memcpy(servo_ptr, &servo, sizeof(PWMServo));
			int retVal = servo_ptr->begin(pwmPin);

			if(retVal == -1){
				vPortFree(servo_ptr);
				return buildError(0, INVALID_ORDER, buf);
			}

			Packages[moduleCommand.nucleoPackage]->addActuator(servo_ptr);

			return buildModAck(&moduleCommand, buf);

			break;
		}
		case XYZ_SMART_SERVO:
		{
			if(Packages[moduleCommand.nucleoPackage]->getActuator() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			// Need to create a new XYZ Smart Servo Object
			if(Packages[moduleCommand.nucleoPackage]->getSensor() == 0){
				uint8_t uartAddress = moduleCommand.param[0];

				XYZrobotServo *servo_ptr = (XYZrobotServo *)pvPortMalloc(sizeof(XYZrobotServo));
				XYZrobotServo servo(&huart4, uartAddress, moduleCommand.nucleoAddress);

				memcpy(servo_ptr, &servo, sizeof(XYZrobotServo));

				uint8_t id = servo_ptr->readIdEeprom();
				if(id != uartAddress){
					vPortFree(servo_ptr);
					return buildError(0, FAULTY_DEVICE, buf);
				}

				Packages[moduleCommand.nucleoPackage]->addActuator(servo_ptr);
				Packages[moduleCommand.nucleoPackage]->needsPID = false;

				return buildModAck(&moduleCommand, buf);
			// Else one already exists so get its reference and set it as the actuator
			}else{
				XYZrobotServo *servo_ptr = (XYZrobotServo *)Packages[moduleCommand.nucleoPackage]->getSensor();
				Packages[moduleCommand.nucleoPackage]->addActuator(servo_ptr);

				return buildModAck(&moduleCommand, buf);
			}

			break;
		}
		case DYNAMIXEL_SERVO:
				{
					if(Packages[moduleCommand.nucleoPackage]->getActuator() != 0){
						return buildError(0, UNKNOWN_ADDRESS, buf);
					}

					// Need to create a new Dynamixel Servo Object
					if(Packages[moduleCommand.nucleoPackage]->getSensor() == 0){
						uint8_t usartAddress = moduleCommand.param[0];

						DynamixelrobotServo *servo_ptr = (DynamixelrobotServo *)pvPortMalloc(sizeof(DynamixelrobotServo));
						DynamixelrobotServo servo(&husart2, usartAddress, moduleCommand.nucleoAddress);

						memcpy(servo_ptr, &servo, sizeof(DynamixelrobotServo));

						uint8_t id = servo_ptr->readIdEeprom();
						if(id != usartAddress){
							vPortFree(servo_ptr);
							return buildError(0, FAULTY_DEVICE, buf);
						}

						Packages[moduleCommand.nucleoPackage]->addActuator(servo_ptr);
						Packages[moduleCommand.nucleoPackage]->needsPID = false;

						return buildModAck(&moduleCommand, buf);
					// Else one already exists so get its reference and set it as the actuator
					}else{
						DynamixelrobotServo *servo_ptr = (DynamixelrobotServo *)Packages[moduleCommand.nucleoPackage]->getSensor();
						Packages[moduleCommand.nucleoPackage]->addActuator(servo_ptr);

						return buildModAck(&moduleCommand, buf);
					}

					break;
				}
		case STEPPER_MOTOR:
		{
			return buildError(0, NOT_IMPLEMENTED, buf);
		}
		case BRUSHLESS_MOTOR:
		{
			if(Packages[moduleCommand.nucleoPackage]->getActuator() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			uint8_t pwmPin = moduleCommand.param[0] + 16;

			BrushlessMotor *motor_ptr = (BrushlessMotor *)pvPortMalloc(sizeof(BrushlessMotor));
			BrushlessMotor motor(_PWMBank, moduleCommand.nucleoAddress);

			memcpy(motor_ptr, &motor, sizeof(BrushlessMotor));
			int retVal = motor_ptr->begin(pwmPin);

			if(retVal == -1){
				vPortFree(motor_ptr);
				return buildError(0, INVALID_ORDER, buf);
			}

			Packages[moduleCommand.nucleoPackage]->addActuator(motor_ptr);

			return buildModAck(&moduleCommand, buf);

			break;
		}
		case BINARY_OUTPUT:
		{
			return buildError(0, NOT_IMPLEMENTED, buf);
		}
		case POTENTIOMETER:
		{
			return buildError(0, NOT_IMPLEMENTED, buf);
		}
		case QUAD_ENCODER:
		{
			if(Packages[moduleCommand.nucleoPackage]->getSensor() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			uint8_t encoderPort = moduleCommand.param[0];
			uint16_t ticksPerRev = moduleCommand.param[1] << 8 | moduleCommand.param[2];

			Encoder *encoder_ptr = (Encoder *)pvPortMalloc(sizeof(Encoder));
			if(encoderPort == 0){
				Encoder enc(&hlptim2, 0, ticksPerRev, moduleCommand.nucleoAddress);
				memcpy(encoder_ptr, &enc, sizeof(Encoder));
			}else if(encoderPort == 1){
				Encoder enc(0, &htim2, ticksPerRev, moduleCommand.nucleoAddress);
				memcpy(encoder_ptr, &enc, sizeof(Encoder));
			}else if(encoderPort == 2){
				Encoder enc(0, &htim3, ticksPerRev, moduleCommand.nucleoAddress);
				memcpy(encoder_ptr, &enc, sizeof(Encoder));
			}else if(encoderPort == 3){
				Encoder enc(0, &htim4, ticksPerRev, moduleCommand.nucleoAddress);
				memcpy(encoder_ptr, &enc, sizeof(Encoder));
			}else{
				return buildError(0, PARAMS_OUT_OF_RANGE, buf);
			}

			encoder_ptr->initEncoder();

			Packages[moduleCommand.nucleoPackage]->addSensor(encoder_ptr);

			return buildModAck(&moduleCommand, buf);

			break;
		}
		case BINARY_INPUT:
		{
			return buildError(0, NOT_IMPLEMENTED, buf);
		}
		case XYZ_SMART_SENSOR:
		{
			if(Packages[moduleCommand.nucleoPackage]->getSensor() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			// Need to create a new XYZ Smart Servo Object
			if(Packages[moduleCommand.nucleoPackage]->getActuator() == 0){
				uint8_t uartAddress = moduleCommand.param[0];

				XYZrobotServo *servo_ptr = (XYZrobotServo *)pvPortMalloc(sizeof(XYZrobotServo));
				XYZrobotServo servo(&huart4, uartAddress, moduleCommand.nucleoAddress);

				memcpy(servo_ptr, &servo, sizeof(XYZrobotServo));

				// Verify Servo is responding
				uint8_t id = servo_ptr->readIdEeprom();
				if(id != uartAddress){
					vPortFree(servo_ptr);
					return buildError(0, FAULTY_DEVICE, buf);
				}

				Packages[moduleCommand.nucleoPackage]->addSensor(servo_ptr);
				Packages[moduleCommand.nucleoPackage]->needsPID = false;

				return buildModAck(&moduleCommand, buf);
			// Else one already exists so get its reference and set it as the actuator
			}else{
				XYZrobotServo *servo_ptr = (XYZrobotServo *)Packages[moduleCommand.nucleoPackage]->getActuator();
				Packages[moduleCommand.nucleoPackage]->addSensor(servo_ptr);

				return buildModAck(&moduleCommand, buf);
			}

			break;
		}
		case DYNAMIXEL_SENSOR:
		{
			if(Packages[moduleCommand.nucleoPackage]->getSensor() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			// Need to create a new Dynamixel Servo Object
			if(Packages[moduleCommand.nucleoPackage]->getActuator() == 0){
				uint8_t usartAddress = moduleCommand.param[0];

				DynamixelrobotServo *servo_ptr = (DynamixelrobotServo *)pvPortMalloc(sizeof(DynamixelrobotServo));
				DynamixelrobotServo servo(&husart2, usartAddress, moduleCommand.nucleoAddress);

				memcpy(servo_ptr, &servo, sizeof(DynamixelrobotServo));

				// Verify Servo is responding
				uint8_t id = servo_ptr->readIdEeprom();
				if(id != usartAddress){
					vPortFree(servo_ptr);
					return buildError(0, FAULTY_DEVICE, buf);
				}

				Packages[moduleCommand.nucleoPackage]->addSensor(servo_ptr);
				Packages[moduleCommand.nucleoPackage]->needsPID = false;

				return buildModAck(&moduleCommand, buf);
			// Else one already exists so get its reference and set it as the actuator
			}else{
				DynamixelrobotServo *servo_ptr = (DynamixelrobotServo *)Packages[moduleCommand.nucleoPackage]->getActuator();
				Packages[moduleCommand.nucleoPackage]->addSensor(servo_ptr);

				return buildModAck(&moduleCommand, buf);
			}

			break;
		}
		case SPI_ENCODER:
		{
			if(Packages[moduleCommand.nucleoPackage]->getSensor() != 0){
				return buildError(0, UNKNOWN_ADDRESS, buf);
			}

			uint8_t sizeOFDaisy = moduleCommand.param[0];
			uint8_t numInDaisy = moduleCommand.param[1];

			AS5048A *sensor_ptr = (AS5048A *)pvPortMalloc(sizeof(AS5048A));
			AS5048A sensor(moduleCommand.nucleoAddress, &hspi5, sizeOFDaisy, numInDaisy);

			memcpy(sensor_ptr, &sensor, sizeof(AS5048A));

			sensor_ptr->swReset();

			Packages[moduleCommand.nucleoPackage]->addSensor(sensor_ptr);

			return buildModAck(&moduleCommand, buf);

			break;
		}

		case LCD_DISPLAY:
		{
			lcd1_screen_buffer = (uint16_t *)pvPortMalloc(57600 * sizeof(uint16_t));

			ST7789 *lcd1_ptr = (ST7789 *)pvPortMalloc(sizeof(ST7789));
			ST7789 lcd1_obj(lcd1_screen_buffer, 57600, &hspi1, LCD1_DCX_GPIO_Port, LCD1_DCX_Pin, LCD1_RSVD_Pin);
			memcpy(lcd1_ptr, &lcd1_obj, sizeof(ST7789));
			lcd1 = lcd1_ptr;
			lcd1->displayInit(generic_st7789, moduleCommand.nucleoAddress);

			lcd2_screen_buffer = (uint16_t *)pvPortMalloc(57600 * sizeof(uint16_t));

			ST7789 *lcd2_ptr = (ST7789 *)pvPortMalloc(sizeof(ST7789));
			ST7789 lcd2_obj(lcd2_screen_buffer, 57600, &hspi3, LCD2_DCX_GPIO_Port, LCD2_DCX_Pin, LCD2_RSVD_Pin);
			memcpy(lcd2_ptr, &lcd2_obj, sizeof(ST7789));
			lcd2 = lcd2_ptr;
			lcd2->displayInit(generic_st7789, moduleCommand.nucleoAddress);

			return buildModAck(&moduleCommand, buf);

			break;
		}
		default:
			return buildError(0, UNKNOWN_MODULE, buf);
		}
		break;
	}
	case XAVIER_COMMAND:
	{
		if(currState == STATE_XAVIER_COMMAND){
			struct MODULECOMMAND_t *comm_ptr = (struct MODULECOMMAND_t *)pvPortMalloc(sizeof(struct MODULECOMMAND_t));
			parseCOMMAND(buf, len, comm_ptr);

			if(Packages[comm_ptr->packageNum] == 0){
				return buildError(comm_ptr, UNKNOWN_PACKAGE, buf);
			}

			switch(comm_ptr->command){
			case XAVIER_SETPOINT:
			{
				if(comm_ptr->paramLen >= 4){
					uint32_t newPos = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
					float fnewPos;
					memcpy(&fnewPos, &newPos, sizeof(uint32_t));
					Packages[comm_ptr->packageNum]->newPosSetpoint(fnewPos);
				}

				if(comm_ptr->paramLen >= 8){
					uint32_t newVel = (comm_ptr->params[4] << 24) | (comm_ptr->params[5] << 16) | (comm_ptr->params[6] << 8) | (comm_ptr->params[7]);
					float fnewVel;
					memcpy(&fnewVel, &newVel, sizeof(uint32_t));
					Packages[comm_ptr->packageNum]->newVelSetpoint(fnewVel);
				}

				if(comm_ptr->paramLen >= 12){
					uint32_t newAcc = (comm_ptr->params[8] << 24) | (comm_ptr->params[9] << 16) | (comm_ptr->params[10] << 8) | (comm_ptr->params[11]);
					float fnewAcc;
					memcpy(&fnewAcc, &newAcc, sizeof(uint32_t));
					Packages[comm_ptr->packageNum]->newAccSetpoint(fnewAcc);
				}

				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_REBOOT:
			{
				if(comm_ptr->params[0] == 0){
					Packages[comm_ptr->packageNum]->torque(false);
				}else{
					Packages[comm_ptr->packageNum]->torque(true);
				}
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_P:
			{
				uint32_t newP = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewP;
				memcpy(&fnewP, &newP, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setPGain(fnewP);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_I:
			{
				uint32_t newI = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewI;
				memcpy(&fnewI, &newI, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setIGain(fnewI);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_D:
			{
				uint32_t newD = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewD;
				memcpy(&fnewD, &newD, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setDGain(fnewD);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_VEL:
			{
				uint32_t newVelGain = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewVelGain;
				memcpy(&fnewVelGain, &newVelGain, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setVelGain(fnewVelGain);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_ACC:
			{
				uint32_t newAccGain = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewAccGain;
				memcpy(&fnewAccGain, &newAccGain, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setAccGain(fnewAccGain);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_PACKAGE_JER:
			{
				uint32_t newJerkGain = (comm_ptr->params[0] << 24) | (comm_ptr->params[1] << 16) | (comm_ptr->params[2] << 8) | (comm_ptr->params[3]);
				float fnewJerkGain;
				memcpy(&fnewJerkGain, &newJerkGain, sizeof(uint32_t));
				Packages[comm_ptr->packageNum]->setJerkGain(fnewJerkGain);
				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_GET_SENSOR:
			{
				float sensorValue = Packages[comm_ptr->packageNum]->getSensorValue();
				uint32_t intSensorVal;

				memcpy(&intSensorVal, &sensorValue, sizeof(uint32_t));
				uint8_t sensorValueArr[4];

				sensorValueArr[0] = intSensorVal >> 24;
				sensorValueArr[1] = intSensorVal >> 16;
				sensorValueArr[2] = intSensorVal >> 8;
				sensorValueArr[3] = intSensorVal;

				return buildCommandResponse(comm_ptr, sensorValueArr, 4, buf);
				break;
			}
			case XAVIER_NEW_BITMAP:
			{
				// LOL fuck no
				return buildError(0, NOT_IMPLEMENTED, buf);
				break;
			}
			case XAVIER_EYE_POS:
			{
				if(comm_ptr->params[0] < 0){
					comm_ptr->params[0] = 0;
				}

				if(comm_ptr->params[0] > 160){
					comm_ptr->params[0] = 160;
				}

				if(comm_ptr->params[1] < 0){
					comm_ptr->params[1] = 0;
				}

				if(comm_ptr->params[1] > 160){
					comm_ptr->params[1] = 160;
				}

				if(comm_ptr->params[2] < 0){
					comm_ptr->params[2] = 0;
				}

				if(comm_ptr->params[2] > 160){
					comm_ptr->params[2] = 160;
				}

				if(comm_ptr->params[3] < 0){
					comm_ptr->params[3] = 0;
				}

				if(comm_ptr->params[3] > 160){
					comm_ptr->params[3] = 160;
				}

				lcd1_x_t = comm_ptr->params[0];
				lcd1_y_t = comm_ptr->params[1];
				lcd2_x_t = comm_ptr->params[2];
				lcd2_y_t = comm_ptr->params[3];

				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			}
			case XAVIER_BLINK:
				blink = comm_ptr->params[0];

				return buildCommandResponse(comm_ptr, {}, 0, buf);
				break;
			default:
			{
				return buildError(comm_ptr, UNKNOWN_COMMAND, buf);
			}
			}
		}else{
			return buildError(0, INVALID_ORDER, buf);
		}
	}
	case XAVIER_RESPOND:
		return buildError(0, NOT_IMPLEMENTED, buf);
		break;
	case XAVIER_RESET:
		__NVIC_SystemReset();
		return buildError(0, NOT_IMPLEMENTED, buf);
		break;
	default:
		return buildError(0, UNKNOWN_OPCODE, buf);
		break;
	}
	return 0;
}

uint8_t getOpCode(uint8_t *buf){
	return *buf;
}

uint8_t buildACK(uint8_t currSeq, uint8_t nextSeq, uint8_t *retPack){
	retPack[0] = XAVIER_ACK;
	retPack[1] = currSeq;
	retPack[2] = nextSeq;

	return 3;
}

uint8_t buildModAck(struct MODULECONFIG_t *config, uint8_t retPack[]){
	retPack[0] = XAVIER_ACK;
	retPack[1] = config->modType;
	retPack[2] = config->nucleoAddress >> 8;
	retPack[3] = config->nucleoAddress;
	retPack[4] = config->nucleoPackage;

	return 5;
}

uint8_t buildError(struct MODULECOMMAND_t *com, uint8_t errorCode, uint8_t retPack[]){
	retPack[0] = XAVIER_ERROR;
	retPack[1] = errorCode;

	if(com != 0){
		retPack[2] = com->command >> 8;
		retPack[3] = com->command;
		retPack[4] = com->address >> 8;
		retPack[5] = com->address;
		retPack[6] = com->packageNum;

		return 7;
	}else{
		return 2;
	}
}

uint8_t buildCommandResponse(struct MODULECOMMAND_t *com, uint8_t responseData[], uint8_t len, uint8_t retPack[]){
	retPack[0] = XAVIER_RESPOND;
	retPack[1] = com->address >> 8;
	retPack[2] = com->address;
	retPack[3] = com->packageNum;
	retPack[4] = com->command >> 8;
	retPack[5] = com->command;
	retPack[6] = len;
	memcpy(&retPack[7], responseData, len);

	return 7 + len;
}

uint8_t buildAVAIL(uint8_t *retPack){
	memcpy(retPack, modsAvail, sizeof(modsAvail));
	return sizeof(modsAvail);
}

uint8_t buildInfo(uint8_t *retPack){
	char info[12] = {'N', 'U', 'C', 'L', 'E', 'O', '_', 'H', '7', ' ', 'V', '0'};
	memcpy(retPack, info, 12);
	return 12;
}

uint8_t* parseACK(uint8_t *buf, uint8_t len){
	return 0;
}

uint8_t* parseFIN(uint8_t *buf, uint8_t len){
	return 0;
}

void parseMODCONF(uint8_t *buf, uint8_t len, struct MODULECONFIG_t *mod){
	mod->modType = buf[1];
	mod->nucleoAddress = (buf[2] << 8) | buf[3];
	mod->nucleoPackage = buf[4];
	memcpy(mod->param, &buf[6], buf[5]);
}

void parseCOMMAND(uint8_t *buf, uint8_t len, struct MODULECOMMAND_t *com){
	com->address = (buf[1] << 8) | buf[2];
	com->packageNum = buf[3];
	com->command = (buf[4] << 8) | buf[5];
	com->paramLen = buf[6];
	memcpy(com->params, &buf[7], com->paramLen);
}

GPIO_TypeDef* getPort(uint8_t portLetter){
	switch(portLetter){
	case 'A':
		return GPIOA;
	case 'B':
		return GPIOB;
	case 'C':
		return GPIOC;
	case 'D':
		return GPIOD;
	case 'E':
		return GPIOE;
	case 'F':
		return GPIOF;
	case 'G':
		return GPIOG;
	case 'H':
		return GPIOH;
	case 'I':
		return GPIOI;
	case 'J':
		return GPIOJ;
	}

	return 0;
}

uint16_t getPin(uint8_t pinNum){
	switch(pinNum){
	case 0:
		return GPIO_PIN_0;
	case 1:
		return GPIO_PIN_1;
	case 2:
		return GPIO_PIN_2;
	case 3:
		return GPIO_PIN_3;
	case 4:
		return GPIO_PIN_4;
	case 5:
		return GPIO_PIN_5;
	case 6:
		return GPIO_PIN_6;
	case 7:
		return GPIO_PIN_7;
	case 8:
		return GPIO_PIN_8;
	case 9:
		return GPIO_PIN_9;
	case 10:
		return GPIO_PIN_10;
	case 11:
		return GPIO_PIN_11;
	case 12:
		return GPIO_PIN_12;
	case 13:
		return GPIO_PIN_13;
	case 14:
		return GPIO_PIN_14;
	case 15:
		return GPIO_PIN_15;
	}

	return 0;
}

void getMotorLayout(uint8_t motorNum, uint8_t layout[]){
	switch(motorNum){
	case 0:
		layout[0] = 8;
		layout[1] = 2;
		layout[2] = 1;
		layout[3] = 0;
		break;
	case 1:
		layout[0] = 10;
		layout[1] = 5;
		layout[2] = 4;
		layout[3] = 3;
		break;
	case 2:
		layout[0] = 12;
		layout[1] = 10;
		layout[2] = 7;
		layout[3] = 6;
		break;
	case 3:
		layout[0] = 14;
		layout[1] = 13;
		layout[2] = 12;
		layout[3] = 11;
		break;
	case 4:
		layout[0] = 9;
		layout[1] = 0;
		layout[2] = 1;
		layout[3] = 2;
		break;
	case 5:
		layout[0] = 11;
		layout[1] = 3;
		layout[2] = 4;
		layout[3] = 5;
		break;
	case 6:
		layout[0] = 13;
		layout[1] = 6;
		layout[2] = 7;
		layout[3] = 10;
		break;
	case 7:
		layout[0] = 15;
		layout[1] = 11;
		layout[2] = 12;
		layout[3] = 13;
		break;
	}
}
