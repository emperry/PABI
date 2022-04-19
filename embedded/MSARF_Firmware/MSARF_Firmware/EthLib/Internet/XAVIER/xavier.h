/*
 * xavier.h
 *
 *  Created on: Nov 21, 2019
 *      Author: tdubuke
 */

#ifndef XAVIER_H_
#define XAVIER_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"

struct MODULECONFIG_t{
	uint8_t modType;
	uint16_t nucleoAddress;
	uint8_t nucleoPackage;
	uint8_t param[32];
};

/* Xavier modules */
#define noOp 					0x00

#define BRUSHED_DC 				0x01
#define PWM_SERVO 				0x02
#define XYZ_SMART_SERVO			0x03
#define DYNAMIXEL_SERVO			0x04

#define STEPPER_MOTOR 			0x05
#define BRUSHLESS_MOTOR 		0x06
#define BINARY_OUTPUT			0x07

#define POTENTIOMETER 			0x08
#define QUAD_ENCODER			0x09
#define BINARY_INPUT			0x0A
#define XYZ_SMART_SENSOR		0x0B
#define DYNAMIXEL_SENSOR		0x0E
#define SPI_ENCODER				0x0C

#define LCD_DISPLAY 			0x0D

/* Number of Xavier Modules Available */
#define BRUSHED_AVAIL 				0x0F
#define PWM_AVAIL 					0x0F
#define XYZ_SMART_SERVO_AVAIL		0xFF
#define DYNAMIXEL_SERVO_AVAIL		0xFF
#define STEP_AVAIL 					0x00
#define BRUSHLESS_AVAIL 			0x00
#define BINARY_IN_AVAIL				0x00
#define POT_AVAIL 					0x00
#define ENCODER_AVAIL 				0x04
#define BINARY_OUT_AVAIL			0x00
#define XYZ_SMART_SENSOR_AVAIL		0xFF
#define	DYNAMIXEL_SENSOR_AVAIL 		0xFF
#define LCD_AVAIL 					0x02

/* Xavier State Machine*/
#define STATE_XAVIER_SYN 		0x00
#define STATE_XAVIER_MODULES 	0x01
#define STATE_XAVIER_FIN 		0x02
#define STATE_XAVIER_COMMAND 	0x03

/*Xavier Opcodes*/
#define XAVIER_SYN     			0x01
#define XAVIER_ACK     			0x02
#define XAVIER_NACK    			0x03
#define XAVIER_FIN     			0x04
#define XAVIER_INFO    			0x05
#define XAVIER_AVAIL   			0x06
#define XAVIER_MODCONF 			0x07
#define XAVIER_COMMAND 			0x08
#define XAVIER_COMMAND_ALL 		0x09
#define XAVIER_RESPOND 			0x0A
#define XAVIER_RESET			0xFE
#define XAVIER_ERROR		 	0xFF

/* Xavier Commands */
#define XAVIER_STATUS			0x0001

#define XAVIER_SETPOINT			0x0002
#define XAVIER_REBOOT			0x0003
#define XAVIER_PACKAGE_P		0x0005
#define XAVIER_PACKAGE_I		0x0006
#define XAVIER_PACKAGE_D		0x0007
#define XAVIER_PACKAGE_VEL		0x0008
#define XAVIER_PACKAGE_ACC		0x0009
#define XAVIER_PACKAGE_JER		0x000A

#define XAVIER_GET_SENSOR		0x0010

#define XAVIER_NEW_BITMAP		0x0100
#define XAVIER_EYE_POS			0x0200
#define XAVIER_BLINK			0x0300

/*Xavier Error Codes*/
#define UNKNOWN_OPCODE 			0x01
#define UNKNOWN_MODULE 			0x02
#define UNKNOWN_ADDRESS			0x03
#define UNKNOWN_PACKAGE			0x04
#define UNKNOWN_COMMAND			0x05
#define PARAMS_OUT_OF_RANGE		0x06
#define INVALID_ORDER			0x07
#define NOT_IMPLEMENTED			0x08
#define FAULTY_DEVICE			0x09

/*Xavier message structure*/
#define XAVIER_SYN_INIT 		0x01
#define XAVIER_SYN_SEQ 			0x02

#define XAVIER_ACK_SYN 			0x01
#define XAVIER_ACK_SEQ 			0x02

uint8_t getResponse(uint8_t *buf, uint8_t len);
uint8_t getOpCode(uint8_t *buf);
uint8_t* parseSYN(uint8_t *buf, uint8_t len);
uint8_t* parseACK(uint8_t *buf, uint8_t len);
uint8_t* parseFIN(uint8_t *buf, uint8_t len);
void parseMODCONF(uint8_t *buf, uint8_t len, struct MODULECONFIG_t *mod);
void parseCOMMAND(uint8_t *buf, uint8_t len, struct MODULECOMMAND_t *com);

void resetStructs();
void initStructs();

uint8_t buildACK(uint8_t currSeq, uint8_t nextSeq, uint8_t retPack[]);
uint8_t buildModAck(struct MODULECONFIG_t *config, uint8_t retPack[]);
uint8_t buildInfo(uint8_t *retPack);
uint8_t buildCommandResponse(struct MODULECOMMAND_t *com, uint8_t responseData[], uint8_t len, uint8_t retPack[]);
uint8_t buildError(struct MODULECOMMAND_t *com, uint8_t errorCode, uint8_t retPack[]);
uint8_t buildAVAIL(uint8_t *retPack);

GPIO_TypeDef* getPort(uint8_t portLetter);
uint16_t getPin(uint8_t pinNum);
void getMotorLayout(uint8_t motorNum, uint8_t layout[]);

#ifdef __cplusplus
}
#endif

#endif /* XAVIER_H_ */
