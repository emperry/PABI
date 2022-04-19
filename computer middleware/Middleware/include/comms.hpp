#include <cstdint>

#ifndef COMMS_H
#define COMMS_H

extern uint8_t SYN[], ACK[], FIN[], RESET[];

#define NO_OP 0

// Communication compiler directives and constants
#define SYN_LEN 3
#define ACK_LEN 3
#define FIN_LEN 3
#define AVAIL_MODULES_LEN 3
#define RESET_LEN 1

// Opcodes
#define OP_SYN 1
#define OP_ACK 2
#define OP_NACK 3
#define OP_FIN 4
#define OP_INFO 5
#define OP_AVAIL 6
#define OP_MODCONF 7
#define OP_COMMAND 8
#define OP_COMMAND_ALL 9
#define OP_RESPOND 0x0A
#define OP_ERROR 0xFF

// Commands
#define CMD_STATUS 0x0001
#define CMD_POSITION 0x0002
#define CMD_RELAX 0x0003
#define CMD_P_GAIN 0x0005
#define CMD_I_GAIN 0x0006
#define CMD_D_GAIN 0x0007
#define CMD_VEL_GAIN 0x0008
#define CMD_ACC_GAIN 0x0009
#define CMD_JERK_GAIN 0x000A
#define CMD_SENSOR 0x0010
#define CMD_NEW_BITMAP 0x0100
#define CMD_NEW_EYE_POS 0x0200
#define CMD_EYE_BLINK 0x0300

// Module Commands
#define CMD_BRUSHED_DC 1
#define CMD_PWM_SERVO 2
#define CMD_XYZ_SMART_SERVO 3
#define CMD_DYNAMIXEL_SERVO 4
#define CMD_STEPPER 5
#define CMD_BRUSHLESS 6
#define CMD_BIN_OUT 7
#define CMD_POTENTIOMETER 8
#define CMD_QUAD_ENC 9
#define CMD_LIMIT_SWITCH 0x0A
#define CMD_XYZ_SMART_SENSOR 0x0B
#define CMD_DYNAMIXEL_SENSOR 0x0E
#define CMD_LCD_DISPLAY 0x0D
#define CMD_SPIENC 0x0C

// Error codes
#define ERR_UNKNOWN_OPCODE 0x01
#define ERR_UNKNOWN_MODULE 0x02
#define ERR_UKNOWN_ADDRESS 0x03
#define ERR_UNKNOWN_PACKAGE 0x04
#define ERR_UNKNOWN_COMMAND 0x05
#define ERR_PARAMS_OUT_OF_RANGE 0x06
#define ERR_INVALID_ORDER 0x07
#define ERR_NOT_IMPLEMENTED 0x08

#endif