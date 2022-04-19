/*
 * TCA9555_GPIO_Driver.h
 *
 *  Created on: Nov 7, 2019
 *      Author: tdubuke
 */

#ifndef TCA9555_GPIO_DRIVER_H_
#define TCA9555_GPIO_DRIVER_H_

#include "stm32h7xx_hal.h"
#include "main.h"

#define PORT_0 0x00
#define PORT_1 0x01

#define OUTPUT 0x00
#define INPUT 0x01

#define LOW 0x00
#define HIGH 0x01

#define NORMAL 0x00
#define INV 0x01

#define CR_IN0 0x00
#define CR_IN1 0x01
#define CR_OUT0 0x02
#define CR_OUT1 0x03
#define CR_INV0 0x04
#define CR_INV1 0x05
#define CR_CFG0 0x06
#define CR_CFG1 0x07

class TCA9555_GPIO_Driver {
public:
	TCA9555_GPIO_Driver(const uint8_t addr, I2C_HandleTypeDef *i2c);
	void begin();

	void setOutputState(uint8_t pinNum, uint8_t value);
	uint8_t getInputState(uint8_t pinNum);

	void pinMode(uint8_t pinNum, uint8_t mode);
	void pinPol(uint8_t pinNum, uint8_t pol);

private:
	uint8_t _i2caddr;
	I2C_HandleTypeDef *_i2c;
};

#endif /* TCA9555_GPIO_DRIVER_H_ */
