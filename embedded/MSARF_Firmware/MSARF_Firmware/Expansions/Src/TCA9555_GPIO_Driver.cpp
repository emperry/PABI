/*
 * TCA9555_GPIO_Driver.cpp
 *
 *  Created on: Nov 7, 2019
 *      Author: tdubuke
 */

#include "TCA9555_GPIO_Driver.h"

/*!
 *  @brief  Instantiates a new TCA9555 GPIO driver chip with the I2C address on a
 * I2C interface
 *  @param  addr The 7-bit I2C address to locate this chip, valid addresses are
 *  0x20 - 0x27
 *  @param  i2c  A reference to a 'HAL I2C' object that we'll use to communicate
 *  with
 */
TCA9555_GPIO_Driver::TCA9555_GPIO_Driver(const uint8_t addr,
		I2C_HandleTypeDef *i2c):_i2caddr(addr), _i2c(i2c){}

void TCA9555_GPIO_Driver::begin(){// Determine previous state of that config registers
	uint8_t oldMode;
	HAL_I2C_Mem_Read(_i2c, _i2caddr, CR_CFG0, 1, &oldMode, 1, 10);
}

/*!
 * @brief  Sets the direction for pin specified by pinNum
 * @param  pinNum the pin number to access (0-15);
 * @param  mode OUTPUT or INPUT
 */
void TCA9555_GPIO_Driver::pinMode(uint8_t pinNum, uint8_t mode){
	// Determine config register to use
	uint8_t reg;
	if(pinNum < 8)
		reg = CR_CFG0;
	else{
		reg = CR_CFG1;
		pinNum -= 8;
	}

	// Determine previous state of that config registers
	uint8_t oldMode;
	HAL_I2C_Mem_Read(_i2c, _i2caddr, reg, 1, &oldMode, 1, 10);

	// Modify the bit to the new mode value
	uint8_t mask = 1 << pinNum;
	uint8_t newMode = (oldMode & ~mask) | ((mode << pinNum) & mask);

	// send config command
	HAL_I2C_Mem_Write(_i2c, _i2caddr, reg, 1, &newMode, 1, 10);
}

/*!
 * @brief  Sets the polarity for pin specified by pinNum
 * @param  pinNum the pin number to access (0-15);
 * @param  pol NORMAL or INV
 */
void TCA9555_GPIO_Driver::pinPol(uint8_t pinNum, uint8_t pol){
	// Determine config register to use
	uint8_t reg;
	if (pinNum < 8)
		reg = CR_INV0;
	else {
		reg = CR_INV0;
		pinNum -= 8;
	}

	if(pol > 0) pol = 1;
	else pol = 0;

	// Determine previous state of that config registers
	uint8_t oldMode;
	HAL_I2C_Mem_Read(_i2c, _i2caddr, reg, 1, &oldMode, 1, 10);

	// Modify the bit to the new mode value
	uint8_t mask = 1 << pinNum;
	uint8_t newMode = (oldMode & ~mask) | ((pol << pinNum) & mask);

	// send config command
	HAL_I2C_Mem_Write(_i2c, _i2caddr, reg, 1, &newMode, 1, 10);
}

/*!
 * @brief  Sets the output state of an output enabled pin
 * @param  pinNum the pin number to set (0-15)
 * @param  value HIGH or LOW
 */
void TCA9555_GPIO_Driver::setOutputState(uint8_t pinNum, uint8_t value){
	// Determine config register to use
	uint8_t reg;
	if (pinNum < 8)
		reg = CR_OUT0;
	else {
		reg = CR_OUT1;
		pinNum -= 8;
	}

	if(value > 0) value = 1;
	else value = 0;

	// Determine previous state of that output registers
	uint8_t oldMode;
	HAL_I2C_Mem_Read(_i2c, _i2caddr, reg, 1, &oldMode, 1, 10);

	// Modify the bit to the new mode value
	uint8_t mask = 1 << pinNum;
	uint8_t newMode = (oldMode & ~mask) | ((value << pinNum) & mask);

	// send port value back
	HAL_I2C_Mem_Write(_i2c, _i2caddr, reg, 1, &newMode, 1, 10);
}

/*!
 * @brief  Gets the value of an input enabled pin
 * @param  pinNum the pin number to get (0-15)
 */
uint8_t TCA9555_GPIO_Driver::getInputState(uint8_t pinNum){
	uint8_t reg;
	if (pinNum < 8)
		reg = CR_IN0;
	else {
		reg = CR_IN1;
		pinNum -= 8;
	}

	uint8_t portVal;
	HAL_I2C_Mem_Read(_i2c, _i2caddr, reg, 1, &portVal, 1, 10);

	return (portVal & (1 << pinNum)) >> pinNum;
}



