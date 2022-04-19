/*
 * GPIOBank.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef GPIOBANK_H_
#define GPIOBANK_H_

#include "main.h"
#include "TCA9555_GPIO_Driver.h"

class GPIOBank {
public:
	GPIOBank(uint8_t numDr);
	void addDriver(uint8_t addr, I2C_HandleTypeDef *i2c);

	int getNewChannel(uint8_t preferenceChannel);

	void pinMode(uint8_t channel, uint8_t mode);

	void setOutput(uint8_t channel, uint16_t val);
	uint8_t getInput(uint8_t channel);

	uint8_t getNumChannels();

	void clearChannels();

private:
	TCA9555_GPIO_Driver *_GPIODrivers[4];

	uint8_t numDrivers = 0;

	// 1 means available, 0 means unavailable or not initialized
	uint64_t _availChannelMask = 0;
};

#endif /* GPIOBANK_H_ */
