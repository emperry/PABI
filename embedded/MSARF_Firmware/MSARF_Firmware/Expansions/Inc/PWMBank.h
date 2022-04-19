/*
 * PWMBank.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef PWMBANK_H_
#define PWMBANK_H_

#include <PCA9685_PWM_Driver.h>
#include "stm32h7xx_hal.h"

class PWMBank {
public:
	PWMBank();

	void setPWM(uint8_t channel, uint16_t on, uint16_t off);
	void writeMicroseconds(uint8_t channel, uint16_t microseconds);
	void setDutyCycle(uint8_t channel, uint8_t duty);

	void addDriver(uint8_t addr, I2C_HandleTypeDef *i2c);

	int getNewChannel(uint8_t preferenceChannel);
	uint8_t getNumChannels();

	void clearChannels();
private:
	PCA9685_PWM_Driver *_PWMDrivers[2];
	uint8_t numDrivers = 0;

	// 1 means available, 0 means unavailable or uninitialized
	uint64_t _availChannelMask = 0;
};

#endif /* PWMBANK_H_ */
