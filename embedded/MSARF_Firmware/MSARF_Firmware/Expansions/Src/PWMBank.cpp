/*
 * PWMBank.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#include "PWMBank.h"
#include "cmsis_os.h"
#include <string.h>

PWMBank::PWMBank(){}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin in the bank without having to
 *   deal with on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  channel One of the PWM output pins, from 0 to (16 * _numOfChannels)
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
void PWMBank::setPWM(uint8_t channel, uint16_t on, uint16_t off){
	// Tried to index into a chip that does not exist, throw error
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;

	_PWMDrivers[chipIndex]->setPWM(channelIndex, on, off);
}

void PWMBank::writeMicroseconds(uint8_t channel, uint16_t microseconds){
	// Tried to index into a chip that does not exist, throw error
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;

	_PWMDrivers[chipIndex]->writeMicroseconds(channelIndex, microseconds);
}

void PWMBank::setDutyCycle(uint8_t channel, uint8_t duty){
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;

	float del = ((float)duty * 4095.0)/100.0;
	_PWMDrivers[chipIndex]->setPWM(channelIndex, 0, (uint16_t)del);
}

/*!
 * @brief Function to add new driver chip to the bank with a unique
 * address
 * @param addr the I2C address of the new chip
 * @param i2c the I2C handler to access the bus
 */
void PWMBank::addDriver(uint8_t addr, I2C_HandleTypeDef *i2c){
	uint64_t newMask = 0xffff;
	newMask = newMask << (numDrivers * 16);
	_availChannelMask |= newMask;

	PCA9685_PWM_Driver *driver_ptr = (PCA9685_PWM_Driver *)pvPortMalloc(sizeof(PCA9685_PWM_Driver));
	PCA9685_PWM_Driver driver(addr, i2c);
	memcpy(driver_ptr, &driver, sizeof(PCA9685_PWM_Driver));

	driver_ptr->begin();
	driver_ptr->setPWMFreq(50);

	_PWMDrivers[numDrivers] = driver_ptr;

	numDrivers++;
}

/*!
 * @brief get the next available channel
 * @param preferenceChannel The channel you would prefer to get for
 * hardware limitation reasons
 * @return Channel number if given, -1 if unavailable
 */
int PWMBank::getNewChannel(uint8_t preferenceChannel){
	uint64_t channelMask = 1 << preferenceChannel;
	if((channelMask & _availChannelMask) >= 1){
		_availChannelMask = _availChannelMask & ~channelMask;
		return preferenceChannel;
	}

	return -1;
}

uint8_t PWMBank::getNumChannels(){
	return numDrivers * 16;
}

void PWMBank::clearChannels(){
	_availChannelMask = 0;
	for(uint8_t i = 0; i < numDrivers; i++){
		_availChannelMask = _availChannelMask << 16 | 0xFFFF;
	}
}
