/*
 * GPIOBank.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#include "GPIOBank.h"
#include "cmsis_os.h"
#include <string.h>

GPIOBank::GPIOBank(uint8_t numDr):numDrivers(numDr) {}

void GPIOBank::addDriver(uint8_t addr, I2C_HandleTypeDef *i2c) {
	uint64_t newMask = 0xffff;
	newMask = newMask << (numDrivers * 16);
	_availChannelMask |= newMask;

	TCA9555_GPIO_Driver *driver_ptr = (TCA9555_GPIO_Driver *)pvPortMalloc(sizeof(TCA9555_GPIO_Driver));
	TCA9555_GPIO_Driver driver(addr, i2c);
	memcpy(driver_ptr, &driver, sizeof(TCA9555_GPIO_Driver));

	driver_ptr->begin();

	_GPIODrivers[numDrivers] = driver_ptr;

	numDrivers++;
}

/*!
 * @brief Function to get pin in the GPIO Bank
 * @param preferenceChannel The channel that is asked for by the device
 * @return Channel number if success or -1 if unavailable
 */
int GPIOBank::getNewChannel(uint8_t preferenceChannel){
	uint64_t channelMask = 1 << preferenceChannel;
	if ((channelMask & _availChannelMask) >= 1) {
		_availChannelMask = _availChannelMask & ~channelMask;
		return preferenceChannel;
	}

	return -1;
}

void GPIOBank::pinMode(uint8_t channel, uint8_t mode){
	// Have not added any chips yet, throw error
	if(numDrivers == 0){
		Error_Handler();
	}

	// Tried to index into a chip that does not exist, throw error
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;
	if(chipIndex > (numDrivers - 1)){
		Error_Handler();
	}

	_GPIODrivers[chipIndex]->pinMode(channelIndex, mode);
}

void GPIOBank::setOutput(uint8_t channel, uint16_t val){
	// Have not added any chips yet, throw error
	if (numDrivers == 0) {
		Error_Handler();
	}

	// Tried to index into a chip that does not exist, throw error
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;
	if (chipIndex > (numDrivers - 1)) {
		Error_Handler();
	}

	_GPIODrivers[chipIndex]->setOutputState(channelIndex, val);
}

uint8_t GPIOBank::getInput(uint8_t channel){
	// Have not added any chips yet, throw error
	if (numDrivers == 0) {
		Error_Handler();
	}

	// Tried to index into a chip that does not exist, throw error
	uint8_t chipIndex = channel / 16;
	uint8_t channelIndex = channel % 16;
	if (chipIndex > (numDrivers - 1)) {
		Error_Handler();
	}

	return _GPIODrivers[chipIndex]->getInputState(channelIndex);
}

uint8_t GPIOBank::getNumChannels(){
	return numDrivers * 16;
}

void GPIOBank::clearChannels(){
	_availChannelMask = 0;
	for(uint8_t i = 0; i < numDrivers; i++){
		_availChannelMask = _availChannelMask << 16 | 0xFFFF;
	}
}
