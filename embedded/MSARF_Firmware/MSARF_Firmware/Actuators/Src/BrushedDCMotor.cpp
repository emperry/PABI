/*
 * BrushedDCMotor.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#include "BrushedDCMotor.h"

BrushedDCMotor::BrushedDCMotor(PWMBank *pwmBank, GPIOBank *gpioBank, uint16_t nucleoAddress):
	Actuator(nucleoAddress), _PWMBank(pwmBank), _GPIOBank(gpioBank) {}

/*!
 * @Brief init the servo by having it grab a pin from the pwm bank
 * @param pwmChannel The pin that would be preferred
 * @param dirChannel The pin that will control the dir input to the motor driver
 * @param faultChannel The pin that will trigger on a motor driver fault
 */
uint8_t BrushedDCMotor::begin(uint16_t pwmChannel, uint16_t dirChannel, uint16_t enableChannel, uint16_t faultChannel){
	int pwmPin = _PWMBank->getNewChannel(pwmChannel);
	int dirPin = _GPIOBank->getNewChannel(dirChannel);
	int enPin = _GPIOBank->getNewChannel(enableChannel);
	int faultPin = _GPIOBank->getNewChannel(faultChannel);

	// Get new pwm channel
	if(pwmPin == -1){
		return -1;
	}else{
		_PWMPin = pwmPin;
	}

	// Get new dir channel
	if(dirPin == -1){
		return -1;
	}else{
		_DirPin = dirPin;
		_GPIOBank->pinMode(_DirPin, OUTPUT);
	}

	// Get new en channel
	if(enPin == -1){
		return -1;
	}else{
		_EnablePin = enPin;
		_GPIOBank->pinMode(_EnablePin, OUTPUT);
		_GPIOBank->setOutput(enPin, 0);
	}

	// Get new fault channel
	if(faultPin == -1){
		return -1;
	}else{
		_FaultPin = faultPin;
		_GPIOBank->pinMode(_FaultPin, INPUT);
	}

	return 0;
}

/*!
 * @brief Move the brushed dc motor using pwm and gpio outputs
 * @param speed Speed -100% to 100%
 * @param position Not used
 */
void BrushedDCMotor::move(float position, float speed){
	if(speed >= 0){
		_GPIOBank->setOutput(_DirPin, 1);
	}else{
		_GPIOBank->setOutput(_DirPin, 0);
	}

	float posSpeed = abs(speed);
	_PWMBank->setDutyCycle(_PWMPin, (uint16_t)posSpeed);
}

void BrushedDCMotor::torque(bool isOn){}
