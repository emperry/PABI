/*
 * BrushlessMotor.cpp
 *
 *  Created on: Feb 18, 2020
 *      Author: tdubuke
 */

#include "BrushlessMotor.h"

BrushlessMotor::BrushlessMotor(PWMBank *pwmBank, uint16_t nucleoAddress): Actuator(nucleoAddress), _PWMBank(pwmBank) {}
BrushlessMotor::~BrushlessMotor() {}

uint8_t BrushlessMotor::begin(uint16_t pwmChannel){
	int pwmPin = _PWMBank->getNewChannel(pwmChannel);

	// Get new pwm channel
	if(pwmPin == -1){
		return -1;
	}else{
		_PWMPin = pwmPin;
	}

	_PWMBank->writeMicroseconds(pwmChannel, 1500);

	return 0;
}

// Speed proportional to percent of full
// 100% = 3686
// 0% = 2047
// -100% = 409
void BrushlessMotor::move(float position, float speed){
	float absSpeed = abs(speed * 5);
	float micros;

	if(speed > 0){
		micros = 1500 + absSpeed;
	}else if(speed < 0){
		micros = 1500 - absSpeed;
	}else{
		micros = 1500;
	}

	_PWMBank->writeMicroseconds(_PWMPin, micros);
}

void BrushlessMotor::torque(bool isOn){
	if(!isOn){
		_PWMBank->writeMicroseconds(_PWMPin, 1);
	}else{
		_PWMBank->writeMicroseconds(_PWMPin, 1500);
	}
}

