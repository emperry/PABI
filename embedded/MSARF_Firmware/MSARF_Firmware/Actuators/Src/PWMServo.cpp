/*
 * PWMServo.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#include "PWMServo.h"

PWMServo::PWMServo(PWMBank *pwmBank, float maxRotation, uint16_t nucleoAddress):
	Actuator(nucleoAddress), _PWMBank(pwmBank), _maxRotAngle(maxRotation){}

/*!
 * @Brief init the servo by having it grab a pin from the pwm bank
 * @param preferenceChannel The pin that would be preferred
 */
int PWMServo::begin(uint16_t preferenceChannel){
	uint16_t pin = _PWMBank->getNewChannel(preferenceChannel);

	// Get new channel
	if(pin == -1){
		return -1;
	}else{
		_PWMPin = pin;
	}

	_PWMBank->writeMicroseconds(_PWMPin, 1500);

	return 0;
}

/*!
 * @brief send a command to the Servo to move to a new position
 * @param position New position given in 0-max
 * @param speed Speed to achieve, since PWM servos do not have a speed value
 * this setting will be ignored here
 */
void PWMServo::move(float position, float speed){
	float servoPerc = position / _maxRotAngle;

	if(servoPerc > 1) servoPerc = 1;

	// redefine percentage between 1000 and 2000 micros
	float micros = (1000 * servoPerc) + 1000;

	_PWMBank->writeMicroseconds(_PWMPin, (uint16_t)micros);
}

void PWMServo::torque(bool isOn){
	if(isOn){
		_PWMBank->writeMicroseconds(_PWMPin, 1500);
	}else{
		_PWMBank->writeMicroseconds(_PWMPin, 1);
	}
}
