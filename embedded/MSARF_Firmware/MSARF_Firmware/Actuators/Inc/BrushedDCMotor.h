/*
 * BrushedDCMotor.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef BRUSHEDDCMOTOR_H_
#define BRUSHEDDCMOTOR_H_

#include "Actuator.h"
#include "PWMBank.h"
#include "GPIOBank.h"

#include <stdlib.h>

class BrushedDCMotor: public Actuator {
public:
	BrushedDCMotor(PWMBank *pwmBank, GPIOBank *gpioBank, uint16_t nucleoAddress);
	uint8_t begin(uint16_t pwmChannel, uint16_t dirChannel, uint16_t enableChannel, uint16_t faultChannel);

	void move(float position, float speed);
	void torque(bool isOn);
private:
	PWMBank *_PWMBank;
	GPIOBank *_GPIOBank;

	uint16_t _PWMPin;
	uint16_t _DirPin;
	uint16_t _FaultPin;
	uint16_t _EnablePin;
};

#endif /* BRUSHEDDCMOTOR_H_ */
