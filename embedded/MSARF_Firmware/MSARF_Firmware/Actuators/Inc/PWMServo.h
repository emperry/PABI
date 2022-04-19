/*
 * PWMServo.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef PWMSERVO_H_
#define PWMSERVO_H_

#include "Actuator.h"
#include "PWMBank.h"

class PWMServo: public Actuator {
public:
	PWMServo(PWMBank *pwmBank, float maxRotation, uint16_t nucleoAddress);
	int begin(uint16_t preferenceChannel);

	void move(float position, float speed = 0);
	void torque(bool isOn);
private:
	PWMBank *_PWMBank;
	uint16_t _PWMPin;
	float _maxRotAngle;
};

#endif /* PWMSERVO_H_ */
