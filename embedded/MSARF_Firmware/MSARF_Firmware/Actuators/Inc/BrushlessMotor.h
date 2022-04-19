/*
 * BrushlessMotor.h
 *
 *  Created on: Feb 18, 2020
 *      Author: tdubuke
 */

#ifndef BRUSHLESSMOTOR_H_
#define BRUSHLESSMOTOR_H_

#include "Actuator.h"
#include "PWMBank.h"
#include "cmsis_os.h"

class BrushlessMotor : public Actuator{
public:
	BrushlessMotor(PWMBank *pwmBank, uint16_t nucleoAddress);
	virtual ~BrushlessMotor();

	uint8_t begin(uint16_t pwmChannel);

	void move(float position, float speed);
	void torque(bool isOn);
private:
	PWMBank *_PWMBank;

	uint16_t _PWMPin;
};

#endif /* BRUSHLESSMOTOR_H_ */
