/*
 * Actuator.h
 *
 *  Created on: Nov 9, 2019
 *      Author: tdubuke
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "main.h"
#include "Sensor.h"

class Actuator {
public:
	Actuator();
	Actuator(uint16_t nucleoAddress);

	// position = radians
	virtual void move(float position, float speed) = 0;
	virtual void torque(bool isOn) = 0;

private:
	uint16_t _nucleoAddress;
};

#endif /* ACTUATOR_H_ */
