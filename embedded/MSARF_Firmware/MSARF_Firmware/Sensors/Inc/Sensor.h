/*
 * Sensor.h
 *
 *  Created on: Nov 12, 2019
 *      Author: tdubuke
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"

class Sensor {
public:
	Sensor();
	Sensor(uint16_t nucleoAddress);
	virtual float getValue() = 0;

private:
	uint16_t _nucleoAddress;
};

#endif /* SENSOR_H_ */
