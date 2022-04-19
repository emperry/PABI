/*
 * Encoder.h
 *
 *  Created on: Feb 4, 2020
 *      Author: tdubuke
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "Sensor.h"
#include "main.h"

class Encoder: public Sensor {
public:
	Encoder(LPTIM_HandleTypeDef *lptim, TIM_HandleTypeDef *tim, uint16_t ticksPerRev, uint16_t nucleoAddress);
	~Encoder();

	void initEncoder();

	float getValue();

private:
	LPTIM_HandleTypeDef *_lptim;
	TIM_HandleTypeDef *_tim;

	uint16_t _ticksPerRev;
};

#endif /* ENCODER_H_ */
