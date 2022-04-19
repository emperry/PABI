/*
 * Encoder.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: tdubuke
 */

#include "Encoder.h"

Encoder::Encoder(LPTIM_HandleTypeDef *lptim, TIM_HandleTypeDef *tim, uint16_t ticksPerRev, uint16_t nucleoAddress):
	Sensor(nucleoAddress), _lptim(lptim), _tim(tim), _ticksPerRev(ticksPerRev){}

Encoder::~Encoder(){}

void Encoder::initEncoder(){
	if(_lptim != 0){
		HAL_LPTIM_Encoder_Start(_lptim, _ticksPerRev);
	}else if(_tim != 0){
		HAL_TIM_Encoder_Start(_tim, TIM_CHANNEL_ALL);
	}
}

float Encoder::getValue(){
	float count;
	if(_lptim != 0){
		count = (float)HAL_LPTIM_ReadCounter(_lptim);
	}else if(_tim != 0){
		count = _tim->Instance->CNT;
	}else{
		return 0;
	}

	count = count / (float)_ticksPerRev;
	return count * 6.28319;
}
