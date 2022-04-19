/*
 * AS5048A.cpp
 *
 *  Created on: Feb 22, 2020
 *      Author: tdubuke
 */

#include "AS5048A.h"
#include "cmsis_os.h"
#include <string.h>

AS5048A::AS5048A(uint16_t nucleoAddress, SPI_HandleTypeDef *hspi, uint8_t sizeOfDaisy, uint8_t numInDaisy):
	Sensor(nucleoAddress), _hspi(hspi), _sizeOfDaisy(sizeOfDaisy), _numInDaisy(numInDaisy) {}

AS5048A::~AS5048A() {}

void AS5048A::transmitRecieveData(uint8_t *txData, uint8_t *rxData, uint16_t size){
	HAL_SPI_TransmitReceive(_hspi, txData, rxData, size, 20);
}


uint8_t AS5048A::calcEvenParity(uint16_t value){
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

void AS5048A::swReset(){
	uint16_t opCode = CLEAR_ERROR;
	opCode |= 0x4000;
	opCode |= ((uint16_t)calcEvenParity(opCode) << 15);

	uint8_t transmitBuffer[2 * _sizeOfDaisy];
	uint8_t recieveBuffer[2 * _sizeOfDaisy];

	uint8_t lsB = opCode;
	uint8_t msB = opCode >> 8;
	for(uint8_t i = 0; i < _sizeOfDaisy; i++){
		memcpy(&transmitBuffer[i * 2], &msB, 1);
		memcpy(&transmitBuffer[(i * 2) + 1], &lsB, 1);
	}

	transmitRecieveData(transmitBuffer, recieveBuffer, 2 * _sizeOfDaisy);

	uint16_t data = recieveBuffer[_numInDaisy * 2] << 8 | recieveBuffer[(_numInDaisy * 2) + 1];

	lastError = data & 0x0007;
}

float AS5048A::getMagnitude(){
	uint16_t opCode = DIAGNOSTICS;
	opCode |= 0x4000;
	opCode |= ((uint16_t)calcEvenParity(opCode) << 15);

	uint8_t transmitBuffer[2];
	transmitBuffer[0] = (opCode >> 8) & 0xFF;
	transmitBuffer[1] = opCode & 0xFF;

	uint8_t recieveBuffer[2];

	transmitRecieveData(transmitBuffer, recieveBuffer, 2);

	uint16_t data = recieveBuffer[0] << 8 | recieveBuffer[1];

	return (float)data;
}

float AS5048A::getValue(){
	uint16_t opCode = ANGLE;
	opCode |= 0x4000;
	opCode |= ((uint16_t)calcEvenParity(opCode) << 15);

	uint8_t transmitBuffer[2 * _sizeOfDaisy];
	uint8_t recieveBuffer[2 * _sizeOfDaisy];

	for(uint8_t i = 0; i < _sizeOfDaisy; i++){
		memcpy(&transmitBuffer[i * 2], &opCode, 2);
	}

	transmitRecieveData(transmitBuffer, recieveBuffer, 2 * _sizeOfDaisy);

	uint16_t data = recieveBuffer[_numInDaisy * 2] << 8 | recieveBuffer[(_numInDaisy * 2) + 1];

	if(data & 0x4000){
		swReset();
		return -1;
	}

	return (float)(data & ANGLE) * AS_RADS_PER_TICK;
}


