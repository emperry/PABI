/*
 * AS5048A.h
 *
 *  Created on: Feb 22, 2020
 *      Author: tdubuke
 */

#ifndef AS5048A_H_
#define AS5048A_H_

#define CLEAR_ERROR 	0x0001
#define PROG_CTL		0x0003
#define DIAGNOSTICS		0x3FFD
#define MAGNITUDE		0x3FFE
#define ANGLE			0x3FFF

#define ERROR_FLAG		0x4000
#define PARITY_ERROR	0x0004
#define COMMAND_ERROR	0x0002
#define FRAMING_ERROR	0x0001

#define AS_RADS_PER_TICK	0.00038223

#include "main.h"
#include "Sensor.h"

class AS5048A: public Sensor {
public:
	AS5048A(uint16_t nucleoAddress, SPI_HandleTypeDef *hspi, uint8_t sizeOfDaisy, uint8_t numInDaisy);
	virtual ~AS5048A();

	void transmitRecieveData(uint8_t *txData, uint8_t *rxData, uint16_t size);
	void swReset();

	uint8_t calcEvenParity(uint16_t value);

	float getMagnitude();
	float getValue();

private:
	SPI_HandleTypeDef *_hspi;
	uint8_t _sizeOfDaisy = 0;
	uint8_t _numInDaisy = 0;

	uint8_t lastError = 0;
};

#endif /* AS5048A_H_ */
