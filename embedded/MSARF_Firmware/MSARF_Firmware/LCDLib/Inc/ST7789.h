/*
 * ST7789.h
 *
 *  Created on: Nov 26, 2019
 *      Author: tdubuke
 */

#ifndef ST7789_H_
#define ST7789_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "main.h"

class ST7789 {
public:
	ST7789(uint16_t *dispPtr, uint16_t dispSize, SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t DCX, uint16_t RSVD);
	virtual ~ST7789();

	void sendCommand(uint8_t commandByte, uint8_t *dataBytes,
			uint8_t numDataBytes);
	void writeCommand(uint8_t cmd);
	void displayInit(uint8_t *addr, uint16_t nucleo_addr);
	void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
	void writePixel(int16_t x, int16_t y, uint16_t color);
	void writeScreen(uint16_t *t_screen_buffer);
	void writeRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *t_screen_buffer, uint16_t size);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
			uint16_t color);
	void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
	void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
	void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners,
			int16_t delta, uint16_t color);
	void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

	void SPI_WRITE32(uint32_t bytes);
	void SPI_Transmit32(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size,
			uint32_t Timeout);
	void SPI_Transmit16(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size,
			uint32_t Timeout);
	uint16_t cords_to_1d(uint16_t x, uint16_t y);

	uint16_t *displayMemory;
	uint16_t displayMemorySize;

	SPI_HandleTypeDef *_hspi;
	GPIO_TypeDef *_gpio;
	uint16_t _DCX;
	uint16_t _RSVD;
	///uint16_t _nucleoAddress;
};


#endif /* ST7789_H_ */
