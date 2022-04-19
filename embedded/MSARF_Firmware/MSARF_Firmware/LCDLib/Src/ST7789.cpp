/*
 * ST7789.cpp
 *
 *  Created on: Nov 26, 2019
 *      Author: tdubuke
 */

#include "ST7789.h"
#include "ST7789Wrapper.h"
#include "cmsis_os.h"

extern volatile bool oneTransmitting;
extern volatile bool threeTransmitting;

ST7789::ST7789(uint16_t *dispPtr, uint16_t dispSize, SPI_HandleTypeDef *hspi, GPIO_TypeDef *gpio, uint16_t DCX, uint16_t RSVD) {
	displayMemory = dispPtr;
	displayMemorySize = dispSize;
	_hspi = hspi;
	_gpio = gpio;
	_DCX = DCX;
	_RSVD = RSVD;

//	_nucleoAddress = 0;
}

ST7789::~ST7789() {
	// TODO Auto-generated destructor stub
}

uint16_t ST7789::cords_to_1d(uint16_t x, uint16_t y){
	return LCD_HEIGHT * x + y;
}

void ST7789::SPI_WRITE32(uint32_t bytes) {
	uint8_t byte_array[4];
	byte_array[0] = bytes >> 24;
	byte_array[1] = bytes >> 16;
	byte_array[2] = bytes >> 8;
	byte_array[3] = bytes;
	HAL_SPI_Transmit(_hspi, byte_array, 4, 100);
}

void ST7789::SPI_Transmit32(SPI_HandleTypeDef *hspi, uint8_t *pData, uint32_t Size, uint32_t Timeout){
	HAL_SPI_Transmit(hspi, (uint8_t *) pData, Size / 2, Timeout);
	pData = pData + (Size / 2);
	HAL_SPI_Transmit(hspi, (uint8_t *) pData, Size / 2, Timeout);
}

void ST7789::SPI_Transmit16(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout){
	HAL_SPI_Transmit(hspi, pData, Size, Timeout);
}

/**
 * This is for commands with parameters
 */
void ST7789::sendCommand(uint8_t commandByte, uint8_t *dataBytes, uint8_t numDataBytes) {
	HAL_GPIO_WritePin(_gpio, _DCX, GPIO_PIN_RESET); //Command mode
	HAL_SPI_Transmit(_hspi, &commandByte, 1, 100); // Send the command byte
	HAL_GPIO_WritePin(_gpio, _DCX, GPIO_PIN_SET); //Command mode off

	for (int i = 0; i < numDataBytes; i++) {
		HAL_SPI_Transmit(_hspi, dataBytes, numDataBytes, 100); // Send the data bytes
		dataBytes++;
	}
}

/**
 * This is for commands without parameters
 */
void ST7789::writeCommand(uint8_t cmd) {
	HAL_GPIO_WritePin(_gpio, _DCX, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_hspi, &cmd, sizeof(cmd), 100);

	HAL_GPIO_WritePin(_gpio, _DCX, GPIO_PIN_SET);
}

void ST7789::displayInit(uint8_t *addr, uint16_t nucleo_addr) {
//	_nucleoAddress = nucleo_addr;

	uint8_t numCommands, cmd, numArgs;
	uint16_t ms;

	numCommands = *(addr++);   // Number of commands to follow
	while (numCommands--) {                 // For each command...
		cmd = *(addr++);         // Read command
		numArgs = *(addr++);    // Number of args to follow
		ms = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
		numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
		sendCommand(cmd, addr, numArgs);
		addr += numArgs;

		if (ms) {
			ms = *(addr++); // Read post-command delay time (ms)
			if (ms == 255)
				ms = 500;     // If 255, delay for 500 ms
			HAL_Delay(ms);
		}
	}
}

void ST7789::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	uint32_t xa = ((uint32_t) x << 16) | (x + w - 1);
	uint32_t ya = ((uint32_t) y << 16) | (y + h - 1);

	writeCommand(ST77XX_CASET); // Column addr set
	SPI_WRITE32(xa);

	writeCommand(ST77XX_RASET); // Row addr set
	SPI_WRITE32(ya);

	writeCommand(ST77XX_RAMWR); // write to RAM
}

void ST7789::writePixel(int16_t x, int16_t y, uint16_t color) {
	//setAddrWindow(x, y, 1, 1);
	uint16_t index = cords_to_1d(x, y);
	uint16_t * cp_displayMemory = displayMemory + index;
	*cp_displayMemory = color;
	//uint8_t color_array[2] = { color & 0xFF, color >> 8 };
	//HAL_SPI_Transmit(hspi, color_array, 2, 100);
}

void ST7789::writeRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *t_screen_buffer, uint16_t size) {
	setAddrWindow(x, y, width, height);

	if(_hspi->Instance == SPI1){
		while(oneTransmitting){}
		oneTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) t_screen_buffer, size);
	}else if(_hspi->Instance == SPI3){
		while(threeTransmitting){}
		threeTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) t_screen_buffer, size);
	}
}

void ST7789::writeScreen(uint16_t *t_screen_buffer) {
	uint16_t *first_buffer = t_screen_buffer;
	uint16_t *second_buffer = t_screen_buffer + displayMemorySize / 2;

	setAddrWindow(0, 0, 240, 240);

	if(_hspi->Instance == SPI1){
		while(oneTransmitting){}
		oneTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) first_buffer, displayMemorySize);

		while(oneTransmitting){}
		oneTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) second_buffer, displayMemorySize);
	}else if(_hspi->Instance == SPI3){
		while(threeTransmitting){}
		threeTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) first_buffer, displayMemorySize);

		while(threeTransmitting){}
		threeTransmitting = true;
		HAL_SPI_Transmit_DMA(_hspi, (uint8_t *) second_buffer, displayMemorySize);
	}
}

void ST7789::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	setAddrWindow(x, y, w, h);

	uint16_t *buffer = (uint16_t*)malloc(sizeof(int16_t)*h*w);
	uint16_t *buff_ptr = buffer;
	for(int i = 0; i < h*w; i++){
		*buff_ptr = color;
		buff_ptr++;
	}

	SPI_Transmit32(_hspi, (uint8_t *)buffer, sizeof(int16_t)*h*w, 200);
	free(buffer);
}

void ST7789::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    writePixel(x0  , y0+r, color);
    writePixel(x0  , y0-r, color);
    writePixel(x0+r, y0  , color);
    writePixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        writePixel(x0 + x, y0 + y, color);
        writePixel(x0 - x, y0 + y, color);
        writePixel(x0 + x, y0 - y, color);
        writePixel(x0 - x, y0 - y, color);
        writePixel(x0 + y, y0 + x, color);
        writePixel(x0 - y, y0 + x, color);
        writePixel(x0 + y, y0 - x, color);
        writePixel(x0 - y, y0 - x, color);
    }
}

void ST7789::writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		_swap_int16_t(x0, y0);
		_swap_int16_t(x1, y1);
	}

	if (x0 > x1) {
		_swap_int16_t(x0, x1);
		_swap_int16_t(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0<=x1; x0++) {
		if (steep) {
			writePixel(y0, x0, color);
		} else {
			writePixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

void ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    writeLine(x, y, x, y+h-1, color);
}

void ST7789::writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    drawFastVLine(x, y, h, color);
}

void ST7789::fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color) {
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) writeFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) writeFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) writeFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) writeFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void ST7789::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    writeFastVLine(x0, y0-r, 2*r+1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}

