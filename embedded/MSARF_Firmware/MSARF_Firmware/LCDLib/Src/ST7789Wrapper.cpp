/*
 * ST7789Wrapper.cpp
 *
 *  Created on: Dec 1, 2019
 *      Author: tdubuke
 */

#include "ST7789Wrapper.h"
#include "ST7789.h"

extern "C"{

	ST7789* ST7789_create(uint16_t *dispPtr, uint16_t dispSize, SPI_HandleTypeDef *_hspi, GPIO_TypeDef *gpio, uint16_t DCX, uint16_t RSVD){
		return new ST7789(dispPtr, dispSize, _hspi, gpio, DCX, RSVD);
	}

	void ST7789_delete(ST7789 *obj){
		delete(obj);
	}

	void ST7789_displayInit(ST7789 *obj, uint8_t *addr, uint16_t nucleo_addr){
		obj->displayInit(addr, nucleo_addr);
	}

//	uint16_t ST7789_getAddr(ST7789 *obj){
//		return obj->_nucleoAddress;
//	}

	void ST7789_writeScreen(ST7789 *obj, uint16_t *t_screen_buffer){
		obj->writeScreen(t_screen_buffer);
	}

	void ST7789_writeRect(ST7789 *obj, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *t_screen_buffer, uint16_t size){
		obj->writeRect(x, y, width, height, t_screen_buffer, size);
	}
}

