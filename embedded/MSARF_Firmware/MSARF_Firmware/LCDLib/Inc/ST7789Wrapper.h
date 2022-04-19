/*
 * ST7789Wrapper.h
 *
 *  Created on: Dec 1, 2019
 *      Author: tdubuke
 */

#ifndef ST7789WRAPPER_H_
#define ST7789WRAPPER_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

#define LCD_WIDTH 240
#define LCD_HEIGHT 240
#define IMAGE_WIDTH 80
#define IMAGE_HEIGHT 80

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST77XX_NOP        0x00
#define ST77XX_SWRESET    0x01
#define ST77XX_RDDID      0x04
#define ST77XX_RDDST      0x09

#define ST77XX_SLPIN      0x10
#define ST77XX_SLPOUT     0x11
#define ST77XX_PTLON      0x12
#define ST77XX_NORON      0x13

#define ST77XX_INVOFF     0x20
#define ST77XX_INVON      0x21
#define ST77XX_DISPOFF    0x28
#define ST77XX_DISPON     0x29
#define ST77XX_CASET      0x2A
#define ST77XX_RASET      0x2B
#define ST77XX_RAMWR      0x2C
#define ST77XX_RAMRD      0x2E

#define ST77XX_PTLAR      0x30
#define ST77XX_TEOFF      0x34
#define ST77XX_TEON       0x35
#define ST77XX_MADCTL     0x36
#define ST77XX_COLMOD     0x3A

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1      0xDA
#define ST77XX_RDID2      0xDB
#define ST77XX_RDID3      0xDC
#define ST77XX_RDID4      0xDD

// Some ready-made 16-bit ('565') color settings:
#define	ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define	ST77XX_RED        0x1F00
#define	ST77XX_GREEN      0xE007
#define	ST77XX_BLUE       0x00F8

// Some ready-made 16-bit ('1555') color settings:
#define	ST77XX_1555_BLACK      0x8000
#define ST77XX_1555_WHITE      0xFFFF
#define	ST77XX_1555_RED        0xFC00
#define	ST77XX_1555_GREEN      0x83E0
#define	ST77XX_1555_BLUE       0x801F

typedef struct ST7789 ST7789;

ST7789* ST7789_create(uint16_t *dispPtr, uint16_t dispSize, SPI_HandleTypeDef *_hspi, GPIO_TypeDef *gpio, uint16_t DCX, uint16_t RSVD);
void ST7789_delete(ST7789 *obj);
void ST7789_displayInit(ST7789 *obj, uint8_t *addr, uint16_t nucleo_addr);
uint16_t ST7789_getAddr(ST7789 *obj);
void ST7789_writeScreen(ST7789 *obj, uint16_t *t_screen_buffer);
void ST7789_writeRect(ST7789 *obj, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *t_screen_buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* ST7789WRAPPER_H_ */
