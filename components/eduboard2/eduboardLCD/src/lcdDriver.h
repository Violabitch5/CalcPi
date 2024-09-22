#pragma once
#include "fontx.h"

#define RED			0xf800
#define GREEN			0x07e0
#define BLUE			0x001f
#define BLACK			0x0000
#define WHITE			0xffff
#define GRAY			0x8c51
#define YELLOW			0xFFE0
#define CYAN			0x07FF
#define PURPLE			0xF81F


#define DIRECTION0		    0
#define DIRECTION90		    1
#define DIRECTION180		2
#define DIRECTION270		3

typedef struct {
	uint16_t _width;
	uint16_t _height;
	uint16_t _offsetx;
	uint16_t _offsety;
	uint16_t _font_direction;
	uint16_t _font_fill;
	uint16_t _font_fill_color;
	uint16_t _font_underline;
	uint16_t _font_underline_color;
	int16_t _dc;
	int16_t _bl;
	spi_device_handle_t _SPIHandle;
} TFT_t;

typedef enum {rot_0, rot_90, rot_180, rot_270} rotation_t;

void delayMS(int ms);

void lcd_init();

void lcdSetupVScreen(rotation_t rotation);
void lcdUpdateVScreen();
void lcdClearVScreen();
uint16_t lcdGetWidth();
uint16_t lcdGetHeight();

void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color);
void lcdDrawMultiPixels(uint16_t x, uint16_t y, uint16_t size, uint16_t * colors);
void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDisplayOff();
void lcdDisplayOn();
void lcdFillScreen(uint16_t color);
void lcdDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color);
void lcdDrawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color);
void lcdDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color);
void lcdDrawArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color);
void lcdDrawFillArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color);
uint16_t rgb565_conv(uint16_t r, uint16_t g, uint16_t b);
int lcdDrawChar(FontxFile *fx, uint16_t x, uint16_t y, uint8_t ascii, uint16_t color);
int lcdDrawString(FontxFile *fx, uint16_t x, uint16_t y, char* ascii, uint16_t color);
int lcdDrawCode(FontxFile *fx, uint16_t x,uint16_t y,uint8_t code,uint16_t color);
void lcdSetFontDirection(uint16_t dir);
void lcdSetFontFill(uint16_t color);
void lcdUnsetFontFill();
void lcdSetFontUnderLine(uint16_t color);
void lcdUnsetFontUnderLine();
void lcdBacklightOff();
void lcdBacklightOn();
void lcdInversionOff();
void lcdInversionOn();

void lcdDrawDataUInt8(uint16_t x, uint16_t y, uint16_t width, uint8_t height, uint8_t min, uint8_t max, bool leftToRight, uint8_t *data, uint16_t color);
void lcdDrawDataInt8(uint16_t x, uint16_t y, uint8_t width, uint8_t height, int8_t min, int8_t max, int8_t* data, uint16_t color);