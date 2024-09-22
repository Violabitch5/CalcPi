#include "../../eduboard2.h"
#include "../eduboard2_lcd.h"

#include <driver/gpio.h>
#include "lcdDriver.h"
#ifdef CONFIG_LCD_ST7789
#include "st7789.h"
#endif
#ifdef CONFIG_LCD_ILI9488
#include "ili9488.h"
#endif

#define TAG "lcdDriver"

#ifdef CONFIG_ENABLE_LCD

#ifdef CONFIG_USE_DIFFUPDATE
	#define DIFFUPDATE_AREAWIDTH 16
	#define DIFFUPDATE_AREAHEIGHT 16

	#define DIFFUPDATE_AREASIZE_X CONFIG_WIDTH / DIFFUPDATE_AREAWIDTH
	#define DIFFUPDATE_AREASIZE_Y CONFIG_HEIGHT / DIFFUPDATE_AREAHEIGHT

	#define DIFFUPDATE_AREA_MAX DIFFUPDATE_AREASIZE_X * DIFFUPDATE_AREASIZE_Y
#endif

TFT_t dev;

struct
{
	bool enabled;
	uint16_t *data1;
#ifdef CONFIG_USE_DIFFUPDATE
	SemaphoreHandle_t diffupdate_lock;
	uint16_t *data2;
	uint8_t updatemap[DIFFUPDATE_AREA_MAX / 8];
#endif
	rotation_t rotation;
} vScreen;

// Helper Functions for memset

void *memset16(void *m, uint16_t val, size_t count)
{
    uint16_t *buf = m;

    while(count--) *buf++ = val;
    return m;
}
void *memset32(void *m, uint32_t val, size_t count)
{
    uint32_t *buf = m;

    while(count--) *buf++ = val;
    return m;
}
void *memsetXX(void *m, void *val, size_t size, size_t count)
{
    char *buf = m;

    while(count--)
    {
        memcpy(buf, val, size);
        buf += size;
    }
    return m;
}

// blocking delay function

void delayMS(int ms)
{
	int _ms = ms + (portTICK_PERIOD_MS - 1);
	TickType_t xTicksToDelay = _ms / portTICK_PERIOD_MS;
	// ESP_LOGD(TAG, "ms=%d _ms=%d portTICK_PERIOD_MS=%d xTicksToDelay=%d",ms,_ms,portTICK_PERIOD_MS,xTicksToDelay);
	vTaskDelay(xTicksToDelay);
}

// Init LCD

void lcd_init()
{

#ifdef CONFIG_LCD_ST7789
	st7789_spi_master_init(&dev, GPIO_MOSI, GPIO_SCK, GPIO_LCD_CS, GPIO_LCD_DC, GPIO_GENERAL_RESET, -1);
	st7789_init(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSET_WIDTH, CONFIG_OFFSET_HEIGHT);
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_spi_master_init(&dev, GPIO_MOSI, GPIO_SCK, GPIO_LCD_CS, GPIO_LCD_DC, GPIO_GENERAL_RESET, -1);
	ili9488_init(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSET_WIDTH, CONFIG_OFFSET_HEIGHT);
#endif
}

/*---------------------------------------------------------------------------------------------------------------------*/
/*   diffupdate Driver                                                                                                    */
/*                                                                                                                     */
/*   Uses Map for changed pixels                                                                                       */
/*                                                                                                                     */
/*                                                                                                                     */
/*---------------------------------------------------------------------------------------------------------------------*/

#ifdef CONFIG_USE_DIFFUPDATE
uint16_t diffupdate_convertCoordinateToArea(uint16_t x, uint16_t y)
{
	uint16_t _x = x / DIFFUPDATE_AREAWIDTH;
	uint16_t _y = y / DIFFUPDATE_AREAHEIGHT;
	uint16_t pos = _y * DIFFUPDATE_AREASIZE_X + _x;
	return pos;
}

void diffupdate_convertAreaPosToCoordinateArea(uint16_t areapos, uint16_t *x1, uint16_t *y1, uint16_t *x2, uint16_t *y2)
{
	uint16_t _y = 0;
	uint16_t _x = 0;
	uint16_t area_per_horizontal_line = DIFFUPDATE_AREASIZE_X;
	_y = areapos / area_per_horizontal_line;
	_x = (areapos - (_y * DIFFUPDATE_AREASIZE_X));
	*x1 = _x * DIFFUPDATE_AREAWIDTH;
	*y1 = _y * DIFFUPDATE_AREAHEIGHT;
	if(x2 != NULL) {
		*x2 = ((_x + 1) * DIFFUPDATE_AREAWIDTH) - 1;
		*y2 = ((_y + 1) * DIFFUPDATE_AREAHEIGHT) - 1;
	}
}

void copyVScreenArea(uint16_t x1, uint16_t y1, uint16_t sizex, uint16_t sizey, uint16_t *buffer)
{
	for (int _y = y1; _y < y1 + sizey; _y++)
	{
		for (int _x = x1; _x < x1 + sizex; _x++)
		{
			buffer[((_y - y1) * (sizex)) + (_x - x1)] = vScreen.data2[(_y * CONFIG_WIDTH) + _x];
		}
	}
}

void diffupdate_coppyDiffAreas(uint16_t areapos) {
	uint16_t x1,y1,x2,y2;
	diffupdate_convertAreaPosToCoordinateArea(areapos, &x1, &y1, &x2, &y2);
	uint8_t sizex = x2-x1+1;
	for(int y = y1; y <= y2; y++) {
		#if 0
		for(int x = x1; x <= x2; x++) {
			uint32_t cell = (y * CONFIG_WIDTH) + x;
			vScreen.data2[cell] = vScreen.data1[cell];
			//Could be faster if use memcopy16 for every line
		}
		#else
		int startpos = (y*CONFIG_WIDTH)+(x1);
		int datalen = sizex*sizeof(uint16_t);
		memcpy(&vScreen.data2[startpos], &vScreen.data1[startpos], datalen);
		#endif
	}
}

uint16_t diffupdate_markArea(uint16_t x, uint16_t y)
{
	uint16_t areapos = diffupdate_convertCoordinateToArea(x, y);
	vScreen.updatemap[areapos / 8] |= 0x01 << (areapos % 8);
	return areapos;
}

void diffupdate_createDiffMap() {
	for(uint16_t y = 0; y < CONFIG_HEIGHT; y++) {
		for(uint16_t x = 0; x < CONFIG_WIDTH; x++) {
			uint32_t cell = (y * CONFIG_WIDTH) + x;
			if(vScreen.data2[cell] != vScreen.data1[cell]) {
				//Mark Area if something changed in this area
				uint16_t areapos = diffupdate_markArea(x,y);
				diffupdate_coppyDiffAreas(areapos);
				areapos++;
				if(areapos < DIFFUPDATE_AREA_MAX) {
					//Calculate x/y startpos of next area.
					diffupdate_convertAreaPosToCoordinateArea(areapos, &x, &y, NULL, NULL);
				} else {
					return;
				}
			}
		}
	}
}

void diffupdate_updateDiffLCD() {
	for (int i = 0; i < DIFFUPDATE_AREA_MAX / 8; i++)
	{
		if (vScreen.updatemap[i] > 0)
		{
			for (int j = 0; j < 8; j++)
			{
				if ((vScreen.updatemap[i] >> j) & 0x01)
				{
					vScreen.updatemap[i] &= ~(0x01 << j);
					uint16_t areapos = (i * 8) + j;
					uint16_t x1, y1, x2, y2;
					diffupdate_convertAreaPosToCoordinateArea(areapos, &x1, &y1, &x2, &y2);
					uint16_t sizex = x2 - x1 + 1;
					uint16_t sizey = y2 - y1 + 1;
					uint16_t colors[(sizex) * (sizey)];
					// Copy Area from VSCREEN to buffer
					copyVScreenArea(x1, y1, sizex, sizey, &colors[0]);
					// xSemaphoreGive(vScreenLock);
					#ifdef CONFIG_LCD_ST7789
					st7789_DrawArea(x1, y1, sizex, sizey, &colors[0]);
					#endif
					#ifdef CONFIG_LCD_ILI9488
					ili9488_DrawArea(x1, y1, sizex, sizey, &colors[0]);
					#endif
				}
			}
		}
	}
}
#endif

void lcdSetupVScreen(rotation_t rotation)
{
	vScreen.data1 = (uint16_t *)malloc(CONFIG_WIDTH * CONFIG_HEIGHT * sizeof(uint16_t));
	memset(vScreen.data1, 0x00, CONFIG_WIDTH*CONFIG_HEIGHT*sizeof(uint16_t));
	#ifdef CONFIG_USE_DIFFUPDATE
		vScreen.diffupdate_lock = xSemaphoreCreateMutex();
		xSemaphoreTake(vScreen.diffupdate_lock, portMAX_DELAY);
		vScreen.data2 = (uint16_t *)malloc(CONFIG_WIDTH * CONFIG_HEIGHT * sizeof(uint16_t));
		memset(vScreen.data2, 0xFF, CONFIG_WIDTH*CONFIG_HEIGHT*sizeof(uint16_t));
		xSemaphoreGive(vScreen.diffupdate_lock);
	#endif
	vScreen.enabled = true;
	vScreen.rotation = rotation;
}
void lcdClearVScreen()
{
	uint32_t sizeofovscreen = sizeof(vScreen.data1);
	memset(vScreen.data1, 0x00, sizeofovscreen);
	#ifdef CONFIG_USE_DIFFUPDATE
	memset(vScreen.data2, 0xFF, sizeofovscreen);
	#endif
}
uint16_t lcdGetWidth()
{
	switch (vScreen.rotation)
	{
	case rot_0:
	case rot_180:
		return lcddevice->_width;
		break;
	case rot_90:
	case rot_270:
		return lcddevice->_height;
		break;
	default:
		return lcddevice->_width;
	}
}
uint16_t lcdGetHeight()
{
	switch (vScreen.rotation)
	{
	case rot_0:
	case rot_180:
		return lcddevice->_height;
		break;
	case rot_90:
	case rot_270:
		return lcddevice->_width;
		break;
	default:
		return lcddevice->_height;
	}
}
void lcdUpdateVScreen()
{
	if (vScreen.enabled == false)
		return;
#ifndef CONFIG_USE_DIFFUPDATE
#ifdef CONFIG_LCD_ST7789
	for (int y = 0; y < lcddevice->_height; y++)
	{
		uint16_t *colors = &vScreen.data1[y * lcddevice->_width];
		st7789_DrawMultiPixels(0, y, lcddevice->_width, colors);
	}
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_DrawMultiLines(0, 480, &vScreen.data1[0]);
#endif
#else
	// Create Diffmap and update Selected Areas.	
	xSemaphoreTake(vScreen.diffupdate_lock, portMAX_DELAY);
	diffupdate_createDiffMap();
	xSemaphoreGive(vScreen.diffupdate_lock);
	diffupdate_updateDiffLCD();
#endif
}

// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if (x >= lcdGetWidth())
		return;
	if (y >= lcdGetHeight())
		return;

	if (vScreen.enabled == false)
	{
#ifdef CONFIG_LCD_ST7789
		st7789_DrawPixel(x, y, color);
#endif
#ifdef CONFIG_LCD_ILI9488
		ili9488_DrawPixel(x, y, color);
#endif
	}
	else
	{
		// uint16_t _x = x;
		// uint16_t _y = y;
		uint16_t _x = x;
		uint16_t _y = y;
		switch (vScreen.rotation)
		{
		case rot_0:
			break;
		case rot_90:
			_x = CONFIG_WIDTH - y - 1;
			_y = x;
			break;
		case rot_180:
			_x = CONFIG_WIDTH - x - 1;
			_y = CONFIG_HEIGHT - y - 1;
			break;
		case rot_270:
			_x = y;
			_y = CONFIG_HEIGHT - x - 1;
			break;
		}
		if (_x >= lcddevice->_width)
			return;
		if (_y >= lcddevice->_height)
			return;
		vScreen.data1[(_y * lcddevice->_width) + _x] = color;
	}
}

// Draw multi pixel
// x:X coordinate
// y:Y coordinate
// size:Number of colors
// colors:colors
void lcdDrawMultiPixels(uint16_t x, uint16_t y, uint16_t size, uint16_t *colors)
{
	if (x + size > lcdGetWidth())
		return;
	if (y >= lcdGetHeight())
		return;

	if (vScreen.enabled == false)
	{
#ifdef CONFIG_LCD_ST7789
		st7789_DrawMultiPixels(x, y, size, colors);
#endif
#ifdef CONFIG_LCD_ILI9488
		ili9488_DrawMultiPixels(x, y, size, colors);
#endif
	}
	else
	{
		int16_t _x1 = x;
		int16_t _y1 = y;
		switch (vScreen.rotation)
		{
		case rot_0:
			if (_x1 + size > lcddevice->_width)
				return;
			if (_y1 >= lcddevice->_height)
				return;
			for (int i = 0; i < size; i++)
			{
				vScreen.data1[(_y1 * lcddevice->_width) + _x1 + i] = colors[i];
			}
			break;
		case rot_90:
			_x1 = CONFIG_WIDTH - y - 1;
			_y1 = x;
			if (_x1 < 0)
				return;
			if (_y1 + size > lcddevice->_height)
				return;
			for (int i = 0; i < size; i++)
			{
				vScreen.data1[((_y1 + i) * lcddevice->_width) + _x1] = colors[i];
			}
			break;
		case rot_180:
			_x1 = CONFIG_WIDTH - x - 1;
			_y1 = CONFIG_HEIGHT - y - 1;
			if (_x1 - (size - 1) < 0)
				return;
			if (_y1 < 0)
				return;
			for (int i = 0; i < size; i++)
			{
				vScreen.data1[(_y1 * lcddevice->_width) + _x1 - i] = colors[i];
			}
			break;
		case rot_270:
			_x1 = y;
			_y1 = CONFIG_HEIGHT - x - 1;
			if (_x1 >= lcddevice->_width)
				return;
			if (_y1 - (size - 1) < 0)
				return;
			for (int i = 0; i < size; i++)
			{
				vScreen.data1[((_y1 - i) * lcddevice->_width) + _x1] = colors[i];
			}
			break;
		}
	}
}

// Draw rectangle of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	if (vScreen.enabled == false)
	{
#ifdef CONFIG_LCD_ST7789
		st7789_DrawFillRect(x1, y1, x2, y2, color);
#endif
#ifdef CONFIG_LCD_ILI9488
		ili9488_DrawFillRect(x1, y1, x2, y2, color);
#endif
	}
	else
	{
		uint16_t _x1 = x1;
		uint16_t _x2 = x2;
		uint16_t _y1 = y1;
		uint16_t _y2 = y2;
		for (int i = _y1; i < _y2; i++)
		{
			for (int j = _x1; j < _x2; j++)
			{
				lcdDrawPixel(j, i, color);
			}
		}
	}
}

// Display OFF
void lcdDisplayOff()
{
#ifdef CONFIG_LCD_ST7789
	st7789_DisplayOff();
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_DisplayOff();
#endif
}

// Display ON
void lcdDisplayOn()
{
#ifdef CONFIG_LCD_ST7789
	st7789_DisplayOn();
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_DisplayOn();
#endif
}

// Fill screen
// color:color
void lcdFillScreen(uint16_t color)
{
	// memset16(vScreen.data, color, CONFIG_WIDTH * CONFIG_HEIGHT);
// #ifdef CONFIG_USE_DIFFUPDATE
// 	xSemaphoreTake(fastLCDMapLock, portMAX_DELAY);
// 	memset(fastLCD_updatemap, 0xFF, sizeof(fastLCD_updatemap));
// 	xSemaphoreGive(fastLCDMapLock);
// #endif
	// lcdDrawFillRect(0, 0, lcdGetWidth()-1, lcdGetHeight()-1, color);
#ifdef CONFIG_USE_DIFFUPDATE
	memset16(vScreen.data1, color, CONFIG_WIDTH * CONFIG_HEIGHT);
#else
	lcdDrawFillRect(0, 0, lcdGetWidth()-1, lcdGetHeight()-1, color);
#endif
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	int i;
	int dx, dy;
	int sx, sy;
	int E;

	/* distance between two points */
	dx = (x2 > x1) ? x2 - x1 : x1 - x2;
	dy = (y2 > y1) ? y2 - y1 : y1 - y2;

	/* direction of two point */
	sx = (x2 > x1) ? 1 : -1;
	sy = (y2 > y1) ? 1 : -1;

	/* inclination < 1 */
	if (dx > dy)
	{
		E = -dx;
		for (i = 0; i <= dx; i++)
		{
			lcdDrawPixel(x1, y1, color);
			x1 += sx;
			E += 2 * dy;
			if (E >= 0)
			{
				y1 += sy;
				E -= 2 * dx;
			}
		}

		/* inclination >= 1 */
	}
	else
	{
		E = -dy;
		for (i = 0; i <= dy; i++)
		{
			lcdDrawPixel(x1, y1, color);
			y1 += sy;
			E += 2 * dx;
			if (E >= 0)
			{
				x1 += sx;
				E -= 2 * dy;
			}
		}
	}
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// color:color
void lcdDrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	lcdDrawLine(x1, y1, x2, y1, color);
	lcdDrawLine(x2, y1, x2, y2, color);
	lcdDrawLine(x2, y2, x1, y2, color);
	lcdDrawLine(x1, y2, x1, y1, color);
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle :Angle of rectangle
// color :color

// When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
//  x1 = x * cos(angle) - y * sin(angle)
//  y1 = x * sin(angle) + y * cos(angle)
void lcdDrawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color)
{
	double xd, yd, rd;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	int x4, y4;
	rd = -angle * M_PI / 180.0;
	xd = 0.0 - w / 2;
	yd = h / 2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w / 2;
	yd = h / 2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	yd = 0.0 - yd;
	x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcdDrawLine(x1, y1, x2, y2, color);
	lcdDrawLine(x1, y1, x3, y3, color);
	lcdDrawLine(x2, y2, x4, y4, color);
	lcdDrawLine(x3, y3, x4, y4, color);
}

// Draw triangle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of triangle
// h:Height of triangle
// angle :Angle of triangle
// color :color

// When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
//  x1 = x * cos(angle) - y * sin(angle)
//  y1 = x * sin(angle) + y * cos(angle)
void lcdDrawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color)
{
	double xd, yd, rd;
	int x1, y1;
	int x2, y2;
	int x3, y3;
	rd = -angle * M_PI / 180.0;
	xd = 0.0;
	yd = h / 2;
	x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = w / 2;
	yd = 0.0 - yd;
	x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	xd = 0.0 - w / 2;
	x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
	y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

	lcdDrawLine(x1, y1, x2, y2, color);
	lcdDrawLine(x1, y1, x3, y3, color);
	lcdDrawLine(x2, y2, x3, y3, color);
}

// Draw circle
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;

	x = 0;
	y = -r;
	err = 2 - 2 * r;
	do
	{
		lcdDrawPixel(x0 - x, y0 + y, color);
		lcdDrawPixel(x0 - y, y0 - x, color);
		lcdDrawPixel(x0 + x, y0 - y, color);
		lcdDrawPixel(x0 + y, y0 + x, color);
		if ((old_err = err) <= x)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y < 0);
}

// Draw circle of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;
	int ChangeX;

	x = 0;
	y = -r;
	err = 2 - 2 * r;
	ChangeX = 1;
	do
	{
		if (ChangeX)
		{
			lcdDrawLine(x0 - x, y0 - y, x0 - x, y0 + y, color);
			lcdDrawLine(x0 + x, y0 - y, x0 + x, y0 + y, color);
		} // endif
		ChangeX = (old_err = err) <= x;
		if (ChangeX)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y <= 0);
}

// Draw rectangle with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// r:radius
// color:color
void lcdDrawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color)
{
	int x;
	int y;
	int err;
	int old_err;
	unsigned char temp;

	if (x1 > x2)
	{
		temp = x1;
		x1 = x2;
		x2 = temp;
	} // endif

	if (y1 > y2)
	{
		temp = y1;
		y1 = y2;
		y2 = temp;
	} // endif

	// ESP_LOGD(TAG, "x1=%d x2=%d delta=%d r=%d",x1, x2, x2-x1, r);
	// ESP_LOGD(TAG, "y1=%d y2=%d delta=%d r=%d",y1, y2, y2-y1, r);
	if (x2 - x1 < r)
		return; // Add 20190517
	if (y2 - y1 < r)
		return; // Add 20190517

	x = 0;
	y = -r;
	err = 2 - 2 * r;

	do
	{
		if (x)
		{
			lcdDrawPixel(x1 + r - x, y1 + r + y, color);
			lcdDrawPixel(x2 - r + x, y1 + r + y, color);
			lcdDrawPixel(x1 + r - x, y2 - r - y, color);
			lcdDrawPixel(x2 - r + x, y2 - r - y, color);
		} // endif
		if ((old_err = err) <= x)
			err += ++x * 2 + 1;
		if (old_err > y || err > x)
			err += ++y * 2 + 1;
	} while (y < 0);

	// ESP_LOGD(TAG, "x1+r=%d x2-r=%d",x1+r, x2-r);
	lcdDrawLine(x1 + r, y1, x2 - r, y1, color);
	lcdDrawLine(x1 + r, y2, x2 - r, y2, color);
	// ESP_LOGD(TAG, "y1+r=%d y2-r=%d",y1+r, y2-r);
	lcdDrawLine(x1, y1 + r, x1, y2 - r, color);
	lcdDrawLine(x2, y1 + r, x2, y2 - r, color);
}

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void lcdDrawArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color)
{
	double Vx = x1 - x0;
	double Vy = y1 - y0;
	double v = sqrt(Vx * Vx + Vy * Vy);
	//	 printf("v=%f\n",v);
	double Ux = Vx / v;
	double Uy = Vy / v;

	uint16_t L[2], R[2];
	L[0] = x1 - Uy * w - Ux * v;
	L[1] = y1 + Ux * w - Uy * v;
	R[0] = x1 + Uy * w - Ux * v;
	R[1] = y1 - Ux * w - Uy * v;
	// printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	// lcdDrawLine(x0,y0,x1,y1,color);
	lcdDrawLine(x1, y1, L[0], L[1], color);
	lcdDrawLine(x1, y1, R[0], R[1], color);
	lcdDrawLine(L[0], L[1], R[0], R[1], color);
}

// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End	X coordinate
// y2:End	Y coordinate
// w:Width of the botom
// color:color
void lcdDrawFillArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color)
{
	double Vx = x1 - x0;
	double Vy = y1 - y0;
	double v = sqrt(Vx * Vx + Vy * Vy);
	// printf("v=%f\n",v);
	double Ux = Vx / v;
	double Uy = Vy / v;

	uint16_t L[2], R[2];
	L[0] = x1 - Uy * w - Ux * v;
	L[1] = y1 + Ux * w - Uy * v;
	R[0] = x1 + Uy * w - Ux * v;
	R[1] = y1 - Ux * w - Uy * v;
	// printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

	lcdDrawLine(x0, y0, x1, y1, color);
	lcdDrawLine(x1, y1, L[0], L[1], color);
	lcdDrawLine(x1, y1, R[0], R[1], color);
	lcdDrawLine(L[0], L[1], R[0], R[1], color);

	int ww;
	for (ww = w - 1; ww > 0; ww--)
	{
		L[0] = x1 - Uy * ww - Ux * v;
		L[1] = y1 + Ux * ww - Uy * v;
		R[0] = x1 + Uy * ww - Ux * v;
		R[1] = y1 - Ux * ww - Uy * v;
		// printf("Fill>L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);
		lcdDrawLine(x1, y1, L[0], L[1], color);
		lcdDrawLine(x1, y1, R[0], R[1], color);
	}
}

// RGB565 conversion
// RGB565 is R(5)+G(6)+B(5)=16bit color format.
// Bit image "RRRRRGGGGGGBBBBB"
uint16_t rgb565_conv(uint16_t r, uint16_t g, uint16_t b)
{
	return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Draw ASCII character
// x:X coordinate
// y:Y coordinate
// ascii: ascii code
// color:color
int lcdDrawChar(FontxFile *fxs, uint16_t x, uint16_t y, uint8_t ascii, uint16_t color)
{
	uint16_t xx, yy, bit, ofs;
	unsigned char fonts[128]; // font pattern
	unsigned char pw, ph;
	int h, w;
	uint16_t mask;
	bool rc;

	rc = GetFontx(fxs, ascii, fonts, &pw, &ph);
	if (!rc)
		return 0;

	int16_t xd1 = 0;
	int16_t yd1 = 0;
	int16_t xd2 = 0;
	int16_t yd2 = 0;
	uint16_t xss = 0;
	uint16_t yss = 0;
	int16_t xsd = 0;
	int16_t ysd = 0;
	int16_t next = 0;
	uint16_t x0 = 0;
	uint16_t x1 = 0;
	uint16_t y0 = 0;
	uint16_t y1 = 0;
	if (lcddevice->_font_direction == 0)
	{
		xd1 = +1;
		yd1 = +1; //-1;
		xd2 = 0;
		yd2 = 0;
		xss = x;
		yss = y - (ph - 1);
		xsd = 1;
		ysd = 0;
		next = x + pw;

		x0 = x;
		y0 = y - (ph - 1);
		x1 = x + (pw - 1);
		y1 = y;
	}
	else if (lcddevice->_font_direction == 2)
	{
		xd1 = -1;
		yd1 = -1; //+1;
		xd2 = 0;
		yd2 = 0;
		xss = x;
		yss = y + ph + 1;
		xsd = 1;
		ysd = 0;
		next = x - pw;

		x0 = x - (pw - 1);
		y0 = y;
		x1 = x;
		y1 = y + (ph - 1);
	}
	else if (lcddevice->_font_direction == 1)
	{
		xd1 = 0;
		yd1 = 0;
		xd2 = -1;
		yd2 = +1; //-1;
		xss = x + ph;
		yss = y;
		xsd = 0;
		ysd = 1;
		next = y + pw; // y - pw;

		x0 = x;
		y0 = y;
		x1 = x + (ph - 1);
		y1 = y + (pw - 1);
	}
	else if (lcddevice->_font_direction == 3)
	{
		xd1 = 0;
		yd1 = 0;
		xd2 = +1;
		yd2 = -1; //+1;
		xss = x - (ph - 1);
		yss = y;
		xsd = 0;
		ysd = 1;
		next = y - pw; // y + pw;

		x0 = x - (ph - 1);
		y0 = y - (pw - 1);
		x1 = x;
		y1 = y;
	}

	if (lcddevice->_font_fill)
		lcdDrawFillRect(x0, y0, x1, y1, lcddevice->_font_fill_color);

	int bits;
	ofs = 0;
	yy = yss;
	xx = xss;
	for (h = 0; h < ph; h++)
	{
		if (xsd)
			xx = xss;
		if (ysd)
			yy = yss;
		// for(w=0;w<(pw/8);w++) {
		bits = pw;
		for (w = 0; w < ((pw + 4) / 8); w++)
		{
			mask = 0x80;
			for (bit = 0; bit < 8; bit++)
			{
				bits--;
				if (bits < 0)
					continue;
				// if(_DEBUG_)printf("xx=%d yy=%d mask=%02x fonts[%d]=%02x\n",xx,yy,mask,ofs,fonts[ofs]);
				if (fonts[ofs] & mask)
				{
					lcdDrawPixel(xx, yy, color);
				}
				else
				{
					// if (lcddevice->_font_fill) lcdDrawPixel(xx, yy, lcddevice->_font_fill_color);
				}
				if (h == (ph - 2) && lcddevice->_font_underline)
					lcdDrawPixel(xx, yy, lcddevice->_font_underline_color);
				if (h == (ph - 1) && lcddevice->_font_underline)
					lcdDrawPixel(xx, yy, lcddevice->_font_underline_color);
				xx = xx + xd1;
				yy = yy + yd2;
				mask = mask >> 1;
			}
			ofs++;
		}
		yy = yy + yd1;
		xx = xx + xd2;
	}

	if (next < 0)
		next = 0;
	return next;
}

int lcdDrawString(FontxFile *fx, uint16_t x, uint16_t y, char* ascii, uint16_t color)
{
	int length = strlen((char *)ascii);
	for (int i = 0; i < length; i++)
	{
		if (lcddevice->_font_direction == 0)
			x = lcdDrawChar(fx, x, y, ascii[i], color);
		if (lcddevice->_font_direction == 1)
			y = lcdDrawChar(fx, x, y, ascii[i], color);
		if (lcddevice->_font_direction == 2)
			x = lcdDrawChar(fx, x, y, ascii[i], color);
		if (lcddevice->_font_direction == 3)
			y = lcdDrawChar(fx, x, y, ascii[i], color);
	}
	if (lcddevice->_font_direction == 0)
		return x;
	if (lcddevice->_font_direction == 2)
		return x;
	if (lcddevice->_font_direction == 1)
		return y;
	if (lcddevice->_font_direction == 3)
		return y;
	return 0;
}

// Draw Non-Alphanumeric character
// x:X coordinate
// y:Y coordinate
// code: charcter code
// color:color
int lcdDrawCode(FontxFile *fx, uint16_t x, uint16_t y, uint8_t code, uint16_t color)
{
	if (lcddevice->_font_direction == 0)
		x = lcdDrawChar(fx, x, y, code, color);
	if (lcddevice->_font_direction == 1)
		y = lcdDrawChar(fx, x, y, code, color);
	if (lcddevice->_font_direction == 2)
		x = lcdDrawChar(fx, x, y, code, color);
	if (lcddevice->_font_direction == 3)
		y = lcdDrawChar(fx, x, y, code, color);
	if (lcddevice->_font_direction == 0)
		return x;
	if (lcddevice->_font_direction == 2)
		return x;
	if (lcddevice->_font_direction == 1)
		return y;
	if (lcddevice->_font_direction == 3)
		return y;
	return 0;
}

// Set font direction
// dir:Direction
void lcdSetFontDirection(uint16_t dir)
{
	lcddevice->_font_direction = dir;
}

// Set font filling
// color:fill color
void lcdSetFontFill(uint16_t color)
{
	lcddevice->_font_fill = true;
	lcddevice->_font_fill_color = color;
}

// UnSet font filling
void lcdUnsetFontFill()
{
	lcddevice->_font_fill = false;
}

// Set font underline
// color:frame color
void lcdSetFontUnderLine(uint16_t color)
{
	lcddevice->_font_underline = true;
	lcddevice->_font_underline_color = color;
}

// UnSet font underline
void lcdUnsetFontUnderLine()
{
	lcddevice->_font_underline = false;
}

// Backlight OFF
void lcdBacklightOff()
{
	if (lcddevice->_bl >= 0)
	{
		gpio_set_level(lcddevice->_bl, 0);
	}
}

// Backlight ON
void lcdBacklightOn()
{
	if (lcddevice->_bl >= 0)
	{
		gpio_set_level(lcddevice->_bl, 1);
	}
}

// Display Inversion Off
void lcdInversionOff()
{
#ifdef CONFIG_LCD_ST7789
	st7789_InversionOff();
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_InversionOff();
#endif
}

// Display Inversion On
void lcdInversionOn()
{
#ifdef CONFIG_LCD_ST7789
	st7789_InversionOn();
#endif
#ifdef CONFIG_LCD_ILI9488
	ili9488_InversionOn();
#endif
}

void lcdDrawDataUInt8(uint16_t x, uint16_t y, uint16_t width, uint8_t height, uint8_t min, uint8_t max, bool leftToRight, uint8_t *data, uint16_t color)
{
	uint8_t diffy = max - min;
	float cropfactor_y = (float)(height) / (float)(diffy);
	for (int i = 0; i < width; i++)
	{		
		uint8_t data_y = (float)((data[i] - min)) * cropfactor_y;
		if(leftToRight) {
			lcdDrawPixel(x + i, (y+height) - data_y, color);
		} else {
			lcdDrawPixel((x+width) - i - 1, (y+height) - data_y, color);
		}
	}
}
void lcdDrawDataInt8(uint16_t x, uint16_t y, uint8_t width, uint8_t height, int8_t min, int8_t max, int8_t *data, uint16_t color)
{
}

#endif