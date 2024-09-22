#include "../../eduboard2.h"
#include "../eduboard2_lcd.h"
#include "lcdDriver.h"

#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"

#define TAG "LCD_Driver"

#ifdef CONFIG_ENABLE_LCD

#ifdef CONFIG_LCD_TEST
TickType_t FillTest(int width, int height) {
	ESP_LOGI(TAG, "Start FillTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdFillScreen(RED);
	lcdUpdateVScreen();
	vTaskDelay(50);
	lcdFillScreen(GREEN);
	lcdUpdateVScreen();
	vTaskDelay(50);
	lcdFillScreen(BLUE);
	lcdUpdateVScreen();
	vTaskDelay(50);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ColorBarTest(int width, int height) {
	ESP_LOGI(TAG, "Start ColorBarTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	if (width < height) {
		uint16_t y1,y2;
		y1 = height/3;
		y2 = (height/3)*2;
		lcdDrawFillRect(0, 0, width-1, y1-1, RED);
		vTaskDelay(1);
		lcdDrawFillRect(0, y1, width-1, y2-1, GREEN);
		vTaskDelay(1);
		lcdDrawFillRect(0, y2, width-1, height-1, BLUE);
		lcdUpdateVScreen();
	} else {
		uint16_t x1,x2;
		x1 = width/3;
		x2 = (width/3)*2;
		lcdDrawFillRect(0, 0, x1-1, height-1, RED);
		vTaskDelay(1);
		lcdDrawFillRect(x1, 0, x2-1, height-1, GREEN);
		vTaskDelay(1);
		lcdDrawFillRect(x2, 0, width-1, height-1, BLUE);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ArrowTest(FontxFile *fx, int width, int height) {
	ESP_LOGI(TAG, "Start ArrowTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	////ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	int	stlen;
	uint8_t ascii[24];
	uint16_t color;

	lcdFillScreen(BLACK);

	strcpy((char *)ascii, "ST7789");
	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 1;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 1;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(fx, xpos, ypos, ascii, color);

	lcdSetFontDirection(0);
	color = RED;
	lcdDrawFillArrow(10, 10, 0, 0, 5, color);
	strcpy((char *)ascii, "0,0");
	lcdDrawString(fx, 0, 30, ascii, color);

	color = GREEN;
	lcdDrawFillArrow(width-11, 10, width-1, 0, 5, color);
	//strcpy((char *)ascii, "79,0");
	sprintf((char *)ascii, "%d,0",width-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(fx, xpos, 30, ascii, color);

	color = GRAY;
	lcdDrawFillArrow(10, height-11, 0, height-1, 5, color);
	//strcpy((char *)ascii, "0,159");
	sprintf((char *)ascii, "0,%d",height-1);
	ypos = (height-11) - (fontHeight) + 5;
	lcdDrawString(fx, 0, ypos, ascii, color);

	color = CYAN;
	lcdDrawFillArrow(width-11, height-11, width-1, height-1, 5, color);
	//strcpy((char *)ascii, "79,159");
	sprintf((char *)ascii, "%d,%d",width-1, height-1);
	stlen = strlen((char *)ascii);
	xpos = (width-1) - (fontWidth*stlen);
	lcdDrawString(fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t DirectionTest(FontxFile *fx, int width, int height) {
	ESP_LOGI(TAG, "Start DirectionTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	////ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

	uint16_t color;
	lcdFillScreen(BLACK);
	uint8_t ascii[20];

	color = RED;
	strcpy((char *)ascii, "Direction=0");
	lcdSetFontDirection(0);
	lcdDrawString(fx, 0, fontHeight-1, ascii, color);

	color = BLUE;
	strcpy((char *)ascii, "Direction=2");
	lcdSetFontDirection(2);
	lcdDrawString(fx, (width-1), (height-1)-(fontHeight*1), ascii, color);

	color = CYAN;
	strcpy((char *)ascii, "Direction=1");
	lcdSetFontDirection(1);
	lcdDrawString(fx, (width-1)-fontHeight, 0, ascii, color);

	color = GREEN;
	strcpy((char *)ascii, "Direction=3");
	lcdSetFontDirection(3);
	lcdDrawString(fx, (fontHeight-1), height-1, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t HorizontalTest(FontxFile *fx, int width, int height) {
	ESP_LOGI(TAG, "Start HorizontalTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	////ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

	uint16_t color;
	lcdFillScreen(BLACK);
	uint8_t ascii[20];

	color = RED;
	strcpy((char *)ascii, "Direction=0");
	lcdSetFontDirection(0);
	lcdDrawString(fx, 0, fontHeight*1-1, ascii, color);
	lcdSetFontUnderLine(RED);
	lcdDrawString(fx, 0, fontHeight*2-1, ascii, color);
	lcdUnsetFontUnderLine();

	lcdSetFontFill(GREEN);
	lcdDrawString(fx, 0, fontHeight*3-1, ascii, color);
	lcdSetFontUnderLine(RED);
	lcdDrawString(fx, 0, fontHeight*4-1, ascii, color);
	lcdUnsetFontFill();
	lcdUnsetFontUnderLine();

	color = BLUE;
	strcpy((char *)ascii, "Direction=2");
	lcdSetFontDirection(2);
	lcdDrawString(fx, width, height-(fontHeight*1)-1, ascii, color);
	lcdSetFontUnderLine(BLUE);
	lcdDrawString(fx, width, height-(fontHeight*2)-1, ascii, color);
	lcdUnsetFontUnderLine();

	lcdSetFontFill(YELLOW);
	lcdDrawString(fx, width, height-(fontHeight*3)-1, ascii, color);
	lcdSetFontUnderLine(BLUE);
	lcdDrawString(fx, width, height-(fontHeight*4)-1, ascii, color);
	lcdUnsetFontFill();
	lcdUnsetFontUnderLine();

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t VerticalTest(FontxFile *fx, int width, int height) {
	ESP_LOGI(TAG, "Start VerticalTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	////ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

	uint16_t color;
	lcdFillScreen(BLACK);
	uint8_t ascii[20];

	color = RED;
	strcpy((char *)ascii, "Direction=1");
	lcdSetFontDirection(1);
	lcdDrawString(fx, width-(fontHeight*1), 0, ascii, color);
	lcdSetFontUnderLine(RED);
	lcdDrawString(fx, width-(fontHeight*2), 0, ascii, color);
	lcdUnsetFontUnderLine();

	lcdSetFontFill(GREEN);
	lcdDrawString(fx, width-(fontHeight*3), 0, ascii, color);
	lcdSetFontUnderLine(RED);
	lcdDrawString(fx, width-(fontHeight*4), 0, ascii, color);
	lcdUnsetFontFill();
	lcdUnsetFontUnderLine();

	color = BLUE;
	strcpy((char *)ascii, "Direction=3");
	lcdSetFontDirection(3);
	lcdDrawString(fx, (fontHeight*1)-1, height, ascii, color);
	lcdSetFontUnderLine(BLUE);
	lcdDrawString(fx, (fontHeight*2)-1, height, ascii, color);
	lcdUnsetFontUnderLine();

	lcdSetFontFill(YELLOW);
	lcdDrawString(fx, (fontHeight*3)-1, height, ascii, color);
	lcdSetFontUnderLine(BLUE);
	lcdDrawString(fx, (fontHeight*4)-1, height, ascii, color);
	lcdUnsetFontFill();
	lcdUnsetFontUnderLine();

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}


TickType_t LineTest(int width, int height) {
	ESP_LOGI(TAG, "Start LineTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	//lcdFillScreen(WHITE);
	lcdFillScreen(BLACK);
	color=RED;
	for(int ypos=0;ypos<height;ypos=ypos+10) {
		lcdDrawLine(0, ypos, width, ypos, color);
	}
	lcdUpdateVScreen();

	for(int xpos=0;xpos<width;xpos=xpos+10) {
		lcdDrawLine(xpos, 0, xpos, height, color);
	}
	lcdUpdateVScreen();

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t CircleTest(int width, int height) {
	ESP_LOGI(TAG, "Start CircleTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	//lcdFillScreen(WHITE);
	lcdFillScreen(BLACK);
	color = CYAN;
	uint16_t xpos = width/2;
	uint16_t ypos = height/2;
	for(int i=5;i<height;i=i+5) {
		lcdDrawCircle(xpos, ypos, i, color);
		vTaskDelay(1);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t RectAngleTest(int width, int height) {
	ESP_LOGI(TAG, "Start RectAngleTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(BLACK);
	color = CYAN;
	uint16_t xpos = width/2;
	uint16_t ypos = height/2;

	uint16_t w = width * 0.4;
	uint16_t h = w * 0.4;
	int angle;
	for(angle=0;angle<=(360*3);angle=angle+30) {
		lcdDrawRectAngle(xpos, ypos, w, h, angle, color);
		lcdUpdateVScreen();
		usleep(10000);
		lcdDrawRectAngle(xpos, ypos, w, h, angle, BLACK);
	}

	for(angle=0;angle<=180;angle=angle+30) {
		lcdDrawRectAngle(xpos, ypos, w, h, angle, color);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t TriangleTest(int width, int height) {
	ESP_LOGI(TAG, "Start TriangleTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	//lcdFillScreen(WHITE);
	lcdFillScreen(BLACK);
	color = CYAN;
	uint16_t xpos = width/2;
	uint16_t ypos = height/2;

	uint16_t w = width * 0.4;
	uint16_t h = w * 1.0;
	int angle;

	for(angle=0;angle<=(360*3);angle=angle+30) {
		lcdDrawTriangle(xpos, ypos, w, h, angle, color);
		lcdUpdateVScreen();
		usleep(10000);
		lcdDrawTriangle(xpos, ypos, w, h, angle, BLACK);
	}

	for(angle=0;angle<=360;angle=angle+30) {
		lcdDrawTriangle(xpos, ypos, w, h, angle, color);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t RoundRectTest(int width, int height) {
	ESP_LOGI(TAG, "Start RoundRectTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();


	uint16_t color;
	uint16_t limit = width;
	if (width > height) limit = height;
	//lcdFillScreen(WHITE);
	lcdFillScreen(BLACK);
	color = BLUE;
	for(int i=5;i<limit;i=i+5) {
		if (i > (limit-i-1) ) break;
		////ESP_LOGI(__FUNCTION__, "i=%d, width-i-1=%d",i, width-i-1);
		lcdDrawRoundRect(i, i, (width-i-1), (height-i-1), 10, color);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t FillRectTest(int width, int height) {
	ESP_LOGI(TAG, "Start FillRectTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(CYAN);

	uint16_t red;
	uint16_t green;
	uint16_t blue;
	srand( (unsigned int)time( NULL ) );
	for(int i=1;i<100;i++) {
		red=rand()%255;
		green=rand()%255;
		blue=rand()%255;
		color=rgb565_conv(red, green, blue);
		uint16_t xpos=rand()%width;
		uint16_t ypos=rand()%height;
		uint16_t size=rand()%(width/5);
		lcdDrawFillRect(xpos, ypos, xpos+size, ypos+size, color);
		lcdUpdateVScreen();
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t ColorTest(int width, int height) {
	ESP_LOGI(TAG, "Start ColorTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint16_t color;
	lcdFillScreen(WHITE);
	color = RED;
	uint16_t delta = height/16;
	uint16_t ypos = 0;
	for(int i=0;i<16;i++) {
		////ESP_LOGI(__FUNCTION__, "color=0x%x",color);
		lcdDrawFillRect(0, ypos, width-1, ypos+delta, color);
		lcdUpdateVScreen();
		color = color >> 1;
		ypos = ypos + delta;
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}


TickType_t BMPTest(char * file, int width, int height) {
	ESP_LOGI(TAG, "Start BMPTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdSetFontDirection(0);
	lcdFillScreen(BLACK);

	// open requested file
	esp_err_t ret;
	FILE* fp = fopen(file, "rb");
	if (fp == NULL) {
		ESP_LOGW(__FUNCTION__, "File not found [%s]", file);		
		return 0;
	}

	// read bmp header
	bmpfile_t *result = (bmpfile_t*)malloc(sizeof(bmpfile_t));
	ret = fread(result->header.magic, 1, 2, fp);
	assert(ret == 2);
	if (result->header.magic[0]!='B' || result->header.magic[1] != 'M') {
		ESP_LOGW(__FUNCTION__, "File is not BMP");
		free(result);
		fclose(fp);
		return 0;
	}
	ret = fread(&result->header.filesz, 4, 1 , fp);
	assert(ret == 1);
	//ESP_LOGD(__FUNCTION__,"result->header.filesz=%d", result->header.filesz);
	ret = fread(&result->header.creator1, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->header.creator2, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->header.offset, 4, 1, fp);
	assert(ret == 1);

	// read dib header
	ret = fread(&result->dib.header_sz, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.width, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.height, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.nplanes, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.depth, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.compress_type, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.bmp_bytesz, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.hres, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.vres, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.ncolors, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.nimpcolors, 4, 1, fp);
	assert(ret == 1);

	if((result->dib.depth == 24) && (result->dib.compress_type == 0)) {
		// BMP rows are padded (if needed) to 4-byte boundary
		uint32_t rowSize = (result->dib.width * 3 + 3) & ~3;
		int w = result->dib.width;
		int h = result->dib.height;
		//ESP_LOGD(__FUNCTION__,"w=%d h=%d", w, h);
		int _x;
		int _w;
		int _cols;
		int _cole;
		if (width >= w) {
			_x = (width - w) / 2;
			_w = w;
			_cols = 0;
			_cole = w - 1;
		} else {
			_x = 0;
			_w = width;
			_cols = (w - width) / 2;
			_cole = _cols + width - 1;
		}
		//ESP_LOGD(__FUNCTION__,"_x=%d _w=%d _cols=%d _cole=%d",_x, _w, _cols, _cole);

		int _y;
		int _rows;
		int _rowe;
		if (height >= h) {
			_y = (height - h) / 2;
			_rows = 0;
			_rowe = h -1;
		} else {
			_y = 0;
			_rows = (h - height) / 2;
			_rowe = _rows + height - 1;
		}
		//ESP_LOGD(__FUNCTION__,"_y=%d _rows=%d _rowe=%d", _y, _rows, _rowe);

#define BUFFPIXEL 20
		uint8_t sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
		uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * w);

		for (int row=0; row<h; row++) { // For each scanline...
			if (row < _rows || row > _rowe) continue;
			// Seek to start of scan line.	It might seem labor-
			// intensive to be doing this on every line, but this
			// method covers a lot of gritty details like cropping
			// and scanline padding.  Also, the seek only takes
			// place if the file position actually needs to change
			// (avoids a lot of cluster math in SD library).
			// Bitmap is stored bottom-to-top order (normal BMP)
			int pos = result->header.offset + (h - 1 - row) * rowSize;
			fseek(fp, pos, SEEK_SET);
			int buffidx = sizeof(sdbuffer); // Force buffer reload

			int index = 0;
			for (int col=0; col<w; col++) { // For each pixel...
				if (buffidx >= sizeof(sdbuffer)) { // Indeed
					fread(sdbuffer, sizeof(sdbuffer), 1, fp);
					buffidx = 0; // Set index to beginning
				}
				if (col < _cols || col > _cole) continue;
				// Convert pixel from BMP to TFT format, push to display
				uint8_t b = sdbuffer[buffidx++];
				uint8_t g = sdbuffer[buffidx++];
				uint8_t r = sdbuffer[buffidx++];
				colors[index++] = rgb565_conv(r, g, b);
			} // end for col
			//ESP_LOGD(__FUNCTION__,"lcdDrawMultiPixels _x=%d _y=%d row=%d",_x, _y, row);
			//lcdDrawMultiPixels(_x, row+_y, _w, colors);
			lcdDrawMultiPixels(_x, _y, _w, colors);
			_y++;
		} // end for row
		lcdUpdateVScreen();
		free(colors);
	} // end if
	free(result);
	fclose(fp);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t QRTest(char * file, int width, int height) {
	ESP_LOGI(TAG, "Start QRTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdSetFontDirection(0);
	lcdFillScreen(BLACK);

	// open requested file
	esp_err_t ret;
	FILE* fp = fopen(file, "rb");
	if (fp == NULL) {
		ESP_LOGW(__FUNCTION__, "File not found [%s]", file);
		return 0;
	}

	// read bmp header
	bmpfile_t *result = (bmpfile_t*)malloc(sizeof(bmpfile_t));
	ret = fread(result->header.magic, 1, 2, fp);
	assert(ret == 2);
	if (result->header.magic[0]!='B' || result->header.magic[1] != 'M') {
		ESP_LOGW(__FUNCTION__, "File is not BMP");
		free(result);
		fclose(fp);
		return 0;
	}
	ret = fread(&result->header.filesz, 4, 1 , fp);
	assert(ret == 1);
	//ESP_LOGD(__FUNCTION__,"result->header.filesz=%d", result->header.filesz);
	ret = fread(&result->header.creator1, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->header.creator2, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->header.offset, 4, 1, fp);
	assert(ret == 1);

	// read dib header
	ret = fread(&result->dib.header_sz, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.width, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.height, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.nplanes, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.depth, 2, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.compress_type, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.bmp_bytesz, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.hres, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.vres, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.ncolors, 4, 1, fp);
	assert(ret == 1);
	ret = fread(&result->dib.nimpcolors, 4, 1, fp);
	assert(ret == 1);

	//ESP_LOGD(__FUNCTION__, "dib.depth=%d dib.compress_type=%d", result->dib.depth, result->dib.compress_type);
	//if((result->dib.depth == 24) && (result->dib.compress_type == 0)) {
	if((result->dib.depth == 1) && (result->dib.compress_type == 0)) {
		//ESP_LOGD(__FUNCTION__, "dib.bmp_bytesz=%d", result->dib.bmp_bytesz);
		// BMP rows are padded (if needed) to 4-byte boundary
		//uint32_t rowSize = (result->dib.width * 3 + 3) & ~3;
		int w = result->dib.width;
		int h = result->dib.height;
		uint32_t rowSize = result->dib.bmp_bytesz / result->dib.height;
		//ESP_LOGD(__FUNCTION__,"dib.width=%d dib.height=%d rowSize=%d", result->dib.width, result->dib.height, rowSize);
		int _x;
		int _w;
		int _cols;
		int _cole;
		if (width >= w) {
			_x = (width - w) / 2;
			_w = w;
			_cols = 0;
			_cole = w - 1;
		} else {
			_x = 0;
			_w = width;
			_cols = (w - width) / 2;
			_cole = _cols + width - 1;
		}
		//ESP_LOGD(__FUNCTION__,"_x=%d _w=%d _cols=%d _cole=%d",_x, _w, _cols, _cole);

		int _y;
		int _rows;
		int _rowe;
		if (height >= h) {
			_y = (height - h) / 2;
			_rows = 0;
			_rowe = h -1;
		} else {
			_y = 0;
			_rows = (h - height) / 2;
			_rowe = _rows + height - 1;
		}
		//ESP_LOGD(__FUNCTION__,"_y=%d _rows=%d _rowe=%d", _y, _rows, _rowe);

		uint8_t *sdbuffer = (uint8_t*)malloc(rowSize); // pixel buffer
		uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _w); // tft buffer

		int debug = 0; // number of logging output
		for (int row=0; row<h; row++) { // For each scanline...
			if (row < _rows || row > _rowe) continue;
			// Seek to start of scan line.	It might seem labor-
			// intensive to be doing this on every line, but this
			// method covers a lot of gritty details like cropping
			// and scanline padding.  Also, the seek only takes
			// place if the file position actually needs to change
			// (avoids a lot of cluster math in SD library).
			// Bitmap is stored bottom-to-top order (normal BMP)
			int pos = result->header.offset + (h - 1 - row) * rowSize;
			//ESP_LOGD(__FUNCTION__,"pos=%d 0x%x", pos, pos);
			fseek(fp, pos, SEEK_SET);
			fread(sdbuffer, rowSize, 1, fp);
			int buffidx = 0;
			if (debug > 0) {
				//ESP_LOGI(__FUNCTION__, "sdbuffer");
				ESP_LOG_BUFFER_HEXDUMP(__FUNCTION__, sdbuffer, rowSize, ESP_LOG_INFO);
			}

			//int buffidx = sizeof(sdbuffer); // Force buffer reload

			int index = 0;
			uint8_t mask = 0x80;
			for (int col=0; col<w; col++) { // For each pixel...
				if (col < _cols || col > _cole) continue;
				// Convert pixel from BMP to TFT format, push to display
				colors[index] = BLACK;
				if ( (sdbuffer[buffidx] & mask) != 0) colors[index] = WHITE;
				index++;
				mask = mask >> 1;
				if (mask == 0x00) {
					buffidx++;
					mask = 0x80;
				}

				//uint8_t b = sdbuffer[buffidx++];
				//uint8_t g = sdbuffer[buffidx++];
				//uint8_t r = sdbuffer[buffidx++];
				//colors[index++] = rgb565_conv(r, g, b);
			} // end for col
			//ESP_LOGD(__FUNCTION__,"lcdDrawMultiPixels _x=%d _y=%d row=%d",_x, _y, row);
			if (debug > 0) {
				//ESP_LOGI(__FUNCTION__, "colors");
				ESP_LOG_BUFFER_HEXDUMP(__FUNCTION__, colors, _w*2, ESP_LOG_INFO);
			}
			lcdDrawMultiPixels(_x, _y, _w, colors);
			debug--;
			_y++;
		} // end for row
		lcdUpdateVScreen();
		free(sdbuffer);
		free(colors);
	} // end if
	free(result);
	fclose(fp);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t JPEGTest(char * file, int width, int height) {
	ESP_LOGI(TAG, "Start JPEGTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdSetFontDirection(0);
	lcdFillScreen(BLACK);


	pixel_jpeg **pixels;
	uint16_t imageWidth;
	uint16_t imageHeight;
	esp_err_t err = decode_jpeg(&pixels, file, width, height, &imageWidth, &imageHeight);
	//ESP_LOGD(__FUNCTION__, "decode_image err=%d imageWidth=%d imageHeight=%d", err, imageWidth, imageHeight);
	if (err == ESP_OK) {

		uint16_t _width = width;
		uint16_t _cols = 0;
		if (width > imageWidth) {
			_width = imageWidth;
			_cols = (width - imageWidth) / 2;
		}
		//ESP_LOGD(__FUNCTION__, "_width=%d _cols=%d", _width, _cols);

		uint16_t _height = height;
		uint16_t _rows = 0;
		if (height > imageHeight) {
			_height = imageHeight;
			_rows = (height - imageHeight) / 2;
		}
		//ESP_LOGD(__FUNCTION__, "_height=%d _rows=%d", _height, _rows);
		uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _width);

#if 0
		for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
				pixel_jpeg pixel = pixels[y][x];
				uint16_t color = rgb565_conv(pixel.red, pixel.green, pixel.blue);
				lcdDrawPixel(x+_cols, y+_rows, color);
			}
			vTaskDelay(1);
		}
#endif

		for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
				//pixel_jpeg pixel = pixels[y][x];
				//colors[x] = rgb565_conv(pixel.red, pixel.green, pixel.blue);
				colors[x] = pixels[y][x];
			}
			lcdDrawMultiPixels(_cols, y+_rows, _width, colors);
			vTaskDelay(1);
		}
		lcdUpdateVScreen();
		free(colors);
		release_image(&pixels, width, height);
		//ESP_LOGD(__FUNCTION__, "Finish");
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t PNGTest(char * file, int width, int height) {
	ESP_LOGI(TAG, "Start PNGTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	lcdSetFontDirection(0);
	lcdFillScreen(BLACK);

	// open PNG file
	FILE* fp = fopen(file, "rb");
	if (fp == NULL) {
			ESP_LOGW(__FUNCTION__, "File not found [%s]", file);
			return 0;
	}

	char buf[1024];
	size_t remain = 0;
	int len;

	pngle_t *pngle = pngle_new(width, height);

	pngle_set_init_callback(pngle, png_init);
	pngle_set_draw_callback(pngle, png_draw);
	pngle_set_done_callback(pngle, png_finish);

	double display_gamma = 2.2;
	pngle_set_display_gamma(pngle, display_gamma);


	while (!feof(fp)) {
			if (remain >= sizeof(buf)) {
					ESP_LOGE(__FUNCTION__, "Buffer exceeded");
					while(1) vTaskDelay(1);
			}

			len = fread(buf + remain, 1, sizeof(buf) - remain, fp);
			if (len <= 0) {
					//printf("EOF\n");
					break;
			}

			int fed = pngle_feed(pngle, buf, remain + len);
			if (fed < 0) {
					ESP_LOGE(__FUNCTION__, "ERROR; %s", pngle_error(pngle));
					while(1) vTaskDelay(1);
			}

			remain = remain + len - fed;
			if (remain > 0) memmove(buf, buf + fed, remain);
	}

	fclose(fp);

	uint16_t _width = width;
	uint16_t _cols = 0;
	if (width > pngle->imageWidth) {
			_width = pngle->imageWidth;
			_cols = (width - pngle->imageWidth) / 2;
	}
	//ESP_LOGD(__FUNCTION__, "_width=%d _cols=%d", _width, _cols);

	uint16_t _height = height;
	uint16_t _rows = 0;
	if (height > pngle->imageHeight) {
			_height = pngle->imageHeight;
			_rows = (height - pngle->imageHeight) / 2;
	}
	//ESP_LOGD(__FUNCTION__, "_height=%d _rows=%d", _height, _rows);
	uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _width);

#if 0
	for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
					pixel_png pixel = pngle->pixels[y][x];
					uint16_t color = rgb565_conv(pixel.red, pixel.green, pixel.blue);
					lcdDrawPixel(x+_cols, y+_rows, color);
			}
	}
#endif

	for(int y = 0; y < _height; y++){
		for(int x = 0;x < _width; x++){
			//pixel_png pixel = pngle->pixels[y][x];
			//colors[x] = rgb565_conv(pixel.red, pixel.green, pixel.blue);
			colors[x] = pngle->pixels[y][x];
		}
		lcdDrawMultiPixels(_cols, y+_rows, _width, colors);
		vTaskDelay(1);
	}
	lcdUpdateVScreen();
	free(colors);
	pngle_destroy(pngle, width, height);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	////ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

TickType_t CodeTest(FontxFile *fx, int width, int height) {
	ESP_LOGI(TAG, "Start CodeTest");
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	////ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	uint8_t xmoji = width / fontWidth;
	uint8_t ymoji = height / fontHeight;
	////ESP_LOGI(__FUNCTION__,"xmoji=%d ymoji=%d",xmoji, ymoji);


	uint16_t color;
	lcdFillScreen(BLACK);
	uint8_t code;

	color = CYAN;
	lcdSetFontDirection(0);
	code = 0xA0;
	for(int y=0;y<ymoji;y++) {
		uint16_t xpos = 0;
		uint16_t ypos =  fontHeight*(y+1)-1;
		for(int x=0;x<xmoji;x++) {
			xpos = lcdDrawCode(fx, xpos, ypos, code, color);
			if (code == 0xFF) break;
			code++;
		}
	}
	lcdUpdateVScreen();
	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	////ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_PERIOD_MS);
	return diffTick;
}

void lcdTest(void *param) {
	ESP_LOGI(TAG, "Start Display Test");
	for(;;) {		
		FillTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		ColorBarTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		ArrowTest(fx16G, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(4000/portTICK_PERIOD_MS);

		LineTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		CircleTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		RoundRectTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		RectAngleTest(lcdGetWidth(), lcdGetHeight());
		vTaskDelay(1000/portTICK_PERIOD_MS);

		TriangleTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		DirectionTest(fx16G, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		HorizontalTest(fx16G, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		VerticalTest(fx16G, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		FillRectTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		ColorTest(lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		CodeTest(fx32G, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		CodeTest(fx32L, lcdGetWidth(), lcdGetHeight());
		lcdUpdateVScreen();
		vTaskDelay(1000/portTICK_PERIOD_MS);

		char file[32];
		strcpy(file, "/spiffs/image.bmp");
		BMPTest(file, lcdGetWidth(), lcdGetHeight());
		vTaskDelay(1000/portTICK_PERIOD_MS);

		strcpy(file, "/spiffs/esp32.jpeg");
		JPEGTest(file, lcdGetWidth(), lcdGetHeight());
		vTaskDelay(1000/portTICK_PERIOD_MS);

		strcpy(file, "/spiffs/esp_logo.png");
		PNGTest(file, 240, 240);
		vTaskDelay(1000/portTICK_PERIOD_MS);

		strcpy(file, "/spiffs/qrcode.bmp");
		QRTest(file, lcdGetWidth(), lcdGetHeight());
		vTaskDelay(1000/portTICK_PERIOD_MS);

		//Multi Font Test
		ESP_LOGI(TAG, "Start MultiFontTest");
		uint16_t color;
		uint8_t ascii[40];
		uint16_t margin = 10;
		lcdFillScreen(BLACK);
		color = WHITE;
		lcdSetFontDirection(0);
		uint16_t xpos = 0;
		uint16_t ypos = 15;
		int xd = 0;
		int yd = 1;
		if(lcdGetWidth() < lcdGetHeight()) {
			lcdSetFontDirection(0);
			xpos = (lcdGetWidth()-1)-16;
			ypos = 0;
			xd = 1;
			yd = 0;
		}
		strcpy((char *)ascii, "16Dot Gothic Font");
		lcdDrawString(fx16G, xpos, ypos, ascii, color);

		xpos = xpos - (24 * xd) - (margin * xd);
		ypos = ypos + (16 * yd) + (margin * yd);
		strcpy((char *)ascii, "24Dot Gothic Font");
		lcdDrawString(fx24G, xpos, ypos, ascii, color);

		xpos = xpos - (32 * xd) - (margin * xd);
		ypos = ypos + (24 * yd) + (margin * yd);
		//if (lcdGetWidth() >= 240) {
		strcpy((char *)ascii, "32Dot Gothic Font");
		lcdDrawString(fx32G, xpos, ypos, ascii, color);
		xpos = xpos - (32 * xd) - (margin * xd);;
		ypos = ypos + (32 * yd) + (margin * yd);
		//}

		xpos = xpos - (10 * xd) - (margin * xd);
		ypos = ypos + (10 * yd) + (margin * yd);
		strcpy((char *)ascii, "16Dot Mincyo Font");
		lcdDrawString(fx16M, xpos, ypos, ascii, color);

		xpos = xpos - (24 * xd) - (margin * xd);;
		ypos = ypos + (16 * yd) + (margin * yd);
		strcpy((char *)ascii, "24Dot Mincyo Font");
		lcdDrawString(fx24M, xpos, ypos, ascii, color);

		//if (lcdGetWidth() >= 240) {
		xpos = xpos - (32 * xd) - (margin * xd);;
		ypos = ypos + (24 * yd) + (margin * yd);
		strcpy((char *)ascii, "32Dot Mincyo Font");
		lcdDrawString(fx32M, xpos, ypos, ascii, color);
		//}
		lcdSetFontDirection(0);
		lcdUpdateVScreen();
		//ESP_LOGI(TAG, "MultiFont Done");
		vTaskDelay(1000/portTICK_PERIOD_MS);
		
	}
}
#endif

void showJuventusLogo() {
	pixel_jpeg **pixels;
	uint16_t imageWidth;
	uint16_t imageHeight;
	uint16_t width = lcdGetWidth();
	uint16_t height = lcdGetHeight();
	char file[32];
	strcpy(file, "/spiffs/eduboard1.jpeg");
	esp_err_t err = decode_jpeg(&pixels, file, width, height, &imageWidth, &imageHeight);
	if (err == ESP_OK) {
		uint16_t _width = width;
		uint16_t _cols = 0;
		if (width > imageWidth) {
			_width = imageWidth;
			_cols = (width - imageWidth) / 2;
		}
		uint16_t _height = height;
		uint16_t _rows = 0;
		if (height > imageHeight) {
			_height = imageHeight;
			_rows = (height - imageHeight) / 2;
		}
		uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _width);
		for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
				colors[x] = pixels[y][x];
			}
			lcdDrawMultiPixels(_cols, y+_rows, _width, colors);
			// vTaskDelay(1);
		}
		free(colors);
		release_image(&pixels, width, height);
	}
}

void showVersionString() {
	char mytext[20];
	float version = EDUBOARD2_HWVERSION;
	sprintf((char *)mytext, "Eduboard V%.1f", version);
	#ifdef CONFIG_LCD_RESOLUTION_240x240
	lcdDrawString(fx16M, 25, 200, &mytext[0], WHITE);
	#endif
    #ifdef CONFIG_LCD_RESOLUTION_240x320
	lcdDrawString(fx16M, 25, 235, &mytext[0], WHITE);
	#endif
    #ifdef CONFIG_LCD_RESOLUTION_320x480
	lcdDrawString(fx24Comic, 55, 300, &mytext[0], WHITE);
	#endif
	
}

void eduboard_init_lcd() {
    ESP_LOGI(TAG, "Init LCD...");
	lcd_init();
	ESP_LOGI(TAG, "Init LCD Done.");
	rotation_t screenRotation = rot_0;
	#if SCREEN_ROTATION == 0
	screenRotation = rot_0;
	#elif SCREEN_ROTATION == 90
	screenRotation = rot_90;
	#elif SCREEN_ROTATION == 180
	screenRotation = rot_180;
	#elif SCREEN_ROTATION == 270
	screenRotation = rot_270;
	#endif
	#ifdef CONFIG_USE_VSCREEN
	ESP_LOGI(TAG, "Init VScreen...");
	lcdSetupVScreen(screenRotation);
	#endif
	lcdBacklightOn();
	lcdFillScreen(BLACK);
	#ifdef CONFIG_USE_VSCREEN
	lcdUpdateVScreen();
    ESP_LOGI(TAG, "Init VScreen Done.");
	#endif
	#ifdef CONFIG_LCD_TEST
	xTaskCreate(lcdTest, "LCD_TEST", 2048*6, NULL, 2, NULL);
	#else
	lcdFillScreen(BLACK);
	showJuventusLogo();
	showVersionString();
	lcdUpdateVScreen();
	#endif
}

#endif