#pragma once

#include "lcdDriver.h"

// #define HOST_ID SPI2_HOST

extern TFT_t * lcddevice;

void st7789_spi_master_init(TFT_t * dev, int16_t PIN_MOSI, int16_t PIN_SCLK, int16_t PIN_CS, int16_t PIN_DC, int16_t PIN_RESET, int16_t PIN_BL);
bool st7789_spi_master_write_byte(spi_device_handle_t SPIHandle, const uint8_t* Data, size_t DataLength);
bool st7789_spi_master_write_command(uint8_t cmd);
bool st7789_spi_master_write_data_byte(uint8_t data);
bool st7789_spi_master_write_data_word(uint16_t data);
bool st7789_spi_master_write_addr(uint16_t addr1, uint16_t addr2);
bool st7789_spi_master_write_color(uint16_t color, uint16_t size);
bool st7789_spi_master_write_colors(uint16_t * colors, uint16_t size);

void st7789_init(TFT_t * dev, int width, int height, int offsetx, int offsety);

void st7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void st7789_DrawMultiPixels(uint16_t x, uint16_t y, uint16_t size, uint16_t * colors);
void st7789_DrawArea(uint16_t x, uint16_t y, uint16_t size_x, uint16_t size_y, uint16_t * colors);
void st7789_DrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void st7789_DisplayOff();
void st7789_DisplayOn();
void st7789_InversionOff();
void st7789_InversionOn();