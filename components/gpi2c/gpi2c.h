#pragma once

#include <esp_err.h>

#define I2C_FREQ_HZ 100000 // Max 1MHz for esp-idf

esp_err_t gpi2c_readRegister(uint8_t addr, uint8_t reg, uint8_t * data, uint8_t len);
esp_err_t gpi2c_writeRegister(uint8_t addr, uint8_t reg, uint8_t * data, uint8_t len);
esp_err_t gpi2c_writeData(uint8_t addr, uint8_t * data, uint8_t len);
void gpi2c_init(int pinI2C_SDA, int pinI2C_SCL, uint32_t frequency);