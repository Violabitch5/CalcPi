#include "../../eduboard2.h"
#include "../eduboard2_sensor.h"

#define TAG "Sensor_TMP112_Driver"

/* Copyright 2016 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
/* TMP112 temperature sensor module for Chrome EC */

//#define I2C_FREQ_HZ 100000 // Max 1MHz for esp-idf



#define C_TO_K(temp_c) ((temp_c) + 273)
//#define TMP112_I2C_ADDR_FLAGS (0x48)
#define TMP112_I2C_ADDR_FLAGS (0x48)
#define TMP112_REG_TEMP	0x00
#define TMP112_REG_CONF	0x01
#define TMP112_REG_HYST	0x02
#define TMP112_REG_MAX	0x03

#define TMP112_RESOLUTION 12
#define TMP112_SHIFT1 (16 - TMP112_RESOLUTION)
#define TMP112_SHIFT2 (TMP112_RESOLUTION - 8)

static float temp_val_local;

static esp_err_t read_register16(uint8_t reg, uint16_t *r)
{
	return gpi2c_readRegister(TMP112_I2C_ADDR_FLAGS, reg, (uint8_t*)r, 2);
}
inline static esp_err_t write_register16(uint8_t reg, uint16_t data)
{
	return gpi2c_writeRegister(TMP112_I2C_ADDR_FLAGS, reg, (uint8_t*)&data, 2);
}


static int get_temp()
{
	esp_err_t err;
	uint16_t temp_raw = 0;
	err = read_register16(TMP112_REG_TEMP, &temp_raw);
	temp_raw = ((temp_raw << 8) | (temp_raw >> 8))>>4;
	// ESP_LOGI(TAG, "Temp read: %4x", temp_raw);
    if (err != ESP_OK)
		return err;
	if((temp_raw & 0x0800) == 0x0000) {
		//Positive Number:
		temp_val_local = (0.0625 * temp_raw);
	} else {
		// ESP_LOGI(TAG, "1: %4X", temp_raw);
		temp_raw = ((~temp_raw)+1)&0x0FFF;
		// ESP_LOGI(TAG, "2: %4X", temp_raw);
		temp_val_local = -(0.0625 * temp_raw);
		// ESP_LOGI(TAG, "3: %f", temp_val_local);

	}
	return ESP_OK;
}
float tmp112_get_value(void)
{
	get_temp();
	return temp_val_local;
}

void eduboard_init_tmp112(void)
{
	gpi2c_init(GPIO_I2C_SDA, GPIO_I2C_SCL, 400000);
    uint16_t tmp;
    read_register16(TMP112_REG_CONF, &tmp);
	ESP_LOGI(TAG, "TMP112 read: %4X", (int)tmp);
	tmp = 0x8062;
	ESP_LOGI(TAG, "TMP112 writeback: %4X", (int)tmp);
	write_register16(TMP112_REG_CONF, tmp);
}