#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <driver/gpio.h>

#include "../gpspi.h"

#define TAG "GPSPI"

bool gpspi_read_write_data(spi_device_handle_t* handle, uint8_t* txdata, uint8_t* rxdata, uint32_t len) {
	spi_transaction_t SPITransaction;
	esp_err_t ret;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
    // SPITransaction.cmd = cmd;
	SPITransaction.length = len * 8;
	SPITransaction.tx_buffer = txdata;
    SPITransaction.rxlength = len * 8;
    SPITransaction.rx_buffer = rxdata;
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	ret = spi_device_polling_transmit( *handle, &SPITransaction );
	spi_device_release_bus(*handle);
	assert(ret==ESP_OK); 
	return true;
}

bool gpspi_write_data(spi_device_handle_t* handle, uint8_t* data, uint32_t len) {
    spi_transaction_t SPITransaction;
	esp_err_t ret;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = len * 8;
	SPITransaction.tx_buffer = data;
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	ret = spi_device_transmit( *handle, &SPITransaction );
	spi_device_release_bus(*handle);
	assert(ret==ESP_OK); 
	return true;
}
bool gpspi_write_cmd_data(spi_device_handle_t* handle, uint8_t cmd, uint8_t* data, uint32_t len) {
    spi_transaction_t SPITransaction;
	esp_err_t ret;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
    SPITransaction.cmd = cmd;
	SPITransaction.length = len * 8;
	SPITransaction.tx_buffer = data;
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	ret = spi_device_transmit( *handle, &SPITransaction );
	spi_device_release_bus(*handle);
	assert(ret==ESP_OK);
	return true;
}
bool gpspi_read_cmd_data(spi_device_handle_t* handle, uint8_t cmd, uint8_t* data, uint32_t len) {
    spi_transaction_t SPITransaction;
	esp_err_t ret;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
    SPITransaction.cmd = cmd;
    SPITransaction.rxlength = len * 8;
    SPITransaction.rx_buffer = data;
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	ret = spi_device_polling_transmit( *handle, &SPITransaction );
	spi_device_release_bus(*handle);
	assert(ret==ESP_OK); 
	return true;
}

void gpspi_init(spi_device_handle_t* handle, int pinMOSI, int pinMISO, int pinSCK, int pinCS, uint32_t frequency, bool halfduplex) {
	static bool spi_initialized = false;
    ESP_LOGI(TAG, "Init SPI Device...");
    esp_err_t ret;
    ESP_LOGI(TAG, "init cs pin");
	if(pinCS >= 0 ) {
		gpio_reset_pin(pinCS);
		gpio_set_direction(pinCS, GPIO_MODE_OUTPUT);
		gpio_set_level(pinCS, 1);
    }
    
    spi_bus_config_t buscfg = {
		.mosi_io_num = pinMOSI,
		.miso_io_num = pinMISO,
		.sclk_io_num = pinSCK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};
	if(spi_initialized == false) {
		ESP_LOGI(TAG, "init spi bus config");
		ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
		spi_initialized = true;
		assert(ret==ESP_OK);
	}

    ESP_LOGI(TAG, "init spi device config");
	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));
	devcfg.clock_speed_hz = frequency;
	devcfg.queue_size = 4;
	devcfg.mode = 0;
	if(halfduplex) {
		devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	}
	
    ESP_LOGI(TAG, "set spi cs pin");
	if ( pinCS >= 0 ) {
		devcfg.spics_io_num = pinCS;
	} else {
		devcfg.spics_io_num = -1;
	}
    ESP_LOGI(TAG, "add spi device");
    ret = spi_bus_add_device( HOST_ID, &devcfg, handle);
    assert(ret==ESP_OK);
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	spi_device_release_bus(*handle);
    ESP_LOGI(TAG, "SPI Device init done");
}

bool gpspi_write_data_nonblocking(spi_device_handle_t* handle, uint8_t* data, uint32_t len) {
    spi_transaction_t SPITransaction;
	esp_err_t ret;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = len * 8;
	SPITransaction.tx_buffer = data;
	ret = spi_device_polling_transmit(*handle, &SPITransaction);
	assert(ret==ESP_OK); 
	return true;
}
void gpspi_init_nonblocking(spi_device_handle_t* handle, int pinMOSI, int pinMISO, int pinSCK, int pinCS, uint32_t frequency, bool halfduplex) {
	static bool spi_initialized = false;
    ESP_LOGI(TAG, "Init SPI Device nonblocking...");
    esp_err_t ret;
    ESP_LOGI(TAG, "init cs pin");
	if(pinCS >= 0 ) {
		gpio_reset_pin(pinCS);
		gpio_set_direction(pinCS, GPIO_MODE_OUTPUT);
		gpio_set_level(pinCS, 1);
    }
    
    spi_bus_config_t buscfg = {
		.mosi_io_num = pinMOSI,
		.miso_io_num = pinMISO,
		.sclk_io_num = pinSCK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};
	if(spi_initialized == false) {
		ESP_LOGI(TAG, "init spi bus config");
		ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
		spi_initialized = true;
		assert(ret==ESP_OK);
	}

    ESP_LOGI(TAG, "init spi device config");
	spi_device_interface_config_t devcfg;
	memset(&devcfg, 0, sizeof(devcfg));
	devcfg.clock_speed_hz = frequency;
	devcfg.queue_size = 4;
	devcfg.mode = 0;
	if(halfduplex) {
		devcfg.flags = SPI_DEVICE_HALFDUPLEX;
	}
	
    ESP_LOGI(TAG, "set spi cs pin");
	if ( pinCS >= 0 ) {
		devcfg.spics_io_num = pinCS;
	} else {
		devcfg.spics_io_num = -1;
	}
    ESP_LOGI(TAG, "add spi device");
    ret = spi_bus_add_device( HOST_ID, &devcfg, handle);
    assert(ret==ESP_OK);
	ESP_LOGI(TAG, "Acquire Spi bus on do not release it anymore");
	spi_device_acquire_bus(*handle, portMAX_DELAY);
	
    ESP_LOGI(TAG, "SPI Device init done");
}