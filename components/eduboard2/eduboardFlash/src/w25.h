#pragma once
#include "../../eduboard2.h"

void w25_read_jedec_id(void);
uint8_t w25_read_status(void);
void w25_write_enable(void);
void w25_read_flash(uint32_t address, uint8_t *data, uint32_t size);
void w25_write_page(uint32_t address, uint8_t *data, uint32_t size);
void w25_erase_page(uint32_t address);
void w25_init(spi_device_handle_t* handle);