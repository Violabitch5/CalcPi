#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "w25.h"
#include "gpspi.h"

#define TAG "W25_Flash"

// FLASH - START
#define W25_JEDEC_ID_COMMAND 0x9F
#define W25_STATUS_COMMAND   0x05
#define W25_WRITE_ENABLE     0x06
#define W25_READ_DATA_COMMAND 0x03
#define W25_PAGE_PROGRAM_COMMAND  0x02
#define W25_SECTOR_ERASE_COMMAND  0x20

// #define W25_DEBUG 1

spi_device_handle_t* spidevice;

void w25_read_jedec_id(void)
{
#ifdef W25_DEBUG
    ESP_LOGW(TAG, "Reading JEDEC ID \n");
#endif

    uint8_t buffer[4];
    buffer[0] = W25_JEDEC_ID_COMMAND;

    gpspi_read_write_data(spidevice, buffer, buffer, 4);

#ifdef W25_DEBUG
    ESP_LOGW(TAG, "JEDEC ID: 0x%02X%02X%02X \n", buffer[0 + 1], buffer[1 + 1], buffer[2 + 1]);
#endif
}

uint8_t w25_read_status(void)
{
    uint8_t mybuffer[2];
    mybuffer[0] = W25_STATUS_COMMAND;

    gpspi_read_write_data(spidevice, mybuffer, mybuffer, 2);

    return mybuffer[0 + 1];
}

void w25_write_enable(void)
{
    uint8_t mybuffer[1];
    mybuffer[0] = W25_WRITE_ENABLE;

    gpspi_read_write_data(spidevice, mybuffer, mybuffer, 1);
}

void w25_read_flash(uint32_t address, uint8_t *data, uint32_t size)
{
#ifdef W25_DEBUG
    ESP_LOGW(TAG, "Reading Flash start %X\n", (unsigned int)(address));
#endif

    uint8_t mybuffer[256 + 4];
    mybuffer[0] = W25_READ_DATA_COMMAND;

    mybuffer[1] = (address >> 16) & 0xFF;
    mybuffer[2] = (address >> 8) & 0xFF;
    mybuffer[3] = address & 0xFF;

    gpspi_read_write_data(spidevice, mybuffer, mybuffer, 4 + size);

    memcpy(data, &mybuffer[4], size);

    while (w25_read_status() & 0x01);
}

void w25_write_page(uint32_t address, uint8_t *data, uint32_t size)
{
#ifdef W25_DEBUG
    ESP_LOGW(TAG, "Write Page start %X\n", (unsigned int)(address));
#endif

    w25_write_enable();

    uint8_t mybuffer[256 + 4];
    mybuffer[0] = W25_PAGE_PROGRAM_COMMAND;

    mybuffer[1] = (address >> 16) & 0xFF;
    mybuffer[2] = (address >> 8) & 0xFF;
    mybuffer[3] = address & 0xFF;

    memcpy(&mybuffer[4], data, size);

    gpspi_read_write_data(spidevice, mybuffer, mybuffer, 4 + size);

    while (w25_read_status() & 0x01);
}

void w25_erase_page(uint32_t address)
{
#ifdef W25_DEBUG
    ESP_LOGW(TAG, "Erase Page %X start \n", (unsigned int)(address));
#endif

    w25_write_enable();

    uint8_t mybuffer[32];
    mybuffer[0] = W25_SECTOR_ERASE_COMMAND;

    mybuffer[1] = (address >> 16) & 0xFF;
    mybuffer[2] = (address >> 8) & 0xFF;
    mybuffer[3] = address & 0xFF;

    gpspi_read_write_data(spidevice, mybuffer, mybuffer, 4);

    while (w25_read_status() & 0x01);
}

void w25_init(spi_device_handle_t* handle) {
    spidevice = handle;

    w25_read_jedec_id();
}
