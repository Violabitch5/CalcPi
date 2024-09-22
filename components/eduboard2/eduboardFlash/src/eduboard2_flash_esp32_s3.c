#include "../../eduboard2.h"
#include "../eduboard2_flash.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"

#ifndef CONFIG_ENABLE_FLASH
#include "../lfs/lfs.h"
#endif

#include "w25.h"

#define FLASH_FREQ_MHZ      SPI_MASTER_FREQ_10M

#define TAG "Flash_driver"

int storage_lfs_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int storage_lfs_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int storage_lfs_erase(const struct lfs_config *cfg, lfs_block_t block);
int storage_lfs_sync(const struct lfs_config *cfg);

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

#define BLOCK_SIZE 4096

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read = storage_lfs_read,
    .prog = storage_lfs_prog,
    .erase = storage_lfs_erase,
    .sync = storage_lfs_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = BLOCK_SIZE,
    .block_count = 1024, // 32Mbit
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};

int storage_lfs_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t addr = (block * BLOCK_SIZE) + off;
    w25_read_flash(addr, buffer, size);

    return 0;
}

int storage_lfs_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t addr = (block * BLOCK_SIZE) + off;
    w25_write_page(addr, (uint8_t *)buffer, size);

    return 0;
}

int storage_lfs_erase(const struct lfs_config *cfg, lfs_block_t block)
{
    uint32_t addr = block * BLOCK_SIZE;
    w25_erase_page(addr);

    return 0;
}

int storage_lfs_sync(const struct lfs_config *cfg)
{
    return 0;
}

void storage_handle_boot_counter(void)
{
    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // print the boot count
    ESP_LOGI(TAG, "boot_count: %d", (int)(boot_count));
}

void storage_handle_fs_list(void)
{
    ESP_LOGI(TAG, "Storage content: ");

    lfs_dir_t dir;
    struct lfs_info info;
    if (lfs_dir_open(&lfs, &dir, "/") == LFS_ERR_OK)
    {
        while (lfs_dir_read(&lfs, &dir, &info) > 0)
        {
            if (info.type == LFS_TYPE_REG)
            {
                ESP_LOGI(TAG, "File: %s", info.name);
            }
        }
        lfs_dir_close(&lfs, &dir);
    }
}

spi_device_handle_t dev_flash_spi;

void flash_checkConnection() {
    w25_read_jedec_id();

    storage_handle_boot_counter();
    storage_handle_fs_list();
}

void eduboard_init_flash() {
    ESP_LOGI(TAG, "init flash...");    
    gpspi_init(&dev_flash_spi, GPIO_MOSI, GPIO_MISO, GPIO_SCK, GPIO_FLASH_DAC_CS, FLASH_FREQ_MHZ, false);
    w25_init(&dev_flash_spi);

    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err)
    {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

    ESP_LOGI(TAG, "init flash done");
}