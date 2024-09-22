/***************************************************************/
/*   eduboard_spiffs                                           */
/*   Mount Spiffs and load fonts                               */
/*                                                             */
/*                                                             */
/***************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"


#include "../eduboard2_spiffs.h"

#define TAG "Spiffs_Driver"

FontxFile fx16G[2];
FontxFile fx24G[2];
FontxFile fx32G[2];
FontxFile fx32L[2];

FontxFile fx16M[2];
FontxFile fx24M[2];
FontxFile fx32M[2];

FontxFile fx20_SIO8859_1[2];

FontxFile fx24Comic[2];

void eduboard_init_spiffs(void) {
	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 12,
		.format_if_mount_failed =true
	};

	// // Use settings defined above toinitialize and mount SPIFFS filesystem.
	// // Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total,&used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		//ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	// Load Font Files
	
	InitFontx(fx16G,"/spiffs/fonts/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/fonts/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/fonts/ILGH32XB.FNT",""); // 16x32Dot Gothic
	InitFontx(fx32L,"/spiffs/fonts/LATIN32B.FNT",""); // 16x32Dot Latin

	
	InitFontx(fx16M,"/spiffs/fonts/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/fonts/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/fonts/ILMH32XB.FNT",""); // 16x32Dot Mincyo

	InitFontx(fx20_SIO8859_1, "/spiffs/fonts/font10x20-ISO8859-1.fnt", "");

	InitFontx(fx24Comic, "/spiffs/fonts/COMIC24XB.FNT", ""); //24Dot Comic Sans
	
	ESP_LOGI(TAG, "Spiffs mounted successfully");
	ESP_LOGI(TAG, "All Fonts loaded");
}